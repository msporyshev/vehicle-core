#pragma once

#include <map>
#include <string>
#include <functional>
#include <type_traits>

#include <ros/ros.h>

#include <libauv/utils/basic.h>
#include <motion/motion_client/robosub_motion_client.h>
#include <libipc/ipc.h>
#include <config_reader/yaml_reader.h>

#include "camera_model.h"
#include "commands.h"
#include "navigation.h"


enum class Kitty
{
    Happy,
    Sad,
    Angry,
    Surprised,
};

template<typename Enum>
struct has_terminal
{
    template<typename T>
    char static test(decltype(T::Terminal)*);

    template<typename T>
    int static test(...);

    static const bool value = sizeof(test<Enum>(nullptr)) == sizeof(char);
};

class TaskBase
{
public:
    TaskBase(const YamlReader& cfg, ipc::Communicator& comm)
            : cfg_(cfg)
            , motion_(comm)
            , cmd_(comm)
            , navig_(comm)
            , front_camera_(YamlReader("front_camera.yml", "mission"))
            , bottom_camera_(YamlReader("bottom_camera.yml", "mission"))
    {}

    virtual ~TaskBase() {}

    virtual Kitty run() = 0;
protected:
    YamlReader cfg_;

    RobosubMotionClient motion_;
    Commands cmd_;
    Navigation navig_;

    CameraModel front_camera_;
    CameraModel bottom_camera_;

    AUTOPARAM(int, timeout_total_);
};


template<typename State>
class Task: public TaskBase
{
public:
    class StateMachine
    {
    public:

        #define REG_STATE(STATE, METHOD_NAME, TIMEOUT, FALLBACK_STATE) \
            register_state(STATE, \
                #STATE, \
                &std::remove_pointer<decltype(this)>::type::METHOD_NAME, \
                this, \
                TIMEOUT, \
                FALLBACK_STATE)

        static_assert(has_terminal<State>::value, "State enum should contain State::Terminal field");

        StateMachine()
        {
            REG_STATE(State::Terminal, handle_terminal, 10, State::Terminal);
            cur_state_handler_ = handler_by_state_[State::Terminal];
        }


        using StateCallback = std::function<State ()>;

        struct StateHandler
        {
            State state;
            std::string name;
            State fallback_state;
            StateCallback callback;
            int timeout;
        };

        template<typename Obj>
        void register_state(State state, std::string state_name, State (Obj::*handler)(), Obj* object,
                int timeout, State fallback_state)
        {
            handler_by_state_[state] = {state, state_name, fallback_state, std::bind(handler, object), timeout};
            ROS_INFO_STREAM("Register state " << state_name << "(" << static_cast<size_t>(state) << ")"
                << " with timeout " << timeout
                <<  ", fallback state " << handler_by_state_[fallback_state].name
                << "(" << static_cast<size_t>(fallback_state) << ")");
        }

        void switch_state_to(State state)
        {
            auto handler = handler_by_state_[state];

            if (state != cur_state_handler_.state) {
                ROS_INFO_STREAM("Switching state to "
                    << handler.name << "(" << (int)handler.state << ")");
                cur_state_handler_ = handler;
                state_start_time_ = fixate_time();
            }
        }

        void process_state()
        {
            if (fixate_time() - state_start_time_ < cur_state_handler_.timeout) {
                switch_state_to(cur_state_handler_.callback());
            } else {
                ROS_INFO("Fall back by timeout %d", cur_state_handler_.timeout);
                switch_state_to(cur_state_handler_.fallback_state);
            }
        }

        State cur_state()
        {
            return cur_state_handler_.state;
        }

        std::string cur_state_name()
        {
            return cur_state_handler_.name;
        }

    private:
        StateHandler cur_state_handler_;
        double state_start_time_;

        std::map<State, StateHandler> handler_by_state_;

        State handle_terminal()
        {
            ROS_INFO("State is State::Terminal, current task has been finished by success");
            return State::Terminal;
        }
    };



    Task(const YamlReader& cfg, ipc::Communicator& comm, State initial_state)
            : TaskBase(cfg, comm)
            , initial_state_(initial_state)
    {}

    Kitty run() override
    {
        state_machine_.switch_state_to(initial_state_);

        double start_time = fixate_time();
        while (fixate_time() - start_time < timeout_total_.get()) {
            ros::spinOnce();
            if (!ros::ok()) {
                return Kitty::Angry;
            }
            state_machine_.process_state();

            if (state_machine_.cur_state() == State::Terminal) {
                break;
            }
        }

        if (state_machine_.cur_state() != State::Terminal) {
            ROS_INFO("Task has been finished by timeout");
            return Kitty::Sad;
        }

        return Kitty::Happy;
    }


protected:
    StateMachine state_machine_;
    State initial_state_;
};