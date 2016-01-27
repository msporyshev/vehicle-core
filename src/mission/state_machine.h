#pragma once

#include <map>
#include <string>
#include <functional>
#include <type_traits>

#include <ros/ros.h>

#include <libauv/utils/basic.h>

template<typename Enum>
struct has_terminal
{
    template<typename T>
    char static test(decltype(T::Terminal)*);

    template<typename T>
    int static test(...);

    static const bool value = sizeof(test<Enum>(nullptr)) == sizeof(char);
};

template<typename State>
class StateMachine
{
public:
    static_assert(has_terminal<State>::value, "State enum should contain State::Terminal field");

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
        ROS_INFO_STREAM("Register state " << state_name
            << " with timeout " << timeout
            <<  ", fallback state " << handler_by_state_[fallback_state].name);
    }

    void switch_state_to(State state)
    {
        auto handler = handler_by_state_[state];

        if (state != cur_state_handler_.state) {
            ROS_INFO_STREAM("Switching state from " << cur_state_handler_.name << " to " << handler.name);
            cur_state_handler_ = handler;
            state_start_time_ = fixate_time();
        }
    }

    void process_state()
    {
        if (fixate_time() - state_start_time_ < cur_state_handler_.timeout) {
            switch_state_to(cur_state_handler_.callback());
        } else {
            ROS_INFO("Fall back by timeout");
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

    void handle_terminal()
    {
        ROS_INFO("State is State::Terminal, current task has been finished");
        return State::Terminal;
    }
};

#define REG_STATE(STATE, METHOD_NAME, TIMEOUT, FALLBACK_STATE) \
    register_state(STATE, #STATE, &std::remove_pointer<decltype(OBJECT_PTR)>::type::METHOD_NAME, this, TIMEOUT, FALLBACK_STATE)