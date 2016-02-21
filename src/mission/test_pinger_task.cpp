#include "task.h"
#include "task_factory.h"

#include <dsp/MsgBeacon.h>

enum class State
{
    Initialization,
    StabHeading,
    Terminal
};

class TestPingerTask: public Task<State>
{
public:
    TestPingerTask(const YamlReader& cfg, ipc::Communicator& com): Task<State>(cfg, com, State::Initialization) 
    {
        init_ipc(com);
        state_machine_.REG_STATE(State::Initialization, handle_initalization, timeout_init_.get(), State::StabHeading);
        state_machine_.REG_STATE(State::StabHeading, handle_stab, timeout_stab_.get(), State::Terminal);
    }

    void handle_pinger_found(const dsp::MsgBeacon& msg)
    {
        ROS_INFO_STREAM("Current pinger heading = " << msg.heading << "\n");
        pinger_heading_ = msg.heading;
        pinger_found_ = true;
    }

    State handle_initalization()
    {
        motion_.fix_pitch();
        motion_.fix_depth(start_depth_.get());
        return State::StabHeading;
    }

    State handle_stab()
    {
        if (pinger_found_) {
            motion_.fix_heading(pinger_heading_, WaitMode::DONT_WAIT);
        }
        return State::StabHeading;
    }

private:
    AUTOPARAM(double, start_depth_);
    AUTOPARAM(double, timeout_stab_);
    AUTOPARAM(double, timeout_init_);

    bool pinger_found_ = false;
    double pinger_heading_;

    void init_ipc(ipc::Communicator& com)
    {
        com.subscribe("dsp", &TestPingerTask::handle_pinger_found, this);
    }
};

REGISTER_TASK(TestPingerTask, test_pinger_task);