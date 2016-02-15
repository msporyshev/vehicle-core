#include "task.h"
#include "task_factory.h"

enum class State
{
    FixHeading,
    FixPitch,
    MoveForward,
    Etc,
    Terminal,
};

class SampleTask: public Task<State>
{
public:
    SampleTask(const YamlReader& cfg, ipc::Communicator& comm): Task<State>(cfg, comm, State::FixHeading)
    {
        state_machine_.REG_STATE(State::Etc, handle_etc, 100, State::Terminal);
        state_machine_.REG_STATE(State::MoveForward, handle_move_fwd, 100, State::Etc);
        state_machine_.REG_STATE(State::FixPitch, handle_fix_pitch, 100, State::MoveForward);
        state_machine_.REG_STATE(State::FixHeading, handle_fix_heading, 100, State::FixPitch);
    }

    State handle_fix_heading()
    {
        motion_.fix_heading(0);
        return State::FixHeading;
    }
    State handle_fix_pitch()
    {
        motion_.fix_pitch();
        return State::FixPitch;
    }
    State handle_move_fwd()
    {
        motion_.move_forward(10, 10);
        return State::MoveForward;
    }
    State handle_etc()
    {
        motion_.move_right(10, 10);
        motion_.move_left(10, 10);
        motion_.move_backward(10, 10);
        motion_.move_up(10);
        motion_.move_down(10);
        return State::Etc;
    }
};

REGISTER_TASK(SampleTask, sample_task);
