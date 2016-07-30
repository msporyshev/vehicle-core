#include "task.h"
#include "task_factory.h"

#include <utils/math_u.h>

namespace {

enum class State
{
    Init,
    Terminal
};

} // namespace

class ThrustForwardTask: public Task<State>
{
public:
    ThrustForwardTask(const YamlReader& cfg, ipc::Communicator& comm): Task<State>(cfg, comm, State::Init)
    {
        state_machine_.REG_STATE(State::Init, handle_init, timeout_thrust_.get(), State::Terminal);
    }

    State handle_init()
    {
        motion_.thrust_forward(thrust_.get(), timeout_thrust_.get(), WaitMode::WAIT);

        return State::Terminal;
    }

private:
    AUTOPARAM(double, timeout_thrust_);
    AUTOPARAM(double, thrust_);
};

REGISTER_TASK(ThrustForwardTask, thrust_forward_task);
