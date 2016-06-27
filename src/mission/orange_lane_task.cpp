#include "task.h"
#include "task_factory.h"

namespace {

enum class State
{
    Init,
    LaneSearch,
    FixLane,
    FollowLane,
    Terminal,
};

}

class OrangeLaneTask: public Task<State>
{
public:
    OrangeLaneTask(const YamlReader& cfg, ipc::Communicator& comm)
            : Task<State>(cfg, comm, State::Init)
    {
        state_machine_.REG_STATE(
            State::Init,
            handle_init,
            timeout_init_.get(),
            State::LaneSearch
            );

        state_machine_.REG_STATE(
            State::LaneSearch,
            handle_lane_search,
            timeout_lane_search_.get(),
            State::FollowLane
            );
    }

    State handle_init() {

    }

    State handle_lane_search() {

    }

    State handle_fix_lane() {

    }

    State handle_follow_lane() {

    }
private:

    AUTOPARAM(double, timeout_init_);
    AUTOPARAM(double, timeout_lane_search_);
};