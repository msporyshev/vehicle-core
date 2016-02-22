#pragma once

#include <libipc/ipc.h>
#include <navig/MsgNavigDepth.h>
#include <navig/MsgNavigAngles.h>
#include <navig/MsgNavigVelocity.h>

class Navigation
{
public:
    Navigation(ipc::Communicator& comm)
            : angle_sub_(comm.subscribe<navig::MsgNavigAngles>("navig"))
            , depth_sub_(comm.subscribe<navig::MsgNavigDepth>("navig"))
    {}

    double last_depth()
    {
        return depth_sub_.msg_wait().depth;
    }

    double last_head()
    {
        return angle_sub_.msg_wait().heading;
    }

private:
    ipc::Subscriber<navig::MsgNavigAngles> angle_sub_;
    ipc::Subscriber<navig::MsgNavigDepth> depth_sub_;
};