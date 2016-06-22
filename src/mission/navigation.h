#pragma once

#include <libipc/ipc.h>
#include <navig/MsgDepth.h>
#include <navig/MsgAngles.h>
#include <navig/MsgVelocity.h>

class Navigation
{
public:
    Navigation(ipc::Communicator& comm)
            : angle_sub_(comm.subscribe<navig::MsgAngles>("navig"))
            , depth_sub_(comm.subscribe<navig::MsgDepth>("navig"))
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
    ipc::Subscriber<navig::MsgAngles> angle_sub_;
    ipc::Subscriber<navig::MsgDepth> depth_sub_;
};