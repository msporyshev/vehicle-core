/**
\file
\brief Реализация модуля управления движителями

В данном файле находятся реализации методов, объявленных в tcu.h
Также здесь находится main

\defgroup tcu_node ДРК

\brief Данный нод предназначен для полноценной работы программной составляющей
Движительно-рулевого комплекса
*/

///@{
#include <cmath>

#include "tcu.h"

using namespace std;

const string Tcu::NODE_NAME = "tcu";
const int Tcu::silence_iterations = 100;

Tcu::Tcu(ipc::Communicator& communicator) :
    communicator_(communicator)
{
    init_ipc();   
}

Tcu::~Tcu()
{}

void Tcu::init_ipc()
{
    communicator_.subscribe_cmd(&Tcu::process_regul_msg, this, 1);
}


void Tcu::increase_loop_params()
{
    last_regul_msg_it_++;
}

void Tcu::reset_regul_msg_it()
{
    last_regul_msg_it_ = 0;
}


bool Tcu::need_stop_thrusters()
{
    return last_regul_msg_it_ > silence_iterations;
}


void Tcu::process_regul_msg(const tcu::CmdForce& msg)
{
    vector<double> values {msg.forward, msg.right, msg.down, msg.mforward, msg.mdown, msg.mright};
    bool is_nan_or_inf_recieved = false;

    for (auto & val : values) {
        if (!isfinite(val)) {
            is_nan_or_inf_recieved = true;
            break;
        }
    }

    if (is_nan_or_inf_recieved) {
        ROS_ERROR("Recieved NaN or Infinity value in CmdForce!");
    } else {
        reset_regul_msg_it();
        calc_new_thrusts(msg);
        calc_new_signals();
        send_thrusts();
    }
}


void Tcu::read_config(const int num_of_thrusters)
{
    XmlRpc::XmlRpcValue thrusters;

    ROS_ASSERT(ros::param::get("/tcu/max_force", max_force_));
    ROS_ASSERT(ros::param::get("/tcu/delta_force", delta_force_));    
    ROS_ASSERT(ros::param::get("/tcu/thrusters", thrusters));

    ROS_ASSERT_MSG(thrusters.size() == num_of_thrusters, "FAIL: Check amount of thrusters in config");

    for (auto i = 0; i < num_of_thrusters; ++i) {

        thrusters_[i]->location = thrusters[i]["params"]["location"] == "horizontal" ? LocationType::Horizontal : LocationType::Vertical;
        thrusters_[i]->direction = thrusters[i]["params"]["direction"] == "FORWARD" ? DirectionType::Forward : DirectionType::Backward;
        thrusters_[i]->propeller_type = thrusters[i]["params"]["propeller"] == "SYM" ? PropellerType::Symmetrical : PropellerType::Asymmetrical;
        thrusters_[i]->shoulder = thrusters[i]["params"]["shoulder"];
        thrusters_[i]->negative_factor = thrusters[i]["params"]["neg_koef"];
        thrusters_[i]->reverse = thrusters[i]["params"]["reverse"];
        thrusters_[i]->forward = thrusters[i]["params"]["forward"];
        thrusters_[i]->right = thrusters[i]["params"]["right"];
        thrusters_[i]->down = thrusters[i]["params"]["down"];
    }
}


void Tcu::stop_thrusters()
{
    for (auto & thruster : thrusters_) {
        thruster->thrust = 0;
        thruster->signal = 0;
    }
}

///@}