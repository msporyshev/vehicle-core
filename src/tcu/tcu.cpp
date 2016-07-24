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
#include <supervisor/CmdCan.h>
#include <cmath>

#include "tcu.h"
#include "matrix_inversion.h"

using namespace std;

namespace {
    int last_settings_it  = 0;
    int last_regul_msg_it = 0;

    int settings_id = 0;

    const int silence_iterations = 5;
    const int settings_iterations = 2;

    void increase_loop_params()
    {
        last_settings_it++;
        last_regul_msg_it++;
    }

    void reset_settings_it()
    {
        last_settings_it = 0;
    }

    void reset_regul_msg_it()
    {
        last_regul_msg_it = 0;
    }

    bool need_send_settings()
    {
        if (last_settings_it > settings_iterations) {
            settings_id = (settings_id + 1) % N;
            return true;
        }

        return false;
    }

    bool need_stop_thrusters()
    {
        return last_regul_msg_it > silence_iterations;
    }
}

const string Tcu::NODE_NAME = "tcu";

Tcu::Tcu(ipc::Communicator& communicator) :
    communicator_(communicator)
{
    init_ipc();
    read_config();
    normalize_config_values();
    calc_thrusters_distribution();
}

Tcu::~Tcu()
{}

void Tcu::init_ipc()
{
	this->can_send_pub_ = this->communicator_.advertise_cmd<supervisor::CmdCan>("supervisor");

	communicator_.subscribe_cmd(&Tcu::process_regul_msg, this, 1);
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


void Tcu::read_config()
{
    XmlRpc::XmlRpcValue thrusters;

    ROS_ASSERT(ros::param::get("/tcu/max_force", max_force_));
    ROS_ASSERT(ros::param::get("/tcu/delta_force", delta_force_));
    ROS_ASSERT(ros::param::get("/tcu/common_can_addr", common_can_addr_));
    ROS_ASSERT(ros::param::get("/tcu/thrusts", thrusts_));
    ROS_ASSERT(ros::param::get("/tcu/codes", codes_));
    ROS_ASSERT(ros::param::get("/tcu/thrusters", thrusters));


    ROS_ASSERT_MSG(codes_.size() == thrusts_.size(), "FAIL: Thrusts and codes have different size. Unable to interprete config");
    ROS_ASSERT_MSG(thrusts_.size() >= 2, "FAIL: Thrusts and codes should contain at least 2 numbers each");
    ROS_ASSERT_MSG(thrusters.size() == N, "FAIL: Check amount of thrusters in config");

    for (auto i = 0; i < N; ++i) {

        thrusters_[i].can_id = thrusters[i]["params"]["id"];
        thrusters_[i].location = thrusters[i]["params"]["location"] == "horizontal" ? LocationType::Horizontal : LocationType::Vertical;
        thrusters_[i].direction = thrusters[i]["params"]["direction"] == "FORWARD" ? DirectionType::Forward : DirectionType::Backward;
        thrusters_[i].propeller_type = thrusters[i]["params"]["propeller"] == "SYM" ? PropellerType::Symmetrical : PropellerType::Asymmetrical;
        thrusters_[i].shoulder = thrusters[i]["params"]["shoulder"];
        thrusters_[i].negative_factor = thrusters[i]["params"]["neg_koef"];
        thrusters_[i].reverse = thrusters[i]["params"]["reverse"];
        thrusters_[i].forward = thrusters[i]["params"]["forward"];
        thrusters_[i].right = thrusters[i]["params"]["right"];
        thrusters_[i].down = thrusters[i]["params"]["down"];
    }
}

void Tcu::normalize_config_values()
{
    double max_thrust = 0;

    double thrust_first = fabs(thrusts_[0]);
    double thrust_last = fabs(thrusts_[thrusts_.size() - 1]);

    if(thrust_first >= thrust_last) {
        max_thrust = thrust_first;
    } else {        
        max_thrust = thrust_last;
    }

    ROS_ASSERT_MSG(max_thrust != 0, "FAIL: Max thrust is 0, check thrusts config");

    for (size_t i = 0; i < thrusts_.size(); ++i) {   
        thrusts_[i] /= max_thrust;
        thrusts_to_codes_[thrusts_[i]] = codes_[i];
    }
}

void Tcu::normalize_channel(const LocationType type)
{
    double maximum = 0.0;
    for (const auto & t : thrusters_) {
        if (t.location == type && fabs(t.thrust) > maximum) {
            maximum = fabs(t.thrust);
        }
    }
    if (maximum > 1.0) {
        for (auto & t : thrusters_) {
            if (t.location == type) {
                t.thrust /= maximum;
            }
        }
    }
}

void Tcu::calc_thrusters_distribution()
{
    for (int i = 0; i < N; ++i) {
        b[0] += fabs(thrusters_[i].forward);
        b[1] += fabs(thrusters_[i].right);
        b[2] += fabs(thrusters_[i].down);

        if (thrusters_[i].location == LocationType::Horizontal) {
            b[3] += fabs(thrusters_[i].shoulder);
        } else {
            b[4] += fabs(thrusters_[i].shoulder);
        }

        A[0][i] = thrusters_[i].forward;
        A[1][i] = thrusters_[i].right;
        A[2][i] = thrusters_[i].down;
        if (thrusters_[i].location == LocationType::Horizontal) {
            A[3][i] = thrusters_[i].shoulder;
            A[4][i] = 0;
        } else {
            A[3][i] = 0;
            A[4][i] = thrusters_[i].shoulder;
        }
    }

    ROS_ASSERT_MSG(invert_matrix(A, A_inverse) == 0, "FAIL: Singular thrusters matrix");

    for (auto & t : thrusters_) {
        t.thrust = t.previous_thrust = 0;
        t.signal = 0;
    }
}

void Tcu::calc_new_thrusts(const tcu::CmdForce& msg)
{
    array<double, DOF> regul_vals {msg.forward, msg.right, msg.down, msg.mdown, msg.mright};

    for (int i = 0; i < N; ++i) {
        thrusters_[i].thrust = 0;
        for (int j = 0; j < DOF; ++j) {
            thrusters_[i].thrust += regul_vals[j] * b[j] * A_inverse[i][j];
        }
    }

    normalize_channel(LocationType::Vertical);
    normalize_channel(LocationType::Horizontal);

    for (auto & t : thrusters_) {
        if (t.thrust > max_force_) {
            t.thrust = max_force_;
            ROS_WARN("Recieved CmdForce with value more than maximum force");
        }
        if (t.thrust < -max_force_) {
            t.thrust = -max_force_;
            ROS_WARN("Recieved CmdForce with value more than maximum force");
        }
        if (fabs(t.thrust - t.previous_thrust) > delta_force_) {
            if (t.thrust > t.previous_thrust) {
                t.thrust = t.previous_thrust + delta_force_;
            } else if (t.thrust < t.previous_thrust) {
                t.thrust = t.previous_thrust - delta_force_;
            }
        }
        t.previous_thrust = t.thrust;
    }
}

void Tcu::calc_new_signals()
{
    for (auto & t : thrusters_) {
        double thrust = t.thrust;
        double signal = 0;

        if (t.direction == DirectionType::Backward) {
            thrust *= -1;
        }

        if (t.propeller_type == PropellerType::Asymmetrical) {
            if ((t.direction == DirectionType::Backward) ^ (thrust < 0)) {
                thrust *= t.negative_factor;
            }
        }

        if (thrust < thrusts_to_codes_.begin()->first) {
            thrust = thrusts_to_codes_.begin()->first;
        }

        auto point_up = thrusts_to_codes_.upper_bound(thrust);
        if (point_up == thrusts_to_codes_.end()) {
            point_up = prev(thrusts_to_codes_.end());
            thrust = point_up->first;
        }
        auto point_down = prev(point_up);

        double x1 = point_down->first;
        double y1 = point_down->second;
        double x2 = point_up->first;
        double y2 = point_up->second;

        if (x1 != x2) {
            signal = y1 + (y2 - y1) * (thrust - x1) / (x2 - x1);
        } else {
            signal = (y1 > y2) ? y2 : y1;
        }

        t.signal = signal;
    }
}

void Tcu::send_settings_individual(const int num)
{
    supervisor::CmdCan msg;

    std::vector<char> data = {(char)(common_can_addr_ % 256), (char)(common_can_addr_ / 256), (char)num, (char)thrusters_[num].reverse, 0x00, 0x00, 0x00, 0x00};
    copy(data.begin(), data.end(), msg.can_data.begin());

    msg.can_id = thrusters_[num].can_id + 3;

    can_send_pub_.publish(msg);
}

void Tcu::send_all_settings()
{
    for (int i = 0; i < N; ++i) {
        send_settings_individual(i);
    }

}

void Tcu::send_thrusts()
{
    // групповая рассылка
    supervisor::CmdCan msg;

    msg.can_id = common_can_addr_;

    for (int i = 0; i < N; ++i) {
        msg.can_data[i] = thrusters_[i].signal;
    }

    can_send_pub_.publish(msg);
}

void Tcu::stop_thrusters()
{
    for (auto & thruster : thrusters_) {
        thruster.thrust = 0;
        thruster.signal = 0;
    }
}

int main(int argc, char **argv)
{
	auto communicator = ipc::init(argc, argv, Tcu::NODE_NAME);
    Tcu tcu(communicator);

    tcu.send_all_settings();

    ipc::EventLoop loop(20);
    while(loop.ok())
    {
        increase_loop_params();

        if (need_stop_thrusters()) {
            tcu.stop_thrusters();
            tcu.send_thrusts();
            reset_regul_msg_it();
        }
        if (need_send_settings()) {
            tcu.send_settings_individual(settings_id);
            reset_settings_it();
        }
    }

    return 0;

}

///@}