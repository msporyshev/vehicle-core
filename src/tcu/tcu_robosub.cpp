#include "tcu_robosub.h"
#include <supervisor/CmdCan.h>
#include "matrix_inversion.h"

using namespace std;

const int TcuRobosub::settings_iterations = 40;

TcuRobosub::TcuRobosub(ipc::Communicator& com) :
    Tcu(com)
{    
    for (auto i = 0; i < N; ++i) {
        thrusters_.push_back(make_shared<FaulhaberThruster>());
    }

    init_ipc();    
	read_config(N);
    normalize_config_values();
    calc_thrusters_distribution();
    send_settings();

}

TcuRobosub::~TcuRobosub()
{

}

void TcuRobosub::init_ipc()
{
    can_send_pub_ = communicator_.advertise_cmd<supervisor::CmdCan>("supervisor");
}

void TcuRobosub::read_config(const int num_of_thrusters)
{
    XmlRpc::XmlRpcValue thrusters;
    
    Tcu::read_config(num_of_thrusters);

    ROS_ASSERT(ros::param::get("/tcu/common_can_addr", common_can_addr_));
    ROS_ASSERT(ros::param::get("/tcu/thrusts", thrusts_));
    ROS_ASSERT(ros::param::get("/tcu/codes", codes_));

    ROS_ASSERT_MSG(codes_.size() == thrusts_.size(), "FAIL: Thrusts and codes have different size. Unable to interprete config");
    ROS_ASSERT_MSG(thrusts_.size() >= 2, "FAIL: Thrusts and codes should contain at least 2 numbers each");

    ROS_ASSERT(ros::param::get("/tcu/thrusters", thrusters));

    for (auto i = 0; i < num_of_thrusters; ++i) {

        static_pointer_cast<FaulhaberThruster>(thrusters_[i])->can_id = thrusters[i]["params"]["id"];
    }
}

void TcuRobosub::increase_loop_params()
{
    Tcu::increase_loop_params();
    last_settings_it_++;
}

void TcuRobosub::send_settings_individual(const int num)
{
    supervisor::CmdCan msg;

    std::vector<char> data = {(char)(common_can_addr_ % 256), (char)(common_can_addr_ / 256), (char)num, (char)thrusters_[num]->reverse, 0x00, 0x00, 0x00, 0x00};
    copy(data.begin(), data.end(), msg.can_data.begin());

    msg.can_id = static_pointer_cast<FaulhaberThruster>(thrusters_[num])->can_id + 3;

    can_send_pub_.publish(msg);
}

void TcuRobosub::send_settings()
{
    for (int i = 0; i < N; ++i) {
        send_settings_individual(i);
    }
}

void TcuRobosub::send_thrusts()
{
    // групповая рассылка
    supervisor::CmdCan msg;

    msg.can_id = common_can_addr_;

    for (int i = 0; i < N; ++i) {
        msg.can_data[i] = thrusters_[i]->signal;
    }

    can_send_pub_.publish(msg);
}

void TcuRobosub::reset_settings_it()
{
    last_settings_it_ = 0;
}

bool TcuRobosub::need_send_settings()
{
    if (last_settings_it_ > settings_iterations) {
        settings_id_ = (settings_id_ + 1) % N;
        return true;
    }

    return false;
}

void TcuRobosub::normalize_config_values()
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

void TcuRobosub::normalize_channel(const LocationType type)
{
    double maximum = 0.0;
    for (const auto & t : thrusters_) {
        if (t->location == type && fabs(t->thrust) > maximum) {
            maximum = fabs(t->thrust);
        }
    }
    if (maximum > 1.0) {
        for (auto & t : thrusters_) {
            if (t->location == type) {
                t->thrust /= maximum;
            }
        }
    }
}

void TcuRobosub::calc_new_thrusts(const tcu::CmdForce& msg)
{
    array<double, DOF> regul_vals {msg.forward, msg.right, msg.down, msg.mdown, msg.mright};

    for (int i = 0; i < N; ++i) {
        thrusters_[i]->thrust = 0;
        for (int j = 0; j < DOF; ++j) {
            thrusters_[i]->thrust += regul_vals[j] * b[j] * A_inverse[i][j];
        }
    }

    normalize_channel(LocationType::Vertical);
    normalize_channel(LocationType::Horizontal);

    for (auto & t : thrusters_) {
        if (t->thrust > max_force_) {
            t->thrust = max_force_;
            ROS_WARN("Recieved CmdForce with value more than maximum force");
        }
        if (t->thrust < -max_force_) {
            t->thrust = -max_force_;
            ROS_WARN("Recieved CmdForce with value more than maximum force");
        }
        if (fabs(t->thrust - t->previous_thrust) > delta_force_) {
            if (t->thrust > t->previous_thrust) {
                t->thrust = t->previous_thrust + delta_force_;
            } else if (t->thrust < t->previous_thrust) {
                t->thrust = t->previous_thrust - delta_force_;
            }
        }
        t->previous_thrust = t->thrust;
    }
}

void TcuRobosub::calc_new_signals()
{
    for (auto & t : thrusters_) {
        double thrust = t->thrust;
        double signal = 0;

        if (t->direction == DirectionType::Backward) {
            thrust *= -1;
        }

        if (t->propeller_type == PropellerType::Asymmetrical) {
            if ((t->direction == DirectionType::Backward) ^ (thrust < 0)) {
                thrust *= t->negative_factor;
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

        t->signal = signal;
    }
}

void TcuRobosub::calc_thrusters_distribution()
{
    for (int i = 0; i < N; ++i) {
        b[0] += fabs(thrusters_[i]->forward);
        b[1] += fabs(thrusters_[i]->right);
        b[2] += fabs(thrusters_[i]->down);

        if (thrusters_[i]->location == LocationType::Horizontal) {
            b[3] += fabs(thrusters_[i]->shoulder);
        } else {
            b[4] += fabs(thrusters_[i]->shoulder);
        }

        A[0][i] = thrusters_[i]->forward;
        A[1][i] = thrusters_[i]->right;
        A[2][i] = thrusters_[i]->down;
        if (thrusters_[i]->location == LocationType::Horizontal) {
            A[3][i] = thrusters_[i]->shoulder;
            A[4][i] = 0;
        } else {
            A[3][i] = 0;
            A[4][i] = thrusters_[i]->shoulder;
        }
    }

    ROS_ASSERT_MSG(invert_matrix(A, A_inverse) == 0, "FAIL: Singular thrusters matrix");

    for (auto & t : thrusters_) {
        t->thrust = t->previous_thrust = 0;
        t->signal = 0;
    }
}

void TcuRobosub::routine()
{
    if (need_send_settings()) {
        send_settings_individual(settings_id_);
        reset_settings_it();
    }
}

