#include "tcu_surface.h"
#include "com/com.h"

using namespace std;

const int TcuSurface::buf_len_ = 65536;

TcuSurface::TcuSurface(ipc::Communicator& com) :
    Tcu(com)
{
	for (auto i = 0; i < N; ++i) {
        thrusters_.push_back(make_shared<TorqeedoThruster>());
    }

	read_config(N);
	string check = init_connections();
	ROS_ASSERT_MSG(check == "", "FAIL: Couldn't open %s", check.c_str());

	buffer_ = new unsigned char[buf_len_];

}

TcuSurface::~TcuSurface()
{
	delete[] buffer_;
}

void TcuSurface::read_config(const int num_of_thrusters)
{
	XmlRpc::XmlRpcValue thrusters;
    
    Tcu::read_config(num_of_thrusters);

    ROS_ASSERT(ros::param::get("/tcu/dead_point", dead_point_));
    ROS_ASSERT(ros::param::get("/tcu/max_signal", max_signal_));
    ROS_ASSERT(ros::param::get("/tcu/thrusters", thrusters));

    for (auto i = 0; i < num_of_thrusters; ++i) {

        (static_pointer_cast<TorqeedoThruster>(thrusters_[i]))->tty_port_name = string(thrusters[i]["params"]["tty_port_name"]);
        (static_pointer_cast<TorqeedoThruster>(thrusters_[i]))->tty_port_baudrate = thrusters[i]["params"]["tty_port_baudrate"];
    }
}

string TcuSurface::init_connections()
{
	for (auto & thruster : thrusters_) {

		auto t = static_pointer_cast<TorqeedoThruster>(thruster);
		t->tty_port_handler = COM_open(t->tty_port_name.c_str(), t->tty_port_baudrate);

		if(t->tty_port_handler == INVALID_FILE_DESCRIPTOR) {
			return t->tty_port_name;
		}

	}

	return "";
}

void TcuSurface::calc_new_signals()
{	
	for (auto & t : thrusters_) {
        double thrust = t->thrust;
        double signal = 0;        

        if(thrust > 0.0) {
        	signal = dead_point_ + t->thrust * (max_signal_ - dead_point_);
        } else if(thrust < 0.0) {
			signal = -dead_point_ + t->thrust * (max_signal_ - dead_point_);
		} else {
			signal = 0.0;
		} 		

        t->signal = signal;        
    }
}


void TcuSurface::calc_new_thrusts(const tcu::CmdForce& msg)
{
	double a;
	for(auto &t : thrusters_) {
		double previous_thrust = t->previous_thrust;
		double thrust = msg.forward + t->shoulder * msg.mdown;

		//сдвигаем оба сигнала: обеспечиваем mz в ущерб tx
		if (thrust > 1) {a = thrust - 1; thrust -= a;}
		if (thrust < -1) {a = -1 - thrust; thrust += a;}
	
		if (t->direction == DirectionType::Backward) {
            thrust *= -1;
        }

        if (t->propeller_type == PropellerType::Asymmetrical) {
            if ((t->direction == DirectionType::Backward) ^ (thrust < 0)) {
                thrust *= t->negative_factor;
            }
        }

		if (thrust > max_force_) {
            thrust = max_force_;
            ROS_WARN("Recieved CmdForce with value more than maximum force");
        }
        if (thrust < -max_force_) {
            thrust = -max_force_;
            ROS_WARN("Recieved CmdForce with value more than maximum force");
        }
        if (fabs(thrust - previous_thrust) > delta_force_) {
            if (thrust > previous_thrust) {
                thrust = previous_thrust + delta_force_;
            } else if (thrust < previous_thrust) {
                thrust = previous_thrust - delta_force_;
            }
        }
        t->thrust = thrust;
        t->previous_thrust = thrust;        
	}

}

void TcuSurface::send_thrusts()
{
	for (auto & thruster : thrusters_) {
		auto t = static_pointer_cast<TorqeedoThruster>(thruster);

        vector<unsigned char> v;
        v.push_back(0x30);
        v.push_back(0x82);
        v.push_back(t->signal?0x09:0x08);
        v.push_back(t->signal?0x64:0x00);
        v.push_back((t->signal & 0xff00) >> 8);
        v.push_back(t->signal & 0xff);
        v.push_back(crc8(v));
        decorate(v);
        write(t->tty_port_handler, v.data(), v.size());        
    }
}

void TcuSurface::decorate(vector<unsigned char> &v)
{
	auto i = 0;
    while (i < v.size()) {           
        if (v[i] == 0xAE || v[i] == 0xAC || v[i] == 0xAD) {
            //порядок важен, т.к. в этой строке используется v[i]
            v.insert(v.begin() + i+1, 0x20 + (v[i] & 0x0f));
            v[i] = 0xAE;
            ++i;
        }
        ++i;
    }
    v.insert(v.begin(), 0xAC);
    v.insert(v.begin(), 0xAA);
    v.insert(v.begin(), 0xAA);
    v.push_back(0xAD);
}

unsigned char TcuSurface::crc8(const vector<unsigned char> &v)
{
	unsigned char res = 0x00;
    for (auto i = 0; i < v.size(); ++i) {
        res ^= v[i];
        for (auto j = 0; j < 8; ++j) {
            res = res & 0x01 ? (res >> 1) ^ 0x8C : res >> 1;
        }
    }
    return res;
}

void TcuSurface::process_message(shared_ptr<TorqeedoThruster> &t)
{
	if (t->message.size() == 3 && !(t->message[0] || t->message[1] || t->message[2])) {
        vector<unsigned char> v;
        v.push_back(0xAC);
        v.push_back(0x30);
        v.push_back(0x03);
        v.push_back(0xCF);
        v.push_back(0xAD);
        write(t->tty_port_handler, v.data(), v.size());
    } else if (t->message.size() == 15 && !(t->message[0] || t->message[1])) {
    	ROS_WARN("Thruster sent error state!");
    } else {
    	ROS_WARN("Thruster sent unknown message! Size: %d", static_cast<int>(t->message.size()));
    }
}

void TcuSurface::process_new_buffer(const unsigned char *buf, int len, shared_ptr<TorqeedoThruster> &t)
{
	for (auto i = 0; i < len; ++i) {
        if (buf[i] == 0xAC && !t->inside_of_message) {
            t->inside_of_message = true;
            t->message.clear();
        } else if (buf[i] == 0xAD && t->inside_of_message) {
            process_message(t);
            t->inside_of_message = false;
        } else if (t->inside_of_message) {
            t->message.push_back(buf[i]);
        }
    }
}

void TcuSurface::routine()
{
	for (auto & thruster : thrusters_) {
		auto t = static_pointer_cast<TorqeedoThruster>(thruster);

		auto read_len = read(t->tty_port_handler, buffer_, buf_len_);
		if(read_len) {
			process_new_buffer(buffer_, read_len, t);	
		}
	}
}
