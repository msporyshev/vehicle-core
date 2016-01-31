/**
\file
\brief Заголовочный файл модуля управления движителями

В данном файле находятся обработчики сообщений, принимаемых tcu,
а также методы, выполняющие обработку и публикацию сообщений модуля tcu

\ingroup tcu_node
*/

///@{

#pragma once

#include <string>
#include <vector>
#include <map>
#include <ros/ros.h>

#include <motion/MsgRegul.h>
#include <libipc/ipc.h>

#define N 5 // количество движков
#define DOF 5 // количество степеней свободы

class Tcu
{
public:
	Tcu(ipc::Communicator& communicator);
	virtual ~Tcu();

	enum class LocationType {Vertical, Horizontal};
    enum class PropellerType {Symmetrical, Asymmetrical};
    enum class DirectionType {Forward, Backward};

    struct Thruster
	{
	    int can_id;
	    LocationType location;
	    DirectionType direction;
	    PropellerType propeller_type;
	    double shoulder;
	    double negative_factor;
	    int reverse;
	    double tx, ty, tz;

	    int signal;
	    double thrust;
	    double previous_thrust;
	};    

	///< Имя модуля
	static const std::string NODE_NAME;

	/**
    Метод выполняет подписку на все сообщения, 
    принимаемые tcu и регистрирует все сообщения,
    публикуемые tcu
    */
    void calc_new_signals();
    void calc_new_thrusts(const motion::MsgRegul& msg);
    void calc_thrusters_distribution();
	void init_ipc();
	void normalize_config_values();
    void normalize_channel(const LocationType type);
    void process_regul_msg(const motion::MsgRegul& msg);
    void read_config();
    void send_settings_individual(const int num); // num = [0; N - 1]
    void send_all_settings();
    void send_thrusts();
    void stop_thrusters();
    void update_thrusts(const motion::MsgRegul& msg);


private:
	ipc::Communicator& communicator_;

    ros::Publisher can_send_pub_;

    std::array<Thruster, N> thrusters_;

    std::array<double, DOF> b;
    std::array<std::array<double, DOF>, DOF> A;
    std::array<std::array<double, DOF>, DOF> A_inverse;

    double max_force_;
	double delta_force_;
	int common_can_addr_;

	std::vector<double> thrusts_;
    std::vector<int> codes_;
    std::map <double, int> thrusts_to_codes_;

};

///@}