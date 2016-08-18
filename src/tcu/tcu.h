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
#include <memory>
#include <ros/ros.h>

#include <tcu/CmdForce.h>
#include <libipc/ipc.h>

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
	    LocationType location;
	    DirectionType direction;
	    PropellerType propeller_type;
	    double shoulder;
	    double negative_factor;
	    int reverse;
	    double forward, right, down;

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
    virtual void calc_new_signals() = 0;
    virtual void calc_new_thrusts(const tcu::CmdForce& msg) = 0;
    virtual void increase_loop_params();
    virtual void init_ipc();        
    bool need_stop_thrusters();
    void process_regul_msg(const tcu::CmdForce& msg);
    void reset_regul_msg_it();
    virtual void routine(){};
    virtual void send_thrusts() = 0;
    void stop_thrusters();
    void update_thrusts(const tcu::CmdForce& msg);

private:
    static const int silence_iterations;    

    int last_regul_msg_it_ = 0;

protected:
    virtual void read_config(const int num_of_thrusters);

    ipc::Communicator& communicator_;    
    std::vector<std::shared_ptr<Thruster>> thrusters_;
    
    double max_force_;
    double delta_force_;

};

///@}