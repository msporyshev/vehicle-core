/**
\file
\brief Заголовочный файл модуля определения координат акустических маяков

В данном файле находятся обработчики сообщений, принимаемых DSP,
а также методы, выполняющие обработку и публикацию сообщений модуля DSP

\ingroup dsp_node
*/

///@{

#pragma once

#include <string>
#include <ros/ros.h>

#include <navig/MsgNavigDepth.h>
#include <libipc/ipc.h>

#include "connector.h"

class Dsp
{
public:
	Dsp(ipc::Communicator& communicator);
	virtual ~Dsp();

	enum class CommandType {DspOff, DspOn, Freq37500, Freq20000, DebugOn, DebugOff, Count};	


	///< Имя модуля
	static const std::string NODE_NAME;    

	Connector* con;

	void read_config();
    void init_ipc();
    void publish_beacon();
    void handle_dsp_cmd(const dsp::CmdDspSendCommand& msg);
	void set_mode(CommandType mode);
	int package_processing();

private:
    ipc::Communicator& communicator_;
    ros::Publisher beacon_pub_;
    
	bool debug_mode_;

	unsigned char* buffer_;

	ConnectorType connector_type_;
	std::string connector_type_str_;
    std::string com_name_;
    int baudrate_;

    int dsp_preamble_;
    int buffer_size_;

	double base_short_;
    double base_long_;
    int max_delay_base_short_;
    int max_delay_base_long_;
    double sound_speed_;
    double dz_max_;
    int preamble_size_;
    double dsp_rate_;

    double bearing_;
    double distance_;
    char beacon_type_;

    int channel_0_;
    int channel_1_;
    int channel_2_;

};

///@}