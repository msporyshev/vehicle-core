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

class Dsp
{
public:
	Dsp(ipc::Communicator& communicator);
	virtual ~Dsp();

	///< Имя модуля
	static const std::string NODE_NAME;

	/**
    Метод выполняет подписку на все сообщения, 
    принимаемые dsp и регистрирует все сообщения,
    публикуемые dsp
    */
	void init_ipc();

	 /**
    Создает и публикует сообщение о координатах маяка и его частоте
    */
	void create_and_publish_beacon();

	/**
    Выполняет обработку сообщений от навига о глубине
    \param[in] msg Сообщение от навига
    */
	void handle_depth(const navig::MsgNavigDepth& msg);

private:
	ipc::Communicator& communicator_;

    ros::Publisher beacon_pub_;
};

///@}