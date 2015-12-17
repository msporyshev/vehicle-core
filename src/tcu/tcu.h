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
#include <ros/ros.h>

#include <motion/MsgRegul.h>
#include <libipc/ipc.h>

class Tcu
{
public:
	Tcu(ipc::Communicator& communicator);
	virtual ~Tcu();

	///< Имя модуля
	static const std::string NODE_NAME;

	/**
    Метод выполняет подписку на все сообщения, 
    принимаемые tcu и регистрирует все сообщения,
    публикуемые tcu
    */
	void init_ipc();

	 /**
    Создает и публикует сообщение о посылке CAN пакета
    */
	void create_and_publish_can_send();

	/**
    Выполняет обработку сообщений от регуляторов и дальнейшую публикацию
    для супервизора
    \param[in] msg Сообщение от регуляторов
    */
	void process_and_publish_regul(const motion::MsgRegul& msg);

private:
	ipc::Communicator& communicator_;

    ros::Publisher can_send_pub_;
};

///@}