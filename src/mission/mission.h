#pragma once

/**
\file
\brief Заголовочный файл миссии

В данном файле находятся обработчики сообщений, принимаемых миссией,
а также методы, выполняющие обработку и публикацию сообщений миссии

*/

#include <libipc/ipc.h>
#include <motion/motion_client/robosub_motion_client.h>

#include <memory>
#include <vector>
#include <string>

class Mission
{
public:
    Mission(ipc::Communicator& communicator);
    ~Mission();

    /**
    Метод выполняет подписку на все сообщения,
    принимаемые навигом и регистрирует все сообщения,
    публикуемые навигом
    */
    void init_ipc();

    /**
    Создание и публикация команд регуляторам
    */
    void publish_commands();

    /**
    Шаблонный обработчик сообщений.
    Печатает на консоль тип полученного сообщения и его содержимое
    \param[in] msg Сообщение
    */
    template<typename T>
    void handle_message(const T& msg)
    {
        ROS_INFO_STREAM("Received " << ipc::classname(msg));
    }

    void send_commands()
    {
        motion_.fix_heading(0);
        motion_.fix_pitch();
        motion_.move_forward(10, 10);
        motion_.move_right(10, 10);
        motion_.move_left(10, 10);
        motion_.move_backward(10, 10);
        motion_.move_up(10);
        motion_.move_down(10);
    }

    ///< Имя модуля
    static const std::string NODE_NAME;
private:
    std::vector<ros::Publisher> publishers_;

    ipc::Communicator& communicator_;
    RobosubMotionClient motion_;
};