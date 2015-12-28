/**
\file
\brief Заголовочный файл навигационного модуля

В данном файле находятся обработчики сообщений, принимаемых навигом,
а также методы, выполняющие обработку и публикацию сообщений навигационного модуля

\ingroup navig_node
*/

///@{
#pragma once

#include <string>

#include <ros/ros.h>

#include <compass/MsgCompassAngle.h>
#include <compass/MsgCompassAcceleration.h>
#include <compass/MsgCompassAngleRate.h>
#include <dvl/MsgDvlDistance.h>
#include <dvl/MsgDvlVelocity.h>
#include <dvl/MsgDvlHeight.h>
#include <gps/MsgGpsCoordinate.h>
#include <gps/MsgGpsSatellites.h>
#include <gps/MsgGpsUtc.h>
#include <supervisor/MsgSupervisorDepth.h>

#include <libipc/ipc.h>

class Navig
{
public:
    Navig(ipc::Communicator& communicator);
    virtual ~Navig();

    ///< Имя модуля
    static const std::string NODE_NAME;

    /**
    Метод выполняет подписку на все сообщения, 
    принимаемые навигом и регистрирует все сообщения,
    публикуемые навигом
    */
    void init_ipc();

    /**
    Выполняет обработку и дальнейшую публикацию сообщений об ускорении,
    получаемых от компаса
    \param[in] msg Сообщение от компаса об ускорениях
    */
    void process_and_publish_acc(const compass::MsgCompassAcceleration& msg);
    
    /**
    Выполняет обработку и дальнейшую публикацию сообщений об углах, 
    получаемых от компаса
    \param[in] msg Сообщение от компаса об углах (курс, крен, дифферент)
    */
    void process_and_publish_angles(const compass::MsgCompassAngle& msg);

    /**
    Выполняет обработку и дальнейшую публикацию сообщений об угловых ускорениях,
    получаемых от компаса
    \param[in] msg Сообщение от компаса об угловых ускорениях
    */
    void process_and_publish_rates(const compass::MsgCompassAngleRate& msg);
    
    /**
    Создает и публикует сообщения о глубине
    */
    void create_and_publish_depth();
    
    /**
    Создает и публикует сообщения о высоте над дном
    */
    void create_and_publish_height();

    /**
    Создает и публикует сообщения о местоположении в локальной и глобальной
    системах координат
    */
    void create_and_publish_position();

    /**
    Создает и публикует сообщения о скоростях аппарата
    */
    void create_and_publish_velocity();


    /**
    Шаблонный обработчик сообщений.
    Печатает на консоль тип полученного сообщения и его содержимое
    \param[in] msg Сообщение
    */
    template<typename T>
    void handle_message(const T& msg)
    {
        ROS_DEBUG_STREAM("Received " << ipc::classname(msg));
        // ROS_DEBUG_STREAM(msg);
    }
    
    /**
    Обрабатывает сообщения об углах от компаса
    \param[in] msg Углы (курс, крен, дифферент)
    */
    void handle_angles(const compass::MsgCompassAngle& msg);
    
    /**
    Обрабатывает сообщения об ускорениях от компаса
    \param[in] msg Ускорения
    */
    void handle_acceleration(const compass::MsgCompassAcceleration& msg);
    
    /**
    Обрабатывает сообщения об угловых ускорениях от компаса
    \param[in] msg Угловые ускорения
    */
    void handle_rate(const compass::MsgCompassAngleRate& msg);
    // void handle_distance_backward(const dvl::MsgDvlDistanceBackward& msg);
    // void handle_distance_forward(const dvl::MsgDvlDistanceForward& msg);
    // void handle_distance_leftward(const dvl::MsgDvlDistanceLeftward& msg);
    // void handle_distance_rightward(const dvl::MsgDvlDistanceRightward& msg);
    // void handle_velocity_down(const dvl::MsgDvlVelocityDown& msg);
    // void handle_velocity_forward(const dvl::MsgDvlVelocityForward& msg);
    // void handle_velocity_right(const dvl::MsgDvlVelocityRight& msg);
    // void handle_height(const dvl::MsgDvlHeight& msg);
    // void handle_coordinate(const gps::MsgGpsCoordinate& msg);
    // void handle_satellites(const gps::MsgGpsSatellites& msg);
    // void handle_utc(const gps::MsgGpsUtc& msg);
    // void handle_depth(const sucan::MsgSucanDepth& msg);

private:
    std::pair<double, double> local_position_ = std::make_pair(0.0, 0.0);
    double longitude_ = 0.0; ///< Долгота
    double latitude_ = 0.0; ///< Широта
    float acc_x_ = 0.0; ///< Ускорение по оси Х
    float acc_y_ = 0.0; ///< Ускорение по оси У
    float acc_z_ = 0.0; ///< Ускорение по оси Z
    float heading_ = 0.0; ///< Курс
    float pitch_ = 0.0; ///< Дифферент
    float roll_ = 0.0; ///< Крен
    float depth_ = 0.0; ///< Глубина
    float height_ = 0.0; ///< Высота над дном
    float rate_heading_ = 0.0; ///< Ускорение по курсу
    float rate_roll_ = 0.0; ///< Ускорение по крену
    float rate_pitch_ = 0.0; ///< Ускорение по дифференту
    float velocity_forward_ = 0.0; ///< Скорость вперед
    float velocity_right_ = 0.0; ///< Скорость вправо
    float velocity_down_ = 0.0; ///< Скорость вниз

    ipc::Communicator& communicator_;

    ros::Publisher acc_pub_, angles_pub_, depth_pub_, height_pub_, position_pub_, rates_pub_, 
        velocity_pub_;
};

///@}