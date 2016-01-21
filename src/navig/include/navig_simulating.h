#pragma once

#include <navig_base.h>

#include <libipc/ipc.h>

class NavigSimulating : public NavigBase
{
public:
    std::string get_name() const override;

    /**
    Метод выполняет подписку на все сообщения, 
    принимаемые навигом и регистрирует все сообщения,
    публикуемые навигом
    */
    void init_ipc(ipc::Communicator& communicator) override;

    void run() override;

    /**
    Функция для моделирования
    Создает и публикует сообщения об ускорениях.
    */
    void create_and_publish_acc();
    
    /**
    Функция для моделирования
    Создает и публикует сообщения об углах (курс, крен, дифферент)
    */
    void create_and_publish_angles();

    /**
    Функция для моделирования
    Создает и публикует сообщения об угловых ускорениях
    */
    void create_and_publish_rates();
    
    /**
    Функция для моделирования
    Создает и публикует сообщения о глубине
    */
    void create_and_publish_depth();
    
    /**
    Функция для моделирования
    Создает и публикует сообщения о высоте над дном
    */
    void create_and_publish_height();

    /**
    Функция для моделирования
    Создает и публикует сообщения о местоположении в локальной и глобальной
    системах координат
    */
    void create_and_publish_position();

    /**
    Функция для моделирования
    Создает и публикует сообщения о скоростях аппарата
    */
    void create_and_publish_velocity();

private:
    std::string NODE_NAME = "navig";
};