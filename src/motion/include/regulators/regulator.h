#pragma once

#include "thrust_info.h"
#include "navig_info.h"

#include <libauv/config_reader/yaml_reader.h>

#include <cmd_status.h>
#include <axis.h>

// #include <msg/devices/MsgNavig.h>
// #include <config_reader/yaml_reader.h>

#include <vector>

class Regulator
{
public:
    // Конструктор регулятора
    // * id -- идентификатор команды
    // * axis -- занимаемые регулятором каналы управления, изменить этот набор за время жизни регулятора нельзя
    // * timeout -- максимальное время работы регулятора, в секундах
    Regulator(int id, std::vector<Axis> axes, double timeout);
    virtual ~Regulator() {}

    // получение идентификатора команды
    int get_id() const;

    // получение занятых в конструкторе осей
    const std::vector<Axis>& get_axes() const;

    // получение обновленных тяг по занятым осям
    const std::vector<ThrustInfo>& get_thrusts() const;

    // активация регулятора
    void activate();
    // обработка навигационного сообщения
    void handle_navig(const NavigInfo& msg);
    // завершение работы регулятора
    void finish();

    // проверка, ожидает ли регулятор активации
    bool is_pending() const;
    // проверка, активен ли регулятор
    bool is_active() const;
    // проверка, завершен ли регулятор
    bool is_finished() const;

    // проверка, достиг ли регулятор на этой итерации заданного значения
    bool has_succeeded() const;

    // проверка, не истек ли таймаут регулятора
    bool is_actual() const;

    // проверка на наличие конфликтов осей управления с другими регуляторами
    bool has_conflicts(const Regulator& other) const;
    bool has_conflicts(std::shared_ptr<Regulator> other) const;

    // получение заголовков и значений величин, которые регулятор на каждой итерации выводит
    std::vector<std::string> get_log_headers() const;
    std::vector<double> get_log_values() const;
protected:

    // методы, необходимые для разработки своего регулятора

    // инициализация, вызывается при первой обработке навигационного сообщения активным регулятором
    virtual void initialize(const NavigInfo& msg);
    // обновление состояния регулятора, вызывается при каждой обработке навигационного сообщения активным регулятором
    virtual void update(const NavigInfo& msg);

    // установка тяг по занятым осям управления
    // если задается неполный набор или избыточный, сохраняются только данные по зарегистрированным осям управления
    void set_thrusts(std::vector<ThrustInfo> new_thrusts);

    // установка флага об успешности регулирования на основании логического условия
    void set_success(bool status);
    // установка флага об успешности регулирования на основании, входит ли величина невязки в заданные рамки
    void set_success(double err, double accuracy);
    // установка флага об успешности регулирования на основании, входит ли величина невязки
    // и ее производная в заданные рамки
    void set_success(double err, double accuracy, double err_d, double accuracy_d);

    // установка заголовков и значений для величин, которые выводит регулятор в ходе своей работы
    void set_log_headers(std::vector<std::string> headers);
    void set_log_values(std::vector<double> values);

    // добавление регулятора в качестве зависимости (вложенного регулятора)
    // при этом тяги, которые зависимость возвращает, добавляются к тягам текущего регулятора
    // флаг об успешности вычисляется конъюнкцией флагов успешности всех зависимостей и текущего регулятора
    // отладочная информация также автоматически добавляется к данным текущего регулятора
    void add_dependency(std::shared_ptr<Regulator> dep);
private:

    enum class State
    {
        PENDING, ACTIVE, FINISHED
    };

    int id;
    std::vector<Axis> axes;
    double start_time;
    double timeout;
    State state;
    bool success;
    bool first_update;
    std::vector<ThrustInfo> thrusts;

    std::vector<std::string> log_headers;
    std::vector<double> log_values;
    std::vector<std::shared_ptr<Regulator>> dependencies;
};
