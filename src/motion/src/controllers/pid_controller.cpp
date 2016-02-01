#include "controllers/pid_controller.h"

#include <utils/basic.h>
#include <log.h>

using namespace std;

PIDController::PIDController(double kp, double ki, double kd):
    kp(kp), ki(ki), kd(kd),
    last_err(0),
    last_time(fixate_time()),
    integral(0)
{

}

PIDController::~PIDController()
{

}

double PIDController::update(double err, double err_d)
{
    double cur_time = fixate_time();
    integral += err * (cur_time - last_time);
    last_err = err;
    last_time = cur_time;
    return kp * err + ki * integral + kd * err_d;
}

double PIDController::update(double err)
{
    double err_d = (err - last_err) / (fixate_time() - last_time);
    return update(err, err_d);
}
