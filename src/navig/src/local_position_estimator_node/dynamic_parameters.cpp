#include "dynamic_parameters.h"

DynamicParameters::DynamicParameters() :
    delta_t(0.), 
    vx(0.), 
    vy(0.), 
    ax(0.), 
    ay(0.)
{}

DynamicParameters::DynamicParameters(double delta_t, double t, double ax, double ay,
    double vx_prev, double vy_prev) :
delta_t(delta_t),
t(t),
ax(ax),
ay(ay)
{
    vx = ax * delta_t - vx_prev;
    vy = ay * delta_t - vy_prev;
}

double DynamicParameters::calc_vx(double vx_prev)
{
    return this->vx = ax * delta_t - vx_prev;
}
    
double DynamicParameters::calc_vy(double vy_prev)
{
    return this->vy = ay * delta_t - vy_prev;
}