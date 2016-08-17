#pragma once

#include <math_u.h>

namespace utils {
//------------------------------------------------------------------------------
// Параметры модели геоида
//------------------------------------------------------------------------------
// const double a = 6378245.0;
const double a = 6378137.0;
// const double e2 = 0.006693421623;
const double e2 = 0.006694380004;

//--------------- programmes for transformation meter -> radian ----------------
// first parameter - displacement (in meters)
// second parameter - current lattitude (in radian)
// return parameter - displacement (in radian)

//------------------------------------------------------------------------------
// Изменение широты при движении вдоль меридиана
//------------------------------------------------------------------------------
double meter_rad_lat ( double delta_N, double Lat );

inline double meter_deg_lat(double delta, double lat)
{
    return meter_rad_lat(to_rad(delta), to_rad(lat));
}


//------------------------------------------------------------------------------
// Изменение долготы при движении вдоль параллели
//------------------------------------------------------------------------------
double meter_rad_long ( double delta_E, double Lat );

inline double meter_deg_long(double delta, double lat)
{
    return meter_rad_long(to_rad(delta), to_rad(lat));
}



//------------------------------------------------------------------------------
// Длина дуги параллели в 1 радиан на широте Lat
//------------------------------------------------------------------------------
double long1rad ( double Lat );
inline double long1deg(double lat)
{
    return to_deg(long1rad(to_rad(lat)));
}

//------------------------------------------------------------------------------
// Радиус кривизны меридианного эллипса в точке, лежащей на широте Lat
// (везде это используется как длина дуги меридиана в 1 радиан на широте Lat)
//------------------------------------------------------------------------------
double lat1rad ( double Lat );
inline double lat1deg(double lat)
{
    return to_deg(lat1rad(to_rad(lat)));
}

} // namespace utils