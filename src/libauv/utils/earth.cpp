#include "earth.h"
#include <math.h>

namespace utils {
//------------------------------------------------------------------------------
// Изменение широты при движении вдоль меридиана
//------------------------------------------------------------------------------
double meter_rad_lat ( double delta_N, double Lat )
{
    double one_rad;
    double delta_rad;

    one_rad = ( a * ( 1 - e2 ) ) / pow ( ( 1 - e2 * pow ( sin ( Lat ), 2 ) ), 1.5 );
    delta_rad = delta_N / one_rad;

    return ( delta_rad );
}

//------------------------------------------------------------------------------
// Изменение долготы при движении вдоль параллели
//------------------------------------------------------------------------------
double meter_rad_long ( double delta_E, double Lat )
{
    double one_rad;
    double delta_rad;

    one_rad = ( a * cos ( Lat ) ) / pow ( ( 1 - e2 * pow ( sin ( Lat ), 2 ) ), 0.5 );
    delta_rad = delta_E / one_rad;

    return ( delta_rad );
}

//------------------------------------------------------------------------------
// Длина дуги параллели в 1 радиан на широте Lat
//------------------------------------------------------------------------------
double long1rad ( double Lat )
{
    double one_rad;
    one_rad = ( a * cos ( Lat ) ) / pow ( ( 1 - e2 * pow ( sin ( Lat ), 2 ) ), 0.5 );

    return ( one_rad );
}

//------------------------------------------------------------------------------
// Радиус кривизны меридианного эллипса в точке, лежащей на широте Lat
//------------------------------------------------------------------------------
double lat1rad ( double Lat )
{
    double one_rad;
    one_rad = ( a * ( 1 - e2 ) ) / pow ( ( 1 - e2 * pow ( sin ( Lat ), 2 ) ), 1.5 );

    return ( one_rad );
}

} // namespace utils