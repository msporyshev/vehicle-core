/* Набросок служебных функций регулятора */

/// Дистанция между двумя точками
template <class Point1, class Point2>
double distance(Point1 point1, Point2 point2)
{
    double d_east  = point1.east  - point2.east;
    double d_north = point1.north - point2.north;
    return sqrt(d_east * d_east + d_north * d_north);
}

/// Направление в градусах на целевую точку из текущей позиции
template <class Target, class Current>
double heading_to_point(Target target, Current current)
{
    return ; //! Тут нужно аккуратненько высчитать направление на точку в градусах!
}


/// Пример того, что должно быть в fill_settings()
/// Если есть лаконичное считывание из объекта yaml'а, то конеш лучше его, а не "_"
template <class Settings>
fill_settings(Settings &settings)
{
    settings.pitch.prop = config::read_as<double>("pitch_p");    // Пропорциональная составляющая по дифференту
    settings.pitch.diff = config::read_as<double>("pitch_d");    // Демпфирующая составляющая по дифференту
    settings.pitch.min  = config::read_as<double>("pitch_min");  // Насыщение по дифференту
    settings.pitch.max  = config::read_as<double>("pitch_max");  // Насыщение по дифференту
    //...
}

/// Элементарная шаблонная функция pd-регулятора
/// TODO: НУЖНО ВЕЗДЕ ПРОВЕРИТЬ ЗНАКИ!!!
template <class Value, class Settings>
double regulator_pd(Value value, Settings settings)
{
    double res = value.prop * settings.prop - value.diff * settings.diff;
    if(res > settings.max) return settings.max;
    if(res < settings.min) return settings.min;
    return res;
}
