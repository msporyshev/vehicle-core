#pragma once

enum class MoveMode
{
    /**
    Режим без стабилизации курса
    */
    HEADING_FREE,

    /**
    Режим с сохранением текущего курса
    */
    HOVER,

    /**
    Режим с сохранением направления к точке
    */
    CRUISE,

    /**
    Режим автоматического выбора режима в зависимости от расстояния до точки
    */
    AUTO,
};
