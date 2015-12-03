#pragma once

enum class MoveMode
{
    // режим без стабилизации курса
    HEADING_FREE,

    // режим с сохранением текущего курса
    HOVER,

    // режим с сохранением направления к точке
    CRUISE,

    // режим автоматического выбора режима в зависимости от расстояния до точки
    AUTO,
};