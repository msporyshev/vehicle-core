#pragma once

enum class WaitMode
{
    /**
    Ожидать информации о выполнении регулирования (SUCCESS, TIMEOUT или STOPPED)
    */
    WAIT,
    /**
    Не ожидать информации о выполнении регулирования
    */
    DONT_WAIT
};
