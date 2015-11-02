#pragma once

enum class WaitMode
{
    // ожидать информации о выполнении регулирования (SUCCESS, TIMEOUT или STOPPED)
    WAIT,
    // не ожидать информации о выполнении регулирования
    DONT_WAIT
};
