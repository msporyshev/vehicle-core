#pragma once

enum class Axis : char
{
    // продольная ось, положительное направление вперед
    TX,
    // поперечная ось, положительное направление вправо
    TY,
    // вертикальная ось, положительное направление вниз
    TZ,
    // ось вращения, положительное направление по часовой стрелке вокруг оси TX
    MX,
    // ось вращения, положительное направление по часовой стрелке вокруг оси TY
    MY,
    // ось вращения, положительное направление по часовой стрелке вокруг оси TZ
    MZ
};
