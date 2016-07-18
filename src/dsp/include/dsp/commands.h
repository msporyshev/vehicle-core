#pragma once

#include <map>
#include <cstdint>

namespace dsp {

    enum class CommandType : std::uint8_t {
        DspOff,
        DspOn,
        Reserved0,
        Reserved1,
        DebugOn,
        DebugOff,
        f25kHz = 33,
        f26kHz = 35,
        f27kHz = 36,
        f28kHz = 37,
        f29kHz = 39,
        f30kHz = 40,
        f31kHz = 41,
        f32kHz = 43,
        f33kHz = 44,
        f34kHz = 45,
        f35kHz = 47,
        f36kHz = 48,
        f37kHz = 49,
        f38kHz = 51,
        f39kHz = 52,
        f40kHz = 53,
        Count,
    };

    const std::map<int, CommandType> dsp_command = {
        {25, CommandType::f25kHz},
        {26, CommandType::f26kHz},
        {27, CommandType::f27kHz},
        {28, CommandType::f28kHz},
        {29, CommandType::f29kHz},
        {30, CommandType::f30kHz},
        {31, CommandType::f31kHz},
        {32, CommandType::f32kHz},
        {33, CommandType::f33kHz},
        {34, CommandType::f34kHz},
        {35, CommandType::f35kHz},
        {36, CommandType::f36kHz},
        {37, CommandType::f37kHz},
        {38, CommandType::f38kHz},
        {39, CommandType::f39kHz},
        {40, CommandType::f40kHz},
    };

} // namespace dsp
