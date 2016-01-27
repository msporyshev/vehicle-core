#pragma once

enum class TcpHeaders {
    Can,
    FirmwareCommand,
    FirmwarePacket,
    Manipulator,
    Light,
    LightRgb,
    SwitchDevices,
    Imu,
    Echo,
    Gans,
    Camera,
    Uart,
    Pwm,
    UdpConfug,
    SystemFlags,
    CasConfig,
    CasCommand
};

enum class UdpHeaders {
    Gyro = 128,
    Accelerometer = 129,
    Compass = 130,
    DepthSensor = 131,
    InnerPresure = 132,
    Facilities = 133,
    Leak = 134,
    FirmwareReady = 135,
    ShortCircuit = 136,
    Adc = 137,
    AdcExternal = 138,
    Compensator = 139
};