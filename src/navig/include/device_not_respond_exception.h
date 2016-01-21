#pragma once

#include <exception>

class DeviceNotRespondException : std::exception
{
public:
    virtual const char* what() const throw()
    {
        return "Devices are not respond.";
    }
};