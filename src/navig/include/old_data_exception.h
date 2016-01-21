#pragma once

#include <exception>

class OldDataException : public std::exception 
{
public:
    OldDataException(int duration) : duration_(duration) {}

    virtual const char* what() const throw()
    {
        return "Received too old data from device: " + duration_;
    }

    int get_duration()
    {
        return duration_;
    }

private:
    int duration_;
};