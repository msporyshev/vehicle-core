#pragma once

#include <string>
#include <ros/ros.h>

#include "ftd2xx.h"

enum class ConnectorType {UsbConnectorType, ComConnectorType};

class Connector
{
protected:
    int try_open_counter = 1;
    int max_try_open_counter = 100;
    int counter = 0;

    unsigned char* buffer;
    int buffer_size;

public:
    Connector() {};
    Connector(unsigned char* buf, int buf_size) :
        buffer(buf), buffer_size(buf_size) {}

    virtual ~Connector() {}
    virtual int open() { return 0; }
    virtual int close() { return 0; }
    virtual int read_package() { return 0; }
    virtual int write_package(unsigned char* buf, DWORD Tx_bytes) { return 0; }
    virtual int purge_handle() { return 0; }
};

class ComConnector : public Connector
{
    std::string name;
    int baudrate;
    int handle;

public:
    ComConnector(std::string _name, int _baudrate, unsigned char* buf, int buf_size) :
        Connector(buf, buf_size), name(_name), baudrate(_baudrate) {}

    ~ComConnector() {}
    int open() override;
    int close() override;
    int read_package() override;
    int write_package(unsigned char* buf, DWORD Tx_bytes) override;
    int purge_handle() override 
    { 
        close(); 
        open(); 
        return 0; 
    };
};

class UsbConnector : public Connector
{
    FT_HANDLE handle;

public:
    UsbConnector(unsigned char* buf, int buf_size) :
        Connector(buf, buf_size) {}

    ~UsbConnector() {}
    int open() override;
    int close() override;
    int read_package() override;
    int write_package(unsigned char* buf, DWORD Tx_bytes) override;
    int purge_handle() override;
};
