#pragma once

#include "ros/ros.h"
#include <libipc/ipc.h>

#include <vector>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>

#define UPLOAD_DATA_MAX_SIZE    5000  

struct EthSettings
{
    std::string ip_address;
    int         tcp_port;
    int         udp_port;
    double      period_tcp_send;
    double      period_udp_listen;
};

class EthConnection
{

public:
    EthConnection();
    ~EthConnection();

    bool get_download_data(std::vector<unsigned char>& data);
    bool set_upload_data(std::vector<unsigned char> data);

    void init_connection_settings(EthSettings settings);

    void state_update();
private:

    void tcp_send();
    void udp_listen();

    void init_tcp();
    void init_udp();
    
    void close_tcp();
    void close_udp();

    int tcp_socket_;
    int udp_socket_;

    EthSettings connection_settings_;

    bool is_tcp_configurated_;
    bool is_udp_configurated_;

    std::vector<unsigned char> uploaded_data_;
    std::vector<unsigned char> downloaded_data_;

    ros::Time time_tcp_send_last_;
    ros::Time time_udp_listen_last_;
};