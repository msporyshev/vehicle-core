
#include "supervisor/eth_connection.h"

using namespace std;

EthConnection::EthConnection()
{
    ros::Time time_tcp_send_last_ = ros::Time::now();
    ros::Time time_udp_listen_last_ = ros::Time::now();

    is_tcp_configurated_ = false;
    is_udp_configurated_ = false;

}

EthConnection::~EthConnection()
{
    close_udp();
    close_tcp();
}


void EthConnection::state_update()
{
    if (ros::Time::now().toSec() > time_tcp_send_last_.toSec() + connection_settings_.period_tcp_send) {
        time_tcp_send_last_ = ros::Time::now();
        tcp_send();
    }
    if (ros::Time::now().toSec() > time_udp_listen_last_.toSec() + connection_settings_.period_udp_listen) {
        time_udp_listen_last_ = ros::Time::now();
        udp_listen();
    }
}

void EthConnection::init_connection_settings(EthSettings settings)
{
    connection_settings_ = settings;
    
    ROS_DEBUG_STREAM("UDP port: " << settings.udp_port);
    ROS_DEBUG_STREAM("UDP ip: " << settings.ip_address);
    ROS_DEBUG_STREAM("UDP period: " << settings.period_udp_listen);

    close_udp();
    init_udp();
}

bool EthConnection::get_download_data(std::vector<unsigned char>& data)
{
    if (!is_udp_configurated_) {
        ROS_WARN_STREAM("UDP connection is not initialized");
        return false;
    }
    
    for (auto &el: downloaded_data_) {
        data.push_back(el);
    }

    downloaded_data_.clear();
    return true;
}

bool EthConnection::set_upload_data(std::vector<unsigned char> data)
{
    if ((data.size() + uploaded_data_.size()) > UPLOAD_DATA_MAX_SIZE) {
        ROS_ERROR_STREAM("Uploaded data is too big");
        return false;
    }

    for (auto &el: data) {
        uploaded_data_.push_back(el);
    }

    return true;
}

void EthConnection::tcp_send()
{
    if (uploaded_data_.size() == 0) {
        return;
    } else {
        ROS_DEBUG_STREAM("Sended bytes: " << uploaded_data_.size());
    }

    char* sending_data = new char[uploaded_data_.size()];
    memcpy(sending_data, uploaded_data_.data(), uploaded_data_.size());
    
    init_tcp();
    if (!is_tcp_configurated_) {
        return;
    }

    int data_send = send(tcp_socket_, sending_data, uploaded_data_.size(), 0);
    close_tcp();

    uploaded_data_.clear();
    delete[] sending_data; 
}

void EthConnection::udp_listen()
{
    if (!is_udp_configurated_) {
        ROS_ERROR_STREAM("UDP is not configurated");
        return;
    }

    char udp_data[10000];
    int bytes_read = recv(udp_socket_, udp_data, 10000, 0);
    for (int i = 0; i < bytes_read; i++) {
        downloaded_data_.push_back(udp_data[i]);
    }
    ROS_DEBUG_STREAM("Received bytes from upd port: " << bytes_read);
}

void EthConnection::init_tcp()
{
    tcp_socket_ = socket(AF_INET, SOCK_STREAM, 0);

    ROS_INFO_STREAM("TCP port: " << connection_settings_.tcp_port);
    ROS_INFO_STREAM("TCP ip: " << connection_settings_.ip_address);
    ROS_INFO_STREAM("TCP period: " << connection_settings_.period_tcp_send);

    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(connection_settings_.tcp_port);
    addr.sin_addr.s_addr = inet_addr(connection_settings_.ip_address.c_str());
    if (connect(tcp_socket_, (const sockaddr *)&addr, sizeof(addr)) < 0) {
        ROS_ERROR_STREAM("Connection failed for TCP");
        is_tcp_configurated_ = false;
    } else {
        is_tcp_configurated_ = true;
    }
}

void EthConnection::init_udp()
{
    udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket_ == -1) {
        ROS_ERROR_STREAM("UDP socket creation failed");
    }

    fcntl(udp_socket_, F_SETFL, O_NONBLOCK);

    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(connection_settings_.udp_port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(udp_socket_, (const sockaddr *)&addr, sizeof(addr)) < 0) {
        ROS_ERROR_STREAM("Connection failed for UDP");
    } else {
        is_udp_configurated_ = true;
    }
}
    
void EthConnection::close_tcp()
{
    close(tcp_socket_);
}

void EthConnection::close_udp()
{
    close(udp_socket_);
}
