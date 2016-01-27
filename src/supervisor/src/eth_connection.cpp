
#include "supervisor/eth_connection.h"

using namespace std;

EthConnection::EthConnection()
{

}

EthConnection::~EthConnection()
{
    close_udp();
    close_tcp();
}


void EthConnection::state_update()
{
    if (ros::Time::now().toSec() > time_tcp_send_last.toSec() + connection_settings.period_tcp_send) {
        time_tcp_send_last = ros::Time::now();
        tcp_send();
    }
    if (ros::Time::now().toSec() > time_udp_listen_last.toSec() + connection_settings.period_udp_listen) {
        time_udp_listen_last = ros::Time::now();
        udp_listen();
    }
}

void EthConnection::init_connection_settings(EthSettings settings)
{
    if (settings.udp_port != connection_settings.udp_port || 
       settings.period_udp_listen != connection_settings.period_udp_listen ||
       settings.ip_address != connection_settings.ip_address) {
        close_udp();
        init_udp();
    }
    
    if (settings.tcp_port != connection_settings.tcp_port || 
       settings.period_tcp_send != connection_settings.period_tcp_send ||
       settings.ip_address != connection_settings.ip_address) {
        close_tcp();
        init_tcp();
    }

    connection_settings = settings;
}

bool EthConnection::get_download_data(std::vector<unsigned char>& data)
{
    if (!is_udp_configurated) {
        ROS_WARN_STREAM("UDP connection is not initialized");
        return false;
    }
    
    for (auto &el: downloaded_data) {
        data.push_back(el);
    }

    return true;
}
bool EthConnection::set_upload_data(std::vector<unsigned char> data)
{
    if (!is_tcp_configurated) {
        ROS_WARN_STREAM("TCP connection is not initialized");
        return false;
    }

    if ((data.size() + uploaded_data.size()) > UPLOAD_DATA_MAX_SIZE) {
        ROS_ERROR_STREAM("Uploaded data is too big");
        return false;
    }

    for (auto &el: data) {
        uploaded_data.push_back(el);
    }

    return true;
}

void EthConnection::tcp_send()
{
    char* sending_data = new char[uploaded_data.size()];
    memcpy(sending_data, uploaded_data.data(), uploaded_data.size());

    close_tcp();
    init_tcp();
    send(tcp_socket, sending_data, uploaded_data.size(), 0);
    
    uploaded_data.clear();
    delete[] sending_data; 
}
void EthConnection::udp_listen()
{
    char udp_data[10000];
    int bytes_read = recv(udp_socket, udp_data, 10000, 0);
    for (int i = 0; i < bytes_read; i++) {
        downloaded_data.push_back(udp_data[i]);
    }
}

void EthConnection::init_tcp()
{
    tcp_socket = socket(AF_INET, SOCK_STREAM, 0);

    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(connection_settings.tcp_port);
    addr.sin_addr.s_addr = inet_addr(connection_settings.ip_address.c_str());
    if (connect(tcp_socket, (const sockaddr *)&addr, sizeof(addr)) < 0) {
        ROS_ERROR_STREAM("Connection failed for TCP");
    } else {
        is_tcp_configurated = true;
    }
}

void EthConnection::init_udp()
{
    udp_socket = socket(AF_INET, SOCK_DGRAM, 0);

    fcntl(udp_socket, F_SETFL, O_NONBLOCK);

    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(connection_settings.udp_port);
    addr.sin_addr.s_addr = INADDR_ANY; //inet_addr(udp_address.c_str());

    if (bind(udp_socket, (const sockaddr *)&addr, sizeof(addr)) < 0) {
        ROS_ERROR_STREAM("Connection failed for UDP");
    } else {
        is_udp_configurated = true;
    }
}
    
void EthConnection::close_tcp()
{
    close(tcp_socket);
}
void EthConnection::close_udp()
{
    close(udp_socket);
}
