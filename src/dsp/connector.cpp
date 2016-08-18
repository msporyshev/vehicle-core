#include "connector.h"

#include "com/com.h"

#include <iostream>

using namespace std;

int ComConnector::open()
{
    while (try_open_counter < max_try_open_counter) {
        handle = COM_open(name.c_str(), baudrate);

        ROS_INFO_STREAM("COM connection to the DSP (try #" << try_open_counter << ")");
        
        if (handle == -1) {
            ROS_ERROR("Failed to connect, invalid descriptor");
            try_open_counter++;
            ros::Duration(1.0).sleep();
        }
        else {
            ROS_INFO("Successfully connected");
            return 0;
        }
    }

    ROS_ERROR("Failed to connect, check connection to dsp");

    return -1;
}

int ComConnector::close()
{
    COM_close(&handle);

    return 0;
}

int ComConnector::read_package()
{
    int bytes_received = read(handle, &(buffer[counter]), buffer_size - counter);
    if (bytes_received > 0) {
        counter += bytes_received;
        if (counter >= buffer_size) {
            counter = 0;
            return 0;
        }
    }
    
    return -1;
}

int ComConnector::write_package(unsigned char* buf, DWORD Tx_bytes)
{
    if (Tx_bytes > 0) {
        write(handle, buf, Tx_bytes);
    }        

    return 0;
}

int ComConnector::purge_handle()
{
    return tcflush(handle, TCIOFLUSH);
};

int UsbConnector::open()
{
    FT_STATUS ft_status = FT_SetVIDPID(0x0403, 0xF69A);

    if (ft_status != FT_OK) {
        ROS_ERROR("FT_SetVIDPID failed");
        return -1;
    }

    while (try_open_counter < max_try_open_counter) {
        FT_STATUS ft_status = FT_Open(0, &handle);
        ROS_INFO_STREAM("USB connection to the DSP (try #" << try_open_counter << ")");
        
        if (ft_status != FT_OK) {
            ROS_ERROR("Failed to connect, invalid descriptor");
            try_open_counter++;
            ros::Duration(1.0).sleep();
        }
        else {
            ROS_INFO("Successfully connected");
            ft_status = FT_Purge(handle, FT_PURGE_RX | FT_PURGE_TX);

            if (ft_status != FT_OK) {
                ROS_ERROR("FT_Purge failed");
                return -1;
            }

            return 0;
        }
    }

    ROS_ERROR("Failed to connect, check connection to dsp");

    return -1;
}

int UsbConnector::close()
{
    FT_STATUS ft_status = FT_Purge(handle, FT_PURGE_RX | FT_PURGE_TX);

    if (ft_status != FT_OK) {
        ROS_ERROR("FT_Purge failed");
    }
    
    ft_status = FT_Close(handle);

    if (ft_status != FT_OK) {
        ROS_ERROR("FT_Close failed");
        return -1; 
    }
       
    return 0;
}

int UsbConnector::read_package()
{
    FT_STATUS ft_status;
    DWORD Rx_bytes, bytes_received;
    
    ft_status = FT_GetQueueStatus(handle, &Rx_bytes);

    if (ft_status != FT_OK) {
        ROS_ERROR("FT_GetQueueStatus failed");
        return -1;
    }        
    if (Rx_bytes > 0) {
        if (Rx_bytes > buffer_size - counter)
            Rx_bytes = buffer_size - counter;

        ft_status = FT_Read(handle, &(buffer[counter]), Rx_bytes, &bytes_received);

        if (ft_status != FT_OK) {
            ROS_ERROR("FT_Read failed");
            return -1;
        }            

        if (bytes_received >= 0) {
            counter += bytes_received;

            if (counter >= buffer_size) {
                counter = 0;
                return 0;
            }
        }

        if (Rx_bytes != bytes_received) {
            ROS_ERROR("Buffer error, FT_Read failed");
            return -1;
        }
          
    }

    return -1;
}

int UsbConnector::write_package(unsigned char* buf, DWORD Tx_bytes)
{
   DWORD bytes_written;

    if (Tx_bytes > 0) {
        FT_STATUS ft_status = FT_Write(handle, buf, Tx_bytes, &bytes_written);

        if (ft_status != FT_OK) {
            ROS_ERROR("FT_Write failed");
            return -1;
        }        

        if (Tx_bytes != bytes_written) {
            ROS_ERROR("Buffer error, FT_Write failed");
            return -1;
        }        
    }

    return 0;
}

int UsbConnector::purge_handle()
{
    FT_STATUS ft_status = FT_Purge(handle, FT_PURGE_RX | FT_PURGE_TX);
    
    if (ft_status != FT_OK) {
        ROS_ERROR("FT_Purge failed");
        return -1;
    }        

    return 0;
}