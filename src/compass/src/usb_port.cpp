/**
\file
\brief Реализация функций работы с USB-устройством

В данном файле находятся все фукции для работы с USB-устройством:
инициализация, чтение, запись.

\ingroup compass_node

*/

#include "compass/usb_port.h"

// #define VID      0x058F
// #define PID      0x6387
#define VID         0x0483
#define PID         0x600d
#define DEV_INTF    0    // номер интерфейса

// #define ENDPOINT_OUT 0x01
// #define ENDPOINT_IN  0x82
#define ENDPOINT_OUT    0x03
#define ENDPOINT_IN     0x81

#define USB_PACKET_SIZE 64
#define USB_DEBUG_LEVEL 3

libusb_device_handle* device = NULL;
libusb_hotplug_callback_handle callback_handle;

libusb_device_handle* find_device(libusb_device **devs, int size, int vid, int pid);

int hotplug_callback(struct libusb_context *ctx, struct libusb_device *dev,
                     libusb_hotplug_event event, void *user_data);

int hotplug_callback(struct libusb_context *ctx, struct libusb_device *dev,
                     libusb_hotplug_event event, void *user_data) 
{
    printf("in callback...\n");
    static libusb_device_handle *handle = NULL;
    struct libusb_device_descriptor desc;
    int rc;

    (void)libusb_get_device_descriptor(dev, &desc);

    if (LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED == event) {
        rc = libusb_open(dev, &handle);
        if (LIBUSB_SUCCESS != rc) {
          printf("Could not open USB device in callback\n");
        } else {
            printf("Opened USB device in callback\n");
        }
    } else if (LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT == event) {
        printf("Device left from system. %d\n", event);
        //if (handle) {
          //libusb_close(handle);
          //handle = NULL;
        // }
    } else {
        printf("Unhandled event %d\n", event);
    }
    
    return 0;
}

libusb_device_handle* find_device(libusb_device **devs, int size, int vid, int pid) 
{
    libusb_device *dev;
    libusb_device_handle *handle;
    int i = 0;

    printf("Trying to find device logging system.\n");
    for(int i = 0; i < size; i++) {
        dev = devs[i];
        struct libusb_device_descriptor desc;
        int r = libusb_get_device_descriptor(dev, &desc);
        if (r < 0) {
            fprintf(stderr, "failed to get device descriptor");
            return NULL;
        }

        printf("%04x:%04x (bus %d, device %d)\n",
            desc.idVendor, desc.idProduct,
            libusb_get_bus_number(dev), libusb_get_device_address(dev));

        if (desc.idVendor == vid && desc.idProduct == pid) {
            printf("Device is found.\n");
            int status = libusb_open(dev, &handle);
            //libusb_device_handle *dev = libusb_open_device_with_vid_pid(NULL, VENDOR_ID, PRODUCT_ID);
            printf("Error status: %d.\n", status);
            return handle;
        }
    }

    return NULL;
}

bool wait_conection()
{

}

bool usb_init_conection()
{
    libusb_init(NULL);   // инициализация
    libusb_set_debug(NULL, USB_DEBUG_LEVEL);  // уровень вывода отладочных сообщений
    
    int status = libusb_hotplug_register_callback(NULL, 
        (libusb_hotplug_event)(LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED | LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT), 
        (libusb_hotplug_flag)0, LIBUSB_HOTPLUG_MATCH_ANY, LIBUSB_HOTPLUG_MATCH_ANY, \
        LIBUSB_HOTPLUG_MATCH_ANY, hotplug_callback, NULL, \
        &callback_handle);
    printf("status of register: %d\n", status);

    libusb_device **devs;
    ssize_t cnt = libusb_get_device_list(NULL, &devs);
    if (cnt < 0) {
        return 1;
    }

    libusb_device_handle *handle = find_device(devs, cnt, VID, PID);
    libusb_free_device_list(devs, 1);

    device = handle;
    
    if (handle == NULL) {
        printf("Device not connect.\n");
        return 1;
    }

    if (libusb_kernel_driver_active(device, DEV_INTF)) {
        libusb_detach_kernel_driver(device, DEV_INTF);
    }

    if (libusb_claim_interface(device,  DEV_INTF) < 0) {
        printf("Interface error.\n");
        return 1;
    }

    return 0;
}

bool usb_close_connection()
{
    if(device == 0) {
        printf("Device is not initialized\n");
        return 1;
    }

    printf("try to deregister callback\n");
    libusb_hotplug_deregister_callback(NULL, callback_handle);
    printf("try to release interface\n");
    libusb_release_interface(device, DEV_INTF); // Освобождаем интерфейс (важно! иначе потом не сможете работать с устройством)
    printf("try to attach to kernel\n");
    libusb_attach_kernel_driver(device, DEV_INTF);
    // printf("try to close\n");
    // libusb_close(device); Не знаю почему, но эта функция наглухо вешает программу
    printf("try to exit\n");
    libusb_exit(NULL);

    return 0;
}

bool usb_send_data(uint8_t* data, int size)
{
    if(device == NULL) {
        printf("Device is not initialized\n");
        return 1;
    }

    if(size > USB_PACKET_SIZE) {
        printf("Size > %d bytes\n", USB_PACKET_SIZE);
        return 1;
    }
    
    uint8_t dataDown[USB_PACKET_SIZE];
    memset(dataDown, 0, USB_PACKET_SIZE);

    for(int i = 0; i < size; i++) {
        dataDown[i] = data[i];
    }

    int actual_length;
    int status = libusb_bulk_transfer(device, ENDPOINT_OUT, dataDown,
                                        size, &actual_length, 0);

    if (status == 0 && actual_length == size) {
        printf("Data sending to USB device.\n");
        return 0;
    } else {
        printf("Something went wrong (status = %i, actual_length = %i , sizeof(data) = %d ).\n",
               status, actual_length, size);
        return 1;
    }
}

int usb_read_data(uint8_t* data)
{
    if(device == NULL) {
        printf("Device is not initialized\n");
        return 1;
    }

    int received = 0;
    uint8_t dataUp[USB_PACKET_SIZE];
    
    int status = 0;
    // libusb_handle_events (NULL);
    status = libusb_bulk_transfer(device, ENDPOINT_IN, dataUp, 64, &received, 5000);
    
    if(status != LIBUSB_SUCCESS) {
        return 0;
    }
    
    memcpy(data, dataUp, received);
    return received;
}