//это - библиотека, которая инкапсулирует работу с IPC
//(если надо использовать ROS, то мы готовим другую библиотеку с таким же интрефейсом)
#ifndef IPC_LIB_H_
#define IPC_LIB_H_

#include "ipc.h"
#include <iostream>
#include <type_traits>
#include <functional>
#include <string>
#include <vector>
#include <cstring>

namespace Central {

/*Для подписки на IPC сообщения*/

#define subscribe(object, method) \
    register_incoming_message(&std::remove_reference<decltype(object)>::type::method, &object)

// Чтобы сделать unsubscribe, нужно вызвать функцию unsubscribe

template<typename M>
void handler(MSG_INSTANCE msgInstance, void *callData, void *clientData)
{
    auto callback = (std::function<void (M)>*) clientData;
    (*callback)(M((typename M::IPC_TYPE*)callData));
    IPC_freeData(IPC_msgFormatter(M::IPC_NAME), callData);
}

inline void timer_handler(void *clientData, unsigned long currentTime, unsigned long scheduledTime)
{
    auto callback = (std::function<void()>*) clientData;
    (*callback)();
}

template<typename M>
void define()
{
    if (IPC_defineMsg(M::IPC_NAME, IPC_VARIABLE_LENGTH, M::IPC_FORMAT) != IPC_OK) {
        std::cout << "Cannot define msg " << M::IPC_NAME << ". " << std::endl;
        throw;
    }
}


template<typename M>
void register_incoming_message(std::function<void (M)> callback) {
    define<M>();
    IPC_subscribeData(M::IPC_NAME, handler<M>, new std::function<void (M)>(callback));
}

template<typename M>
void register_incoming_message(void (*callback)(M)) {
    register_incoming_message(std::function<void (M)>(callback));
}

template<typename M, typename Obj>
void register_incoming_message(void (Obj::*callback)(M), Obj* object)
{
    register_incoming_message<M>(std::bind(callback, object, std::placeholders::_1));
}

template<typename M>
void unsubscribe()
{
    IPC_unsubscribe(M::IPC_NAME, handler<M>);
}

/*для работы с таймером*/

#define timer(object, method, timer_period) \
    register_timer(std::bind(&std::remove_reference<decltype(object)>::type::method, &object), timer_period);

inline void register_timer(std::function<void()> callback, double timer_period)
{
    IPC_addTimer(timer_period, TRIGGER_FOREVER, timer_handler, new std::function<void()>(callback));
}

/*для публикации IPC сообщений*/

#define outbox(msg_type) \
    register_outcoming_message<msg_type>();

template<typename M>
void register_outcoming_message()
{
    define<M>();
}

template<typename M>
void publish_message(M msg)
{
    typename M::IPC_TYPE* data = msg.to_ipc();
    IPC_publishData(M::IPC_NAME, data);
    IPC_freeData(IPC_msgFormatter(M::IPC_NAME), data);
}

inline void publish_message(std::string msg_name, std::vector<char> msg_data)
{
    if(IPC_isMsgDefined(msg_name.c_str()) != IPC_OK) {
        std::cout << "Message \"" << msg_name << "\" is not currently defined." << std::endl;
        return;
    }

    char* data = (char*)malloc(sizeof(char) * msg_data.size());
    memcpy(data, msg_data.data(), sizeof(char) * msg_data.size());
    IPC_publish(msg_name.c_str(), msg_data.size(), data);
    free(data);
}

/*для работы с сервером IPC*/

inline void init(const char* task_name, const char* host = NULL)
{
    if (IPC_connectModule(task_name, host) != IPC_OK) {
        std::cout << "Connection error with module " << task_name << " on host " << host << std::endl;
        throw;
    }
}

inline void deinit()
{
    IPC_disconnect();
}

inline void process_messages(int delay)
{
    if (IPC_listenClear(delay) == IPC_Error) {
        std::cout << "Listen clear error" << std::endl;
        throw;
    }
}

inline bool is_module_connected(std::string module_name)
{
    return IPC_isModuleConnected(module_name.c_str());
}

} // namespace Central

#endif
