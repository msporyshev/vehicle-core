#include "registry.h"

#include <log.h>

using namespace std;

shared_ptr<Producers> Registry::data;
shared_ptr<MsgNames> Registry::used_msgs;

Producers& Registry::get_producers()
{
    if (!data) {
        data = shared_ptr<Producers>(new Producers());
    }
    return *data;
}

MsgNames& Registry::get_used_msgs()
{
    if (!used_msgs) {
        used_msgs = shared_ptr<MsgNames>(new MsgNames());
    }
    return *used_msgs;
}

void Registry::add(string name, string msg_name, shared_ptr<BaseRegulProducer> producer)
{
    auto& producers = get_producers();
    auto& msgs = get_used_msgs();
    for (auto used_msg : msgs) {
        if (used_msg == msg_name) {
            LOG << "Warning: trying to add used msg " << msg_name << " to registry" << endl;
            throw;
        }
    }
    msgs.push_back(msg_name);
    if (producers.find(name) == producers.end()) {
        producers[name] = {};
    }
    producers.at(name).push_back(producer);
}

vector<shared_ptr<BaseRegulProducer>> Registry::get(std::string name)
{
    auto& producers = get_producers();
    if (producers.find(name) == producers.end()) {
        LOG << "regul producer not found: " << name << endl;
    }
    return producers.at(name);
}
