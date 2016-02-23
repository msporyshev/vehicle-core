#pragma once

#include <map>
#include <list>
#include <string>
#include <memory>
#include <utility>

#include <factory/static_registrator.h>
#include "recognizer.h"

class RecognizerFactory: public Factory<RecognizerBase>
{
public:
    void init_all(const YamlReader& cfg,
            ipc::CommunicatorPtr comm)
    {
        for (auto& elem : obj_) {
            ROS_INFO_STREAM("Init recognizer " << elem.first);
            elem.second->init(cfg.node(elem.first), comm);
        }
    }
};

template<typename RecognizerImpl>
using RecognizerRegistrator = StaticRegistrator<RecognizerFactory, Recognizer<RecognizerImpl> >;

using RegisteredRecognizers = Singleton<RecognizerFactory>;

#define REGISTER_RECOGNIZER(CLASSNAME, CONFIG_NAME) \
static RecognizerRegistrator<CLASSNAME> BOOST_PP_CAT(CLASSNAME##CONFIG_NAME, __COUNTER__) \
    (#CONFIG_NAME, new Recognizer<CLASSNAME>);


