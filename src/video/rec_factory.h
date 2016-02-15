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
            elem.second->init(cfg.node(elem.first), comm);
        }
    }
};

template<typename RecognizerImpl>
using RecognizerRegistrator = StaticRegistrator<RecognizerFactory, Recognizer<RecognizerImpl> >;

using RegisteredRecognizers = Singleton<RecognizerFactory>;

#define REGISTER_RECOGNIZER(CLASSNAME, CONFIG_NAME) \
RecognizerRegistrator<CLASSNAME> CLASSNAME##CONFIG_NAME(#CONFIG_NAME, new Recognizer<CLASSNAME>);


