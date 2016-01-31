#pragma once

#include <map>
#include <list>
#include <string>
#include <memory>
#include <utility>

#include "recognizer.h"


template<typename T>
class Singleton
{
private:
    Singleton() {}
    Singleton( const Singleton&);
    Singleton& operator=( Singleton& );

public:
    static T& instance()
    {
        static T inst;
        return inst;
    }
};

class RecognizerFactory
{
public:
    template<typename CustomRecognizer>
    void reg(std::string name)
    {
        recognizers[name] = std::make_shared<Recognizer<CustomRecognizer> >();
    }

    void init_all(const YamlReader& cfg,
            Ipc mode,
            ipc::Communicator& comm)
    {
        for (auto& elem : recognizers) {
            elem.second->init(cfg, mode, comm);
        }
    }

    std::map<std::string, std::shared_ptr<RecognizerBase> > recognizers;
};

using RegisteredRecognizers = Singleton<RecognizerFactory>;

template<typename CustomRecognizer>
struct StaticRegistrator
{
    StaticRegistrator(std::string name)
    {
        RegisteredRecognizers::instance().reg<CustomRecognizer>(name);
    }
};

#define REGISTER_RECOGNIZER(CLASSNAME, CONFIG_NAME)                     \
StaticRegistrator<CLASSNAME> CLASSNAME##CONFIG_NAME(#CONFIG_NAME);

