#pragma once

#include <map>

template<typename T>
class Singleton
{
private:
    Singleton() {}
    Singleton(const Singleton&);
    Singleton& operator=(Singleton&);

public:
    static T& instance()
    {
        static T inst;
        return inst;
    }
};

template<typename BaseClass>
class Factory
{
public:
    template<typename Class>
    void reg(std::string name, Class* obj_ptr)
    {
        obj[name] = std::shared_ptr<Class>(obj_ptr);
    }

    BaseClass& get(std::string name)
    {
        return obj.at(name);
    }

protected:
    std::map<std::string, std::shared_ptr<BaseClass> > obj;
};


template<typename Factory, typename RegisterClass>
struct StaticRegistrator
{
    StaticRegistrator(std::string name, RegisterClass* obj_ptr)
    {
        Singleton<Factory>::instance().reg(name, obj_ptr);
    }
};
