#pragma once

#include <map>
#include <memory>
#include <string>
#include <ros/ros.h>

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
        obj_[name] = std::shared_ptr<Class>(obj_ptr);
    }

    std::shared_ptr<BaseClass> get(std::string name)
    {
        return obj_.at(name);
    }

protected:
    std::map<std::string, std::shared_ptr<BaseClass> > obj_;
};

template<typename BaseClass, typename InitFunc>
class LazyFactory
{
public:
    void reg(std::string name, InitFunc cons)
    {
        constructor_[name] = cons;
    }

    template<typename ... Args>
    std::shared_ptr<BaseClass> init(std::string name, Args ... args)
    {
        auto ptr = std::shared_ptr<BaseClass>(constructor_.at(name)(args...));
        obj_[name] = ptr;
        return ptr;
    }

    template<typename ... Args>
    void init_all(Args ... args)
    {
        for (auto& cons : constructor_) {
            obj_.insert(make_pair(cons.first, std::shared_ptr<BaseClass>(cons.second(args...))));
        }
    }

    BaseClass& get(std::string name)
    {
        return *obj_.at(name);
    }

protected:
    std::map<std::string, std::shared_ptr<BaseClass> > obj_;
    std::map<std::string, InitFunc> constructor_;
};


template<typename FactoryImpl, typename Register>
struct StaticRegistrator
{
    StaticRegistrator(std::string name, Register obj)
    {
        Singleton<FactoryImpl>::instance().reg(name, obj);
    }

    StaticRegistrator(std::string name, Register* obj_ptr)
    {
        Singleton<FactoryImpl>::instance().reg(name, obj_ptr);
    }
};

