#ifndef YAML_READER_H
#define YAML_READER_H

#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <list>
#include <utils/basic.h>
#include <stdexcept>

#include "log.h"

template<typename T>
T get_as(const YAML::Node& node)
{
    return node.as<T>();
}

template<>
inline YAML::Node get_as(const YAML::Node& node)
{
    if (!node.IsDefined()) {
        throw std::runtime_error("node is undefined");
    }
    return node;
}

template<typename T>
void log_var(std::string param_name, T& var)
{
    LOG << param_name << ": " << var << std::endl;
}

template<typename T>
void log_var(std::string param_name, std::vector<T> v)
{
    LOG << param_name << ": ";
    for (const auto& e : v) {
        LOG_WT << e << ", ";
    }
    LOG_WT << std::endl;
}

class YamlReaderException : public std::runtime_error
{
};

class YamlReader
{
public:

    YamlReader();
    explicit YamlReader(YAML::Node source);
    explicit YamlReader(const char* source);
    YamlReader(std::string source, std::string package_name);

    static std::string to_filename(YAML::Node node);
    static std::string to_filename(std::string shortname);

    static YamlReader from_file(std::string filename, std::string package_name);
    static YamlReader from_string(std::string raw_config);

    YamlReader& add_from_string(std::string raw_config);
    YamlReader& add_source(std::string filename);
    YamlReader& add_source(YAML::Node node);

    YamlReader& clear_sources();

    YamlReader& set_silent_mode();
    YamlReader& set_base_dir(const char* base_dir);
    YamlReader& set_package(std::string package_name);

    YamlReader node(std::string name) const
    {
        return YamlReader(read_as<YAML::Node>(name));
    }

    template<typename T>
    T read_as(std::string param_name) const
    {
        T result;
        read_param(result, param_name);
        return result;
    }

    template<typename T>
    bool is_param_readable(T& var, std::string param_name) const
    {
        if (configs.empty()) {
            LOG << "sources list to read " << param_name
                << " from is empty, be careful" << std::endl;
        }
        for (auto it = configs.begin(); it != configs.end(); ++it) {
            try {
                auto config = *it;
                var = get_as<T>(config[param_name]);
                if (!silent_mode) {
                    log_var(param_name, var);
                }
                break;
            } catch (std::runtime_error e) {
                if (std::next(it, 1) == configs.end()) {
                    //on_not_found(e);
                    return false;
                }
            }
        }

        return true;
    }

    template<typename T>
    T read_as_throw(std::string name) const
    {
        T res;
        if (!is_param_readable(res, name)) {
            throw std::runtime_error("param " + name + " not found");
        }

        return res;
    }

    template<typename T>
    T read_param(T& var, std::string param_name) const
    {
        if (!is_param_readable(var, param_name)) {
            LOG << "* param " << param_name << " could not be read" << std::endl;
            var = T();
        }

        return var;
    }

private:
    std::list<YAML::Node> configs;
    std::string base_dir = "config/";
    bool silent_mode;
};

template<>
inline YamlReader get_as(const YAML::Node& node)
{
    return YamlReader(get_as<YAML::Node>(node));
}



inline std::string rm_right_dash(std::string name) {
    if (name.back() == '_') {
        return name.substr(0, name.size() - 1);
    }

    return name;
}

template<typename T>
class ParamBase
{
public:
    ParamBase(T value): value_(value) {}

    virtual const T& get() const
    {
        return value_;
    }
protected:
    T value_;
};


template<typename T>
class AutoReadParam: public ParamBase<T>
{
public:
    AutoReadParam(std::string name, const YamlReader& cfg):
            ParamBase<T>(cfg.read_as_throw<T>(rm_right_dash(name))) {}
};


template<typename T>
class AutoReadParamOptional: public ParamBase<T>
{
public:
    AutoReadParamOptional(std::string name, const YamlReader& cfg, const T& default_value):
            ParamBase<T>(cfg.read_as<T>(rm_right_dash(name))),
            isset_(cfg.is_param_readable(param_, rm_right_dash(name)))
    {
        if (!isset_) {
            param_ = default_value;
        }
    }

    const T& get() const override
    {
        return param_;
    }

    bool is_set() const
    {
        return isset_;
    }
private:
    bool isset_;
    T param_;
};

#define AUTOPARAM_OPTIONAL(type, name, default_value)\
    AutoReadParamOptional<type> name = AutoReadParamOptional<type>(#name, cfg_, default_value)
#define AUTOPARAM(type, name)\
    AutoReadParam<type> name = AutoReadParam<type>(#name, cfg_);

#define SET_PARAM(PARAM_NAME)\
    read_param(PARAM_NAME, #PARAM_NAME)

#endif // YAML_READER_H
