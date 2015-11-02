#ifndef YAML_READER_H
#define YAML_READER_H

#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <list>
#include <utils/basic.h>
#include <stdexcept>

#include "log.h"

namespace {
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

}

class YamlReaderException : public std::runtime_error
{
};

class YamlReader
{
public:

    YamlReader();
    explicit YamlReader(YAML::Node source);
    explicit YamlReader(std::string source);
    explicit YamlReader(const char* source);

    static std::string to_filename(YAML::Node node);
    static std::string to_filename(std::string shortname);

    YamlReader& add_source(std::string filename);
    YamlReader& add_source(YAML::Node node);

    YamlReader& clear_sources();

    YamlReader& set_silent_mode();
    YamlReader& set_base_dir(const char* base_dir);

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

namespace {
    template<>
    inline YamlReader get_as(const YAML::Node& node)
    {
        return YamlReader(get_as<YAML::Node>(node));
    }
}

#define SET_PARAM(PARAM_NAME)\
    read_param(PARAM_NAME, #PARAM_NAME)

#endif // YAML_READER_H
