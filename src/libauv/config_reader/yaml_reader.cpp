#include "yaml_reader.h"

#include <ros/package.h>

using namespace std;

YamlReader::YamlReader()
{
    silent_mode = false;
}

YamlReader::YamlReader(YAML::Node source):
    YamlReader()
{
    add_source(source);
}

YamlReader::YamlReader(string source, string package_name):
    YamlReader()
{
    set_package(package_name);
    add_source(source);
}

YamlReader::YamlReader(const char* source):
    YamlReader()
{
    add_source(source);
}

string YamlReader::to_filename(YAML::Node node)
{
    return node.as<std::string>() + ".yml";
}

string YamlReader::to_filename(string shortname)
{
    return shortname + ".yml";
}

YamlReader YamlReader::from_file(string filename, string package_name)
{
    return YamlReader(filename, package_name);
}

YamlReader YamlReader::from_string(string raw_config)
{
    YamlReader cfg;
    return cfg.add_from_string(raw_config);
}

YamlReader& YamlReader::add_from_string(string raw_config) {
    try {
        configs.push_back(YAML::Load(raw_config));
    } catch (YAML::Exception &e) {
        LOG << "loading from string: " << raw_config << endl
            << "got error: " << e.what() << endl;
            throw;
    }
    return *this;
}


YamlReader& YamlReader::add_source(string filename)
{
    try {
        configs.push_back(YAML::LoadFile(base_dir + filename));
    } catch (YAML::Exception &e) {
        LOG << "caught exception in file loading: '" << base_dir + filename << "' "
            << " (" << e.what() << ")" << std::endl;
        throw;
    }
    return *this;
}

YamlReader& YamlReader::add_source(YAML::Node node)
{
    configs.push_back(node);
    return *this;
}

YamlReader& YamlReader::clear_sources()
{
    configs.clear();
    return *this;
}

YamlReader& YamlReader::set_silent_mode()
{
    silent_mode = true;
    return *this;
}

YamlReader& YamlReader::set_base_dir(const char* base_dir)
{
    this->base_dir = base_dir;
    return *this;
}

YamlReader& YamlReader::set_package(std::string package_name)
{
    base_dir = ros::package::getPath(package_name) + "/config/";
    return *this;
}

