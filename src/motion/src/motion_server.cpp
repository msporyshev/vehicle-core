#include "motion_server.h"

#include <iostream>
#include <iomanip>

#include "log.h"

#include <motion/MsgCmdStatus.h>
#include <motion/MsgRegul.h>

#include <array>
#include <functional>
#include <memory>
#include <stdexcept>

#include "regulators/regulator.h"

#include "registry.h"

using namespace std;

const string MotionServer::NODE_NAME = "motion";

MotionServer::MotionServer(ipc::Communicator& communicator, const YamlReader& config) :
    communicator_(communicator)
{
    read_config(config);
}

MotionServer::~MotionServer()
{

}

void MotionServer::init_ipc()
{
    communicator_.subscribe("navig", &MotionServer::handle_angles, this);
    communicator_.subscribe("navig", &MotionServer::handle_depth, this);
    communicator_.subscribe("navig", &MotionServer::handle_height, this);
    communicator_.subscribe("navig", &MotionServer::handle_position, this);
    communicator_.subscribe("navig", &MotionServer::handle_velocity, this);

    cmd_status_pub_ = communicator_.advertise<motion::MsgCmdStatus>();
    regul_pub_ = communicator_.advertise<motion::MsgRegul>();
}

void MotionServer::handle_angles(const navig::MsgNavigAngles& msg)
{
    navig.heading = msg.heading;
    navig.pitch = msg.pitch;
    navig.roll = msg.roll;
    process_navig(navig);
}

void MotionServer::handle_depth(const navig::MsgNavigDepth& msg)
{
    navig.depth = msg.depth;
    process_navig(navig);
}

void MotionServer::handle_height(const navig::MsgNavigHeight& msg)
{
    navig.height = msg.height;
    process_navig(navig);
}

void MotionServer::handle_position(const navig::MsgNavigPosition& msg)
{
    navig.position.x = msg.x;
    navig.position.y = msg.y;
    process_navig(navig);
}

void MotionServer::handle_velocity(const navig::MsgNavigVelocity& msg)
{
    navig.velocity_forward = msg.velocity_forward;
    // navig.velocity_depth = msg.velocity_depth;
    process_navig(navig);
}


void MotionServer::run()
{
    ipc::EventLoop loop(10);
    while (loop.ok()) {
        update_activity_list();
    }
}

YAML::Node MotionServer::make_regul_config(string name, YamlReader config, vector<string> rejected_dependencies) const
{
    YAML::Node result;
    if (config.is_param_readable(result, name)) {
        result = config.read_as<YAML::Node>(name);
    } else {
        return result;
    }
    YamlReader regul_config = YamlReader(result).set_silent_mode();

    rejected_dependencies.push_back(name);
    vector<string> dependencies;
    if (regul_config.is_param_readable(dependencies, "dependencies")) {
        YAML::Node dependencies_node;
        for (auto dep : dependencies) {
            for (auto rej_dep : rejected_dependencies) {
                if (dep == rej_dep) {
                    throw std::runtime_error("cycled config dependencies are not allowed");
                }
            }
            auto dep_node = make_regul_config(dep, config, rejected_dependencies);
            dependencies_node[dep] = dep_node;
        }
        result["dependencies"] = dependencies_node;
    }
    return result;
}

void MotionServer::read_config(YamlReader config)
{
    vector<string> regul_names;
    config.read_param(regul_names, "regulators");

    YamlReader params = YamlReader(config.read_as<YAML::Node>("params")).set_silent_mode();
    config.set_silent_mode();
    for (auto name : regul_names) {
        auto producers = Registry::get(name);
        for (auto p : producers) {
            p->init(shared_ptr<RegulStorage>(&pending_list),
                YamlReader(make_regul_config(name, params)).set_silent_mode(),
                communicator_);
        }
        LOG << "regul producer " << name << " initialized" << endl;
    }

    auto limits_config = YamlReader(config.read_as<YAML::Node>("limits"));
    LOG << "reading thrust limits" << endl;
    limits_config.read_param(tx_limit, "tx");
    limits_config.read_param(ty_limit, "ty");
    limits_config.read_param(tz_limit, "tz");
    limits_config.read_param(mx_limit, "mx");
    limits_config.read_param(my_limit, "my");
    limits_config.read_param(mz_limit, "mz");
}

void MotionServer::update_activity_list()
{
    vector<shared_ptr<Regulator>> active_reguls = active_list.get_stored();
    vector<shared_ptr<Regulator>> new_active_reguls;
    for (auto r : active_reguls) {
        if (r->is_actual()) {
            if (r->has_succeeded()) {
                publish_cmd_status(r->get_id(), CmdStatus::SUCCESS);
            }
            new_active_reguls.push_back(r);
        } else {
            r->finish();
            publish_cmd_status(r->get_id(), CmdStatus::TIMEOUT);
        }
    }
    active_list.clear_stored();
    active_list.add(new_active_reguls);

    for (auto r : pending_list.get_conflicted()) {
        r->finish();
        publish_cmd_status(r->get_id(), CmdStatus::STOPPED);
    }
    for (auto r : pending_list.get_stored()) {
        r->activate();
    }
    active_list.add(pending_list.get_stored());
    for (auto r : active_list.get_conflicted()) {
        r->finish();
        publish_cmd_status(r->get_id(), CmdStatus::STOPPED);
    }

    pending_list.clear_all();
    active_list.clear_conflicted();
}

void MotionServer::update_thrusts(const NavigInfo& msg)
{
    motion::MsgRegul result;

    array<double, 6> thrusts = { {0.0, 0.0, 0.0, 0.0, 0.0, 0.0} };

    string log_str = "";
    for (auto r : active_list.get_stored()) {
        r->handle_navig(msg);
        auto cur_thrusts = r->get_thrusts();
        auto log_headers = r->get_log_headers();
        auto log_values = r->get_log_values();
        for (size_t i = 0; i < log_headers.size(); i++) {
            log_str += log_headers[i] + ": ";
            log_str += to_string(log_values[i]) + "\t";
        }
        for (auto t : cur_thrusts) {
            thrusts[static_cast<int>(t.axis)] = t.value;
        }
    }


    result.tx = bound(thrusts[static_cast<int>(Axis::TX)], tx_limit);
    result.ty = bound(thrusts[static_cast<int>(Axis::TY)], ty_limit);
    result.tz = bound(thrusts[static_cast<int>(Axis::TZ)], tz_limit);
    result.mx = bound(thrusts[static_cast<int>(Axis::MX)], mx_limit);
    result.my = bound(thrusts[static_cast<int>(Axis::MY)], my_limit);
    result.mz = bound(thrusts[static_cast<int>(Axis::MZ)], mz_limit);

    LOG << "tx: " << result.tx << "\t"
        << "ty: " << result.ty << "\t"
        << "tz: " << result.tz << "\t"
        << "mx: " << result.mx << "\t"
        << "my: " << result.my << "\t"
        << "mz: " << result.mz << "\t"
        << log_str << endl;
    regul_pub_.publish(convert(result));
}

void MotionServer::publish_cmd_status(int id, CmdStatus status)
{
    motion::MsgCmdStatus msg;
    msg.id = id;
    msg.status = static_cast<char>(status);

    string status_str;
    switch(status) {
        case CmdStatus::SUCCESS:
            status_str = "SUCCESS";
        break;
        case CmdStatus::PROCESSING:
            status_str = "PROCESSING";
        break;
        case CmdStatus::TIMEOUT:
            status_str = "TIMEOUT";
        break;
        case CmdStatus::STOPPED:
            status_str = "STOPPED";
        break;
    }
    LOG << "command #" << id << ": " << status_str << endl;

    cmd_status_pub_.publish(msg);
}

void MotionServer::process_navig(const NavigInfo& msg)
{
    update_thrusts(msg);
}

double MotionServer::bound(double num, double limit)
{
    double abs_limit = fabs(limit);
    if (num < -abs_limit) {
        num = -abs_limit;
    }
    if (num > abs_limit) {
        num = abs_limit;
    }
    return num;
}

motion::MsgRegul MotionServer::convert(const motion::MsgRegul& msg) const
{
    motion::MsgRegul result;
    result.tx = msg.ty;
    result.ty = msg.tx;
    result.tz = -msg.tz;
    result.mx = -msg.my;
    result.my = msg.mx;
    result.mz = msg.mz;

    return result;
}
