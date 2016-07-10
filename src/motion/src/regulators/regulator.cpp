#include "regulators/regulator.h"
#include <log.h>
#include <utils/basic.h>

using namespace std;

Regulator::Regulator(int id, vector<Axis> axes, double timeout):
    id(id),
    timeout(timeout),
    state(State::PENDING),
    success(false),
    first_update(true)
{
    if (id != -1) {
        LOG << "starting cmd #" << id << ", timeout: " << timeout << endl;
    }
    vector<int> added(6, 0);
    for (auto axis : axes) {
        int idx = static_cast<int>(axis);
        if (!added[idx]) {
            added[idx] = true;
            this->axes.push_back(axis);
        }
    }
}

int Regulator::get_id() const
{
    return id;
}

const vector<Axis>& Regulator::get_axes() const
{
    return axes;
}

const vector<ThrustInfo>& Regulator::get_thrusts() const
{
    return thrusts;
}

void Regulator::set_thrusts(vector<ThrustInfo> new_thrusts)
{
    for (auto d : dependencies) {
        auto dep_thrusts = d->get_thrusts();
        new_thrusts.insert(new_thrusts.end(), dep_thrusts.begin(), dep_thrusts.end());
    }
    vector<ThrustInfo> res;
    for (auto axis : axes) {
        bool axis_found = false;
        for (size_t i = 0; i < new_thrusts.size() && !axis_found; ++i) {
            if (new_thrusts[i].axis == axis) {
                axis_found = true;
                res.push_back(new_thrusts[i]);
            }
        }
        if (!axis_found) {
            LOG << "axis " << static_cast<int>(axis) << " not found" << endl;
            res.push_back({axis, 0.0});
        }
    }
    thrusts = res;
}

void Regulator::set_success(bool status)
{
    bool dep_status = true;
    for (auto d : dependencies) {
        dep_status &= d->has_succeeded();
    }
    success = status && dep_status;
}

void Regulator::set_success(double err, double accuracy, double err_d, double accuracy_d)
{
    LOG << "setting success -- " << "err: " << err << ", accuracy: " << accuracy
        << ", err_d: " << err_d << ", accuracy_d: " << accuracy_d << endl;
    set_success((fabs(err) < fabs(accuracy)) && (fabs(err_d) < fabs(accuracy_d)));
}

void Regulator::set_success(double err, double accuracy)
{
    set_success(err, accuracy, 0.0, 1.0);
}

void Regulator::activate()
{
    start_time = timestamp();
    state = State::ACTIVE;
}

void Regulator::handle_navig(const NavigInfo& msg)
{
    if (first_update) {
        first_update = false;
        initialize(msg);
    }
    for (auto d : dependencies) {
        d->handle_navig(msg);
    }
    update(msg);
}

void Regulator::initialize(const NavigInfo& msg)
{

}

void Regulator::update(const NavigInfo& msg)
{
    set_thrusts({{Axis::TX, 0.0}, {Axis::TY, 0.0}, {Axis::TZ, 0.0},
        {Axis::MX, 0.0}, {Axis::MY, 0.0}, {Axis::MZ, 0.0}});
}


void Regulator::finish()
{
    state = State::FINISHED;
}

bool Regulator::is_pending() const
{
    return state == State::PENDING;
}

bool Regulator::is_active() const
{
    return state == State::ACTIVE;
}

bool Regulator::is_finished() const
{
    return state == State::FINISHED;
}

bool Regulator::has_succeeded() const
{
    return success;
}

bool Regulator::is_actual() const
{
    return timestamp() - start_time < timeout;
}

bool Regulator::has_conflicts(const Regulator& other) const
{
    for (auto axis : axes) {
        for (auto other_axis : other.axes) {
            if (axis == other_axis) {
                return true;
            }
        }
    }
    return false;
}

bool Regulator::has_conflicts(shared_ptr<Regulator> other) const
{
    if (other == nullptr) {
        LOG << "checking conflicts with bad regul (nullptr)" << endl;
    }
    return has_conflicts(*other);
}

vector<string> Regulator::get_log_headers() const
{
    return log_headers;
}

vector<double> Regulator::get_log_values() const
{
    return log_values;
}

void Regulator::set_log_headers(vector<string> headers)
{
    log_headers = headers;
    for (auto d : dependencies) {
        auto dep_headers = d->get_log_headers();
        log_headers.insert(log_headers.end(), dep_headers.begin(), dep_headers.end());
    }
}

void Regulator::set_log_values(vector<double> values)
{
    log_values = values;
    for (auto d : dependencies) {
        auto dep_values = d->get_log_values();
        log_values.insert(log_values.end(), dep_values.begin(), dep_values.end());
    }
}

void Regulator::add_dependency(shared_ptr<Regulator> dep)
{
    dependencies.push_back(dep);
}
