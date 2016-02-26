#pragma once

#include "task.h"
#include "task_factory.h"

#include <dsp/MsgBeacon.h>
#include <video/MsgFoundCircle.h>

#include <supervisor/CmdDeviceKey.h>
#include <supervisor/supervisor_devices.h>

enum class State
{
    Initialization,
    ListenToFirstPing,
    GoToPinger,
    BucketFindingInit,
    FindBucket,
    ActiveSearching,
    StabilizeBucket,
    DropBall,
    Finalize,
    Terminal
};

enum class Zone
{
    Far,
    Middle,
    Near,
    Close
};

const std::map<Zone, std::string> zone_typename = {
    {Zone::Far,     "Far"},
    {Zone::Middle,  "Middle"},
    {Zone::Near,    "Near"},
    {Zone::Close,   "Close"},
};

struct ZoneInfo
{
    Zone zone;
    double min;
    double max;
    int pings_in_row = 0;

    ZoneInfo() {}
    ZoneInfo(Zone zone, double min, double max)
            : zone(zone)
            , min(min)
            , max(max)
    {}
};

class DrumTask: public Task<State>
{
public:
    DrumTask(const YamlReader& cfg, ipc::Communicator& com);

    State handle_initialization();
    State handle_listen_first_ping();
    State handle_go_pinger();
    State handle_bucket_finding_init();
    State handle_active_searching();
    State handle_find_bucket();
    State handle_stabilize_bucket();
    State handle_drop_ball();
    State handle_finalize();

    void config_vehicle_thrust(Zone zone);

    void handle_ping(const dsp::MsgBeacon& msg);
    void handle_circle_found(const video::MsgFoundCircle& msg);

private:
    AUTOPARAM(double, timeout_initialization_);
    AUTOPARAM(double, timeout_listen_firt_ping_);
    AUTOPARAM(double, timeout_go_pinger_);
    AUTOPARAM(double, timeout_bucket_finding_init_);
    AUTOPARAM(double, timeout_find_bucket_);
    AUTOPARAM(double, timeout_active_searching_);
    AUTOPARAM(double, timeout_stabilize_bucket_);
    AUTOPARAM(double, timeout_drop_ball_);
    AUTOPARAM(double, timeout_finalize_);
    AUTOPARAM(double, timeout_regul_);

    AUTOPARAM(double, start_depth_);
    AUTOPARAM(double, end_depth_);
    AUTOPARAM(double, heading_delta_);
    AUTOPARAM(double, heading_delta_timeout_);

    AUTOPARAM(double, thrust_far_);
    AUTOPARAM(double, thrust_middle_);
    AUTOPARAM(double, thrust_near_); 
    
    AUTOPARAM(double, infinity_);
    AUTOPARAM(double, far_border_);
    AUTOPARAM(double, middle_border_);
    AUTOPARAM(double, close_border_);
    
    AUTOPARAM(double, filtered_heading_size_);

    AUTOPARAM(int, pings_needed_);

    AUTOPARAM(double, thrust_finalize_);
    
    AUTOPARAM(double, active_searching_thrust_);
    AUTOPARAM(double, active_searching_step_timeout_);
    
    AUTOPARAM(double, stab_coef_p_);
    AUTOPARAM(double, stab_thrust_limit_);

    AUTOPARAM(double, stabilize_count_needed_);
    AUTOPARAM(double, stabilization_eps_);
    
    AUTOPARAM(int, finding_count_needed_);
    AUTOPARAM(double, drum_real_radius_);

    AUTOPARAM(double, drum_distance_max_);
    AUTOPARAM(double, drum_distance_min_);
    AUTOPARAM(double, drum_depth_factor_);
    AUTOPARAM(double, drop_ball_delta_depth_default_);

    Zone cur_zone_;
    std::vector<ZoneInfo> zones_;
    
    dsp::MsgBeacon pinger_state_;
    video::MsgCircle drum_state_;

    ipc::Subscriber<dsp::MsgBeacon> subscribe_ping_;
    ipc::Subscriber<video::MsgFoundCircle> subscribe_circle_;
    
    ros::Publisher key_send_pub_;

    bool ping_found_;
    bool drum_found_;
    bool recognizer_init_;

    int finding_count_;
    int searching_sub_state_;
    int stabilize_count_;
    ros::Time searching_timer_start_;

    double drum_distance_;
    libauv::Point2d drum_center_;

    std::vector<double> filtered_heading_;
    std::vector<double> filtered_distance_;

    void init_ipc(ipc::Communicator& com);
    void init_zones();
    bool stabilize();
    libauv::Point2d get_new_thrust(libauv::Point2d drum_center);
    double filter_pinger_heading(double heading);
    Zone update_zone(const dsp::MsgBeacon& msg);
    double median_filter(std::vector<double> data);
};