#include "tcu.h"

#define N 5 // количество движков
#define DOF 5 // количество степеней свободы

class TcuRobosub : public Tcu
{
public:
    TcuRobosub(ipc::Communicator& com);
    ~TcuRobosub();

    struct FaulhaberThruster : public Thruster
    {
    	int can_id;
    };

    void send_thrusts();
    void routine();
    void increase_loop_params();

private:
    void read_config(const int num_of_thrusters) override;
    void init_ipc() override;
    bool need_send_settings();
    void send_settings_individual(const int num);
    void send_settings();
    void reset_settings_it();
    void normalize_config_values();
    void normalize_channel(const LocationType type);
    void calc_new_signals() override;
    void calc_new_thrusts(const tcu::CmdForce& msg) override;
    void calc_thrusters_distribution();

    static const int settings_iterations;

    int common_can_addr_;
    ros::Publisher can_send_pub_;

    std::vector<double> thrusts_;
    std::vector<int> codes_;
    std::map <double, int> thrusts_to_codes_;

    std::array<double, DOF> b;
    std::array<std::array<double, DOF>, DOF> A;
    std::array<std::array<double, DOF>, DOF> A_inverse;

    int last_settings_it_  = 0;    

    int settings_id_ = 0;
};