#include "tcu.h"

#define N 2 // количество движков
#define DOF 2 // количество степеней свободы

class TcuSurface : public Tcu
{
public:

    TcuSurface(ipc::Communicator& com);
    ~TcuSurface();

    struct TorqeedoThruster : public Thruster
    {
    	std::string tty_port_name;
    	int tty_port_handler;
    	int tty_port_baudrate;
    	bool inside_of_message;
		std::vector<unsigned char> message;
    };

    void routine();
	void send_thrusts();

private:
    static const int buf_len_;
    unsigned char *buffer_;
    int dead_point_;
    int max_signal_;

	void calc_new_signals();
	void calc_new_thrusts(const tcu::CmdForce& msg);
	unsigned char crc8(const std::vector<unsigned char> &v);
	void decorate(std::vector<unsigned char> &v);
    std::string init_connections();
    void process_message(std::shared_ptr<TorqeedoThruster> &t);
    void process_new_buffer(const unsigned char *buf, int len, std::shared_ptr<TorqeedoThruster> &t);
    void read_config(const int num_of_thrusters) override;


};