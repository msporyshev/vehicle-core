#include "position_estimator.h"

#include <dynamic_reconfigure/server.h>

int main(int argc, char* argv[])
{
    std::cout << argv[0] << std::endl;

    auto communicator = ipc::init(argc, argv, PositionEstimator::NODE_NAME);

    PositionEstimator estimator(PositionEstimator::Device::IMU);

    estimator.init_ipc(communicator);

    dynamic_reconfigure::Server<navig::PositionEstimatorConfig> server;
    dynamic_reconfigure::Server<navig::PositionEstimatorConfig>::CallbackType node_callback;

    node_callback = boost::bind(&PositionEstimator::read_config, &estimator, _1, _2);
    server.setCallback(node_callback);

    ipc::EventLoop loop(estimator.get_period());
    while (loop.ok()) {
        if (estimator.current_device_ready()) {
            estimator.read_current_device_msg();
        }
    }
    return 0;
}