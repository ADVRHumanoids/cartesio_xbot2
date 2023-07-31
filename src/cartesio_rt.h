#ifndef CARTESIO_RT_H
#define CARTESIO_RT_H

#include <xbot2/xbot2.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/sdk/rt/LockfreeBufferImpl.h>
#include <cartesian_interface/ros/RosServerClass.h>

namespace XBot {

class CartesioRt : public ControlPlugin
{

public:

    using ControlPlugin::ControlPlugin;

    bool on_initialize() override;
    void starting() override;
    void run() override;
    void stopping() override;
    void on_abort() override;
    void on_close() override;

private:

    std::unique_ptr<ros::NodeHandle> _nh;

    JointIdMap _qmap;
    Eigen::VectorXd _q, _qdot;
    ModelInterface::Ptr _rt_model;
    Cartesian::CartesianInterfaceImpl::Ptr _rt_ci;
    Cartesian::LockfreeBufferImpl::Ptr _nrt_ci;
    std::atomic_bool _rt_active, _ros_active, _nrt_exit;
    double _fake_time, _solve_time;

    std::map<std::string, ControlMode> _ctrl_map;

    std::unique_ptr<thread> _nrt_th;

    XBot::MatLogger2::Ptr _profiling_logger;
};


}

#endif // CARTESIO_RT_H