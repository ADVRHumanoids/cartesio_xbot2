#include "cartesio_rt.h"

using namespace XBot;
using namespace XBot::Cartesian;

bool CartesioRt::on_initialize()
{
    setJournalLevel(Journal::Level::Low);

    jinfo("loading..");

    _nh = std::make_unique<ros::NodeHandle>();

    /* Get ik problem from ros param */
    YAML::Node ik_yaml;
    std::string ik_ros_param;
    if(getParam("~problem_description/content", ik_yaml))
    {   
        jinfo("found ik file via config");
    }
    else if(getParam("~problem_param", ik_ros_param))
    {
        std::string ik_str;

        if(!_nh->getParam(ik_ros_param, ik_str))
        {
            jerror("ros param '{}' not found", ik_ros_param);
            return false;
        }

        jinfo("found ik file via ros param");

        ik_yaml = YAML::Load(ik_str);
    }


    /* Create model and ci for rt loop */
    _rt_model = ModelInterface::getModel(_robot->getConfigOptions());

    auto rt_ctx = std::make_shared<Cartesian::Context>(
        std::make_shared<Parameters>(getPeriodSec()),
        _rt_model);

    ProblemDescription ik_problem(ik_yaml, rt_ctx);

    auto impl_name = getParamOr<std::string>("~solver", "OpenSot");

    _rt_ci = CartesianInterfaceImpl::MakeInstance(impl_name, ik_problem, rt_ctx);
    _rt_ci->enableOtg(rt_ctx->params()->getControlPeriod());
    _rt_ci->update(0, 0);

    /* Create model and ci for nrt loop */
    auto nrt_model = ModelInterface::getModel(_robot->getConfigOptions());

    auto nrt_ctx = std::make_shared<Cartesian::Context>(
        std::make_shared<Parameters>(*_rt_ci->getContext()->params()),
        nrt_model);

    _nrt_ci = std::make_shared<LockfreeBufferImpl>(_rt_ci.get(), nrt_ctx);
    _nrt_ci->pushState(_rt_ci.get(), _rt_model.get());
    _nrt_ci->updateState();
    auto nrt_ci = _nrt_ci;

    /*  Create ros api server */
    RosServerClass::Options opt;
    opt.tf_prefix = getParamOr<std::string>("~tf_prefix", "ci");
    opt.ros_namespace = getParamOr<std::string>("~ros_ns", "cartesian");

    /* Initialization */
    _rt_active = false;
    auto rt_active_ptr = &_rt_active;

    _nrt_exit = false;
    auto nrt_exit_ptr = &_nrt_exit;

    _ros_active = false;
    auto _ros_active_ptr = &_ros_active;

    _qdot = _q.setZero(_rt_model->getJointNum());
    _robot->getPositionReference(_qmap);


    /* Spawn thread */
    _nrt_th = std::make_unique<thread>(
        [rt_active_ptr, nrt_exit_ptr, _ros_active_ptr, nrt_ci, opt]()
        {
            this_thread::set_name("cartesio_nrt");
            std::shared_ptr<RosServerClass> ros_srv;

            while(!*nrt_exit_ptr)
            {
                if ((*rt_active_ptr) && (!*_ros_active_ptr)) 
                {
                    // plugin start detected
                    ros_srv = std::make_shared<RosServerClass>(nrt_ci, opt);
                    *_ros_active_ptr = true;
                }

                this_thread::sleep_for(10ms);

                if (*_ros_active_ptr)
                {
                    if (*rt_active_ptr) 
                    {
                        // normal operation
                        nrt_ci->updateState();
                        ros_srv->run();
                    }

                    else
                    {
                        // plugin stop detected
                        ros_srv.reset();
                        *_ros_active_ptr = false;
                    }
                }
            }

        });

    /* Set robot control mode */
    std::vector<std::string> enabled_chains;
    if(!getParam("~enabled_chains", enabled_chains))
    {
        jwarn("parameter ~enabled_chains is empty: enabling all chains)");
        enabled_chains = _robot->getChainNames();
    }

    for(auto ch : enabled_chains)
    {
        for(auto jn : _robot->chain(ch).getJointNames())
        {
            jinfo("enabling joint {}", jn);
            _ctrl_map[jn] = ControlMode::PosImpedance() + ControlMode::Effort();
        }
    }

#if XBOT2_VERSION_MINOR < 10
    _robot->setControlMode(ControlMode::Idle());
    _robot->setControlMode(_ctrl_map);
#else 
    setDefaultControlMode(_ctrl_map);
#endif

    // logger
    _profiling_logger = XBot::MatLogger2::MakeLogger("profiling_ik");
    _profiling_logger->create("solve_time", 1);
    _profiling_logger->create("overhead_time", 1);

    return true;
}

void CartesioRt::starting()
{
    // we use a fake time, and integrate it by the expected dt
    _fake_time = 0;

    // align model to current position reference
    _robot->sense(false);
    _robot->getPositionReference(_qmap);
    _rt_model->setJointPosition(_qmap);
    _rt_model->update();

    // reset ci
    _rt_ci->reset(_fake_time);

    // signal nrt thread that rt is active
    _rt_active = true;

    // transit to run
    start_completed();

}

void CartesioRt::run()
{
    /* Receive commands from nrt */
    _nrt_ci->callAvailable(_rt_ci.get());

    auto tic = std::chrono::high_resolution_clock::now();
    /* Solve IK */
    if(!_rt_ci->update(_fake_time, getPeriodSec()))
    {
        jerror("unable to solve \n");
        return;
    }
    auto toc = std::chrono::high_resolution_clock::now();
    _solve_time = std::chrono::duration_cast<std::chrono::nanoseconds>(toc-tic).count()*1e-9;
    _profiling_logger->add("solve_time", _solve_time);


    tic = std::chrono::high_resolution_clock::now();

    /* Integrate solution */
    _rt_model->getJointPosition(_q);
    _rt_model->getJointVelocity(_qdot);
    _q += getPeriodSec() * _qdot;
    _rt_model->setJointPosition(_q);
    _rt_model->update();

    _fake_time += getPeriodSec();

    /* Send state to nrt */
    _nrt_ci->pushState(_rt_ci.get(), _rt_model.get());

    /* Move robot */
    _robot->setReferenceFrom(*_rt_model);

    _robot->move();

    toc = std::chrono::high_resolution_clock::now();
    double overhead_time = std::chrono::duration_cast<std::chrono::nanoseconds>(toc-tic).count()*1e-9;
    _profiling_logger->add("overhead_time", overhead_time);
}

void CartesioRt::stopping()
{
    _rt_active = false;
    
    if (!_ros_active)
    {   
        stop_completed();
    }
    stop_completed();
}

void CartesioRt::on_abort()
{
    _rt_active = false;
}

void CartesioRt::on_close()
{
    _nrt_exit = true;
    jinfo("joining with nrt thread..");
    if(_nrt_th) _nrt_th->join();

    _profiling_logger->flush_available_data();
}

XBOT2_REGISTER_PLUGIN(CartesioRt,
                      cartesio_rt);