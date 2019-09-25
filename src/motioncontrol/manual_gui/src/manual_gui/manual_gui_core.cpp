#include "../../include/manual_gui_core.hpp"

namespace manual_gui {

manual_gui_demo::manual_gui_demo(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
        : reconfigSrv_{private_node_handle}, params_{private_node_handle} {

    /**
     * Initialization
     */
    params_.fromParamServer();

    /**
     * Set up timer and dynamic reconfigure server
     */
    timer_ = private_node_handle.createTimer(ros::Duration(1. / params_.rate), &manual_gui_demo::timerCallback, this);
    reconfigSrv_.setCallback(boost::bind(&manual_gui_demo::reconfigureRequest, this, _1, _2));

    params_pub = n.advertise<control_msgs::ModeCmd>("modecmd",10);
}

manual_gui_demo::~manual_gui_demo(){
    params_.auto_mode = 0;
    params_.speed_mode = 0;
    params_.steer_mode = 0;
    params_.toParamServer();
}
/*
 * Use const ConstPtr for your callbacks.
 * The 'const' assures that you can not edit incoming messages.
 * The Ptr type guarantees zero copy transportation within nodelets.
 */
void manual_gui_demo::timerCallback(const ros::TimerEvent& event) {
//    ROS_INFO_STREAM("Timer callback. auto_mode = " << params_.auto_mode);
}

/**
  * This callback is called whenever a change was made in the dynamic_reconfigure window
*/
void manual_gui_demo::reconfigureRequest(DemoConfig& config, uint32_t level) {
    params_.fromConfig(config);
    timer_.setPeriod(ros::Duration(1. / params_.rate));
    ROS_WARN_STREAM("Parameter update:\n" << params_);
}
DemoParameters manual_gui_demo::GetParams(){
    return params_;
}

void manual_gui_demo::run() {
    DemoParameters node_params = GetParams();

    ros::Rate loop_rate(node_params.rate);
    while(ros::ok()){
        ros::spinOnce();
        control_msgs::ModeCmd modecmd_;
        modecmd_.auto_mode = (int)params_.auto_mode;
        modecmd_.speed_mode = (int)params_.speed_mode;
        modecmd_.throttle_enable = params_.throttle_enable;
        modecmd_.brake_enable = params_.brake_enable;
        modecmd_.steer_mode = params_.steer_mode;
        modecmd_.gear_mode = params_.gear_mode;

        params_pub.publish(modecmd_);
        ROS_INFO("auto_mode:%d, speed_mode:%d, throttle_enable:%d, brake_enable:%d, steering_mode:%d, gear_mode:%d",
                 modecmd_.auto_mode, modecmd_.speed_mode, modecmd_.throttle_enable, modecmd_.brake_enable,
                 modecmd_.steer_mode, modecmd_.gear_mode);
        loop_rate.sleep();
    }
}
} // namespace rosparam_handler_tutorial
