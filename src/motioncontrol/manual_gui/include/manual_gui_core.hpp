#pragma once

#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include "manual_gui/DemoParameters.h"

#include "control_msgs/ModeCmd.h"

namespace manual_gui {

class manual_gui_demo {
public:
    manual_gui_demo(ros::NodeHandle, ros::NodeHandle);
    ~manual_gui_demo();
    DemoParameters GetParams();
    void run();
    ros::NodeHandle n;
    ros::Publisher params_pub;
private:
    DemoParameters params_;

    ros::Timer timer_;
    dynamic_reconfigure::Server<DemoConfig> reconfigSrv_; // Dynamic reconfiguration service

    void timerCallback(const ros::TimerEvent& event);
    void reconfigureRequest(DemoConfig&, uint32_t);
};

} // namespace rosparam_handler_tutorial
