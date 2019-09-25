//
// Created by aidous on 18-12-3.
//

#ifndef UDP_SEND_UDP_HPP
#define UDP_SEND_UDP_HPP

#endif //UDP_SEND_UDP_HPP

#include <ros/ros.h>
#include <iostream>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <cstring>
#include <arpa/inet.h>
#include <queue>
#include <chrono>
#include <sensor_msgs/Joy.h>
#include"std_msgs/Float32.h"
#include "control_msgs/GetECUReport.h"
#include "control_msgs/SteerCmd.h"
#include "speed_ctrl_msgs/speed_ctrl.h"
#include "sensor_driver_msgs/GpswithHeading.h"
#include "sensor_driver_msgs/OdometrywithGps.h"

using namespace std;

namespace carnet {
struct CarState {
    double expvel;
    double tempvel;
    double expsteering;
    double tempsteering;
    double pos_x;
    double pos_y;
    double lat;
    double lon;
    double theta;
    double heading;
};

union _intbyte
{
    //INT16 _int;
    short _int;
    char	_char[2];
};

typedef union {
    float fdata;
    unsigned long ldata;
} FloatLongType;

union data {
    unsigned long a;
    unsigned char tab[4];
} IntToChar;

class udp_car {
public:
    udp_car(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private,
            std::string node_name);

    // ros
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Timer loop_timer_;
    std::string node_name_{"udp_sender_node"};

    ros::Subscriber joy_sub_;

    std_msgs::Float32 cmd_throttle;
    std_msgs::Float32 cmd_brake;
    std_msgs::Float32 cmd_steer;

    bool ecudataupdate;
    bool cmddataupdate;

    bool longitutdectrl_enabled;
    bool lateralctrl_enabled;
    int auto_enable, auto_enable_cancel, auto_enable_position, auto_enable_cancel_position, steer_position, throttle_position, brake_position;
    double steer_gain, brake_gain, throttle_gain;

    //// UDP Settings
    /* local address and port */
    int CarNetSocket;
    sockaddr_in addr_local;
    int local_port;

    /* remote address and port */
    sockaddr_in addr_remote, addr_vel_display;
    socklen_t addr_remote_len; //// do not forget to init the 'addr_remote_len'
    std::string remote_ip, vel_display_ip;
    int remote_port, vel_display_port;

    CarState carstate;

    float tank_dir_value, tank_str_value, tank_vel_value;

    void initialize();
    void joyCallback(const sensor_msgs::Joy joy);

    void timerCb();

    bool initSocket(void);
    void sendmsgs();
    void UpdateMsg();
};
}//namespace carnet
