//
// Created by wangwei on 18-3-16.
//
// Include Files
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/console.h>
#include <glog/logging.h>
#include <sstream>
#include <fstream>
#include <std_msgs/UInt8.h>

#include "util/playback/iv_data_playback.h"
#include "../include/ECUDataOutPut.h"
#include "sensor_driver_msgs/startconfig.h"
#include "sensor_driver_msgs/ECUData.h"
#include "control_msgs/SendECUCmd.h"
#include "control_msgs/GetECUReport.h"
#include "control_msgs/SpeedCmd.h"

//常量定义
const int port_CAN0 = 8400;//发送端口号
const char* Send_IP = "192.168.0.10";
class SendecuModule
{
public:
  SendecuModule(std::string configstr, ros::NodeHandle& nodehandleIn):configstr_(configstr),nh(nodehandleIn)
  {
    Init();
    pnh_ = ros::NodeHandle("~");
    initParameters();
    printParameters();
    is_speedCmd_update      = false;
    is_steeringCmd_update   = false;
    is_GearCmd_update       = false;
    is_lightsignal_update   = false;
    mode = 0;
    SteeringCmd_Sub     = nh.subscribe("steercmd",1, &SendecuModule::CallbackfromSteerCmd,this);
    SpeedCmd_Sub        = nh.subscribe("speedcmd",1, &SendecuModule::CallbackfromSpeedCmd,this);
    GearCmd_Sub         = nh.subscribe("/gearcmd",1, &SendecuModule::CallbackfromGearCmd,this);
    ModeCmd_Sub         = nh.subscribe("modecmd",1, &SendecuModule::CallbackfromModeCmd,this);
    GetECUData_Sub      = nh.subscribe("ecudatareport",1, &SendecuModule::CallbackfromECUReport,this);
    LightLanguage_Sub   = nh.subscribe("/light_type",1,&SendecuModule::CallbackfromLightReport,this);
  }

  void Init()
  {
    std::cout<<"Send ECU IP is "<<Send_IP<<std::endl;
    //初始化CAN网
    bool CommSucess=m_ECUDataOutPut_CAN0.Init(Send_IP,port_CAN0);
    if(!CommSucess){
      ROS_ERROR("SendECUData module Init failure");
    }
    else{
      ROS_INFO("SendECUData module Init  successfully :)");
    }
  }

  bool process()
  {
    ///CAN0下发量
    m_ECUDataOutPut_CAN0.autodriving = this->mode;
    m_ECUDataOutPut_CAN0.longitutdectrl_enabled = sendECUCmd_.mode.speed_mode && this->mode;
    m_ECUDataOutPut_CAN0.lateralctrl_enabled = sendECUCmd_.mode.steer_mode && this->mode;

    if(sendECUCmd_.brake_cmd.brake < 0)
        sendECUCmd_.brake_cmd.brake = 0;
    if(sendECUCmd_.throttle_cmd.throttle < 0)
        sendECUCmd_.throttle_cmd.throttle = 0;

//    m_ECUDataOutPut_CAN0.SendECUData_struct.throttle = -sendECUCmd_.throttle_cmd.throttle*10;   ///油门量扩大10倍且取负，用于匹配底层 -1000~1000的油门开度，负值为前进
//    m_ECUDataOutPut_CAN0.SendECUData_struct.brake = (int)sendECUCmd_.brake_cmd.brake*12;        ///制动量扩大12倍，用于匹配底层从0~1200mA的制动电流
//    m_ECUDataOutPut_CAN0.SendECUData_struct.shift = sendECUCmd_.gear_cmd.gear;
//    m_ECUDataOutPut_CAN0.SendECUData_struct.steering_ctrl = sendECUCmd_.steer_cmd.steer;
    m_ECUDataOutPut_CAN0.SendECUData_struct.throttle = 5*10;   ///油门量扩大10倍且取负，用于匹配底层 -1000~1000的油门开度，负值为前进
    m_ECUDataOutPut_CAN0.SendECUData_struct.brake = 5*12;        ///制动量扩大12倍，用于匹配底层从0~1200mA的制动电流
//    m_ECUDataOutPut_CAN0.SendECUData_struct.shift = sendECUCmd_.gear_cmd.gear;
    m_ECUDataOutPut_CAN0.SendECUData_struct.steering_ctrl = 0;

    if (m_ECUDataOutPut_CAN0.autodriving ==1){
        m_ECUDataOutPut_CAN0.SendDeVehicleState();
    } else
      std::cout<<"当前处于人工驾驶状态"<<std::endl;

    return true;
  }

  void CallbackfromSteerCmd(control_msgs::SteerCmd msg);
  void CallbackfromSpeedCmd(control_msgs::SpeedCmd msg);
  void CallbackfromECUReport(control_msgs::GetECUReport msg);
  void CallbackfromGearCmd(control_msgs::GearCmd msg);
  void CallbackfromModeCmd(control_msgs::ModeCmd msg);
  void CallbackfromLightReport(std_msgs::UInt8 msg);

  bool is_steeringCmd_update;
  bool is_speedCmd_update;
  bool is_GearCmd_update;
  bool is_lightsignal_update;
  int mode;
  int lightlanguage;

  control_msgs::SendECUCmd sendECUCmd_;
  ros::WallTime last_timestamp_for_steeringCmd_received;
  ros::WallTime last_timestamp_for_speedCmd_received;
  ros::WallTime last_timestamp_for_GearCmd_received;
  ros::WallTime last_timestamp_for_modeCmd_received;
  ros::WallTime last_timestamp_for_LightSignal_received;

  void resetFLag();

private:
  void initParameters();
  void printParameters() const;
  double mode_elapse_time_;
  double gear_elapse_time_;
  double steer_elapse_time_;
  double speed_elapse_time_;
  double light_elapse_time_;

  std::string configstr_;

  ECUDataOutPut m_ECUDataOutPut_CAN0;
  ECUDataOutPut m_ECUDataOutPut_CAN1;///另一路CAN
  ros::Subscriber SteeringCmd_Sub, SpeedCmd_Sub, GetECUData_Sub, GearCmd_Sub, ModeCmd_Sub, decdelay_Sub, LightLanguage_Sub;
  ros::NodeHandle& nh;
  ros::NodeHandle pnh_;
  control_msgs::GetECUReport getECUReport_;
};

void SendecuModule::initParameters(){
  auto &pnh = this->pnh_;
  this->mode_elapse_time_ = pnh.param("mode_elapse_time_ms", 200.0);
  this->gear_elapse_time_ = pnh.param("gear_elapse_time_ms", 200.0);
  this->steer_elapse_time_ = pnh.param("steer_elapse_time_ms", 200.0);
  this->speed_elapse_time_ = pnh.param("speed_elapse_time_ms", 200.0);
  this->light_elapse_time_ = pnh.param("light_elapse_time_ms", 200.0);
}
void SendecuModule::printParameters() const {
  ROS_INFO_STREAM("Using mode_elapse_time_ms = " << this->mode_elapse_time_);
  ROS_INFO_STREAM("Using gear_elapse_time_ms = " << this->gear_elapse_time_);
  ROS_INFO_STREAM("Using steer_elapse_time_ms = " << this->steer_elapse_time_);
  ROS_INFO_STREAM("Using speed_elapse_time_ms = " << this->speed_elapse_time_);
  ROS_INFO_STREAM("Using light_elapse_time_ms = " <<this->light_elapse_time_);
}

void SendecuModule::CallbackfromSteerCmd(control_msgs::SteerCmd msg){
  sendECUCmd_.steer_cmd.steer = msg.steer;
  is_steeringCmd_update = true;
  last_timestamp_for_steeringCmd_received = ros::WallTime::now();
  ROS_INFO("Get SteeringCmd: %f",sendECUCmd_.steer_cmd.steer);
}

void SendecuModule::CallbackfromSpeedCmd(control_msgs::SpeedCmd msg){
  sendECUCmd_.brake_cmd.brake = msg.brake_cmd.brake;
  sendECUCmd_.throttle_cmd.throttle = msg.throttle_cmd.throttle;
  is_speedCmd_update = true;
  last_timestamp_for_speedCmd_received = ros::WallTime::now();
  ROS_INFO("Get SpeedControlCmd: brake_value=%f, throttle_value=%f",sendECUCmd_.brake_cmd.brake, sendECUCmd_.throttle_cmd.throttle);
}

void SendecuModule::CallbackfromGearCmd(control_msgs::GearCmd msg){
  sendECUCmd_.gear_cmd.gear = msg.gear;
  ROS_INFO("sendECUCmd_.gear_cmd.gear: %d", sendECUCmd_.gear_cmd.gear);
  is_GearCmd_update = true;
  last_timestamp_for_GearCmd_received = ros::WallTime::now();
}

void SendecuModule::CallbackfromModeCmd(control_msgs::ModeCmd msg){
  mode = msg.auto_mode;
  sendECUCmd_.mode.steer_mode = msg.steer_mode;
  sendECUCmd_.mode.speed_mode = msg.speed_mode;
  //    sendECUCmd_.mode.brake_enable = msg.brake_enable;
  //    sendECUCmd_.mode.throttle_enable = msg.throttle_enable;
  ROS_INFO("Get ModeCmd: auto_mode = %d, steer_mode = %d, speed_mode = %d",mode, sendECUCmd_.mode.steer_mode, sendECUCmd_.mode.speed_mode);
  last_timestamp_for_modeCmd_received = ros::WallTime::now();
}

void SendecuModule::CallbackfromECUReport(control_msgs::GetECUReport msg){
  getECUReport_.manual.is_human_brake = msg.manual.is_human_brake;
  getECUReport_.brake_cur.brake_ESC_left = msg.brake_cur.brake_ESC_left;
  getECUReport_.brake_cur.brake_ESC_right = msg.brake_cur.brake_ESC_right;
}

void SendecuModule::CallbackfromLightReport(std_msgs::UInt8 msg){
    lightlanguage = msg.data;  //fill 0~8      define 9 modern:0-全灭;1-左亮;2-右亮;3-左右亮;4-左闪;5-右闪;6-左右闪;7-左闪右亮;8-左亮右闪
    is_lightsignal_update = true;
    last_timestamp_for_LightSignal_received = ros::WallTime::now();
}

void SendecuModule::resetFLag() {
  ros::WallTime end = ros::WallTime::now();
  double mode_elapse_time = (end - last_timestamp_for_modeCmd_received).toSec() * 1000;
  double gear_elapse_time = (end - last_timestamp_for_GearCmd_received).toSec() * 1000;
  double steer_elapse_time = (end - last_timestamp_for_steeringCmd_received).toSec() * 1000;
  double speed_elapse_time = (end - last_timestamp_for_speedCmd_received).toSec() * 1000;
  double light_elapse_time = (end - last_timestamp_for_LightSignal_received).toSec() * 1000;


  if (mode_elapse_time < mode_elapse_time_) {
    mode = true;
  } else {
    mode = false;
    ROS_ERROR("Receive ModeCmd delay exceed time!");
  }
  if (gear_elapse_time < gear_elapse_time_) {
    is_GearCmd_update = true;
  } else {
    is_GearCmd_update = false;
    ROS_ERROR("Receive GearCmd delay exceed time!");
  }
  if (steer_elapse_time < steer_elapse_time_) {
    is_steeringCmd_update = true;
  } else {
    is_steeringCmd_update = false;
    ROS_ERROR("Receive SteerCmd delay exceed time!");
  }
  if (speed_elapse_time < speed_elapse_time_) {
    is_speedCmd_update = true;
  } else {
    is_speedCmd_update = false;
    ROS_ERROR("Receive SpeedCmd delay exceed time!");
  }
  if (light_elapse_time < light_elapse_time_) {
    is_lightsignal_update = true;
  } else {
    is_lightsignal_update = false;
  }

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sendecudata");
  ros::NodeHandle nh;

  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  //  FLAGS_colorlogtostderr = true;

  ros::ServiceClient configclient = nh.serviceClient<sensor_driver_msgs::startconfig>("startconfigsrv");
  sensor_driver_msgs::startconfig configsrv;

  while(!configclient.call(configsrv))
  {
    ros::Duration(0.01).sleep();
  }

  std::string startconfig = configsrv.response.configstr;

  SendecuModule sendecu_module(startconfig, nh);

  ros::Rate loop_rate(50);
  ROS_INFO("SendECUData node starts...");
  while(ros::ok())
  {
    bool succeed = sendecu_module.process();

    if(!sendecu_module.is_lightsignal_update){
      sendecu_module.lightlanguage = 0;
    }

    if(!succeed) {
      LOG(ERROR)<<"sendecudata module process error";
      break;
    }
    loop_rate.sleep();
    ros::spinOnce();
  }
}
