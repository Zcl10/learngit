/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :
* Copyright    :
* Descriptoin  : 获取ECU数据并发送话题
* References   :
======================================================================*/
#include <sstream>
#include <fstream>
#include <iostream>
#include <iomanip>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <glog/logging.h>

#include "util/playback/iv_data_playback.h"
#include "AnalysisECU.h"
#include "sensor_driver_msgs/startconfig.h"
#include "sensor_driver_msgs/ECUData.h"
#include "control_msgs/GetECUReport.h"

using std::cout; using std::endl;


class ecuModule
{
public:
  ecuModule(std::string configstr):configstr_(configstr)
  {
    pnh_ = ros::NodeHandle("~");
    init_xml();
    initParameters();
  }

  void init_xml()
  {
    bool ListenSucess;
    ListenSucess = m_AnalysisECU_.Init(8500);               //初始化CAN0接收端口
    if(!ListenSucess)
      ROS_ERROR("Creat Listen to ECU  failure!!!");
    else
      ROS_INFO("Build Listen to ECU  successfully :)");
  }

  void initParameters()
  {
    auto &pnh = this->pnh_;
    this->is_human_brake_threshold_ = pnh.param("is_human_brake_threshold", 0.65);
    this->ecudata_ROS_INFO_enabled_ = pnh.param("ecudata_ROS_INFO_enabled", true);
    m_AnalysisECU_.is_human_brake_threshold_ = is_human_brake_threshold_;
  }

  bool process()
  {
    static long long counter = 0;
    if(counter%10 == 0)
      ROS_INFO("Receive ECU data...");
    bool res_status = m_AnalysisECU_.Update();
    bool res = true;
    ++counter;
    return res;
  }

  bool LoadConfigFile(const char* configstr)
  {
    if(!config.Parse(configstr, "GetECUData"))
    {
      std::cout<<"GetECUData  is not exist in config xml  file"<<std::endl;
      return false;
    }
    play_back_.Setup(config);
    ConfigParam();
    return true;
  }

  void ConfigParam()
  {
    if(!config.GetModuleParam("port",port))
    {
      std::cout<<"port num is incorrect"<<std::endl;
    }
    if(!config.GetSystemParam("GetECUData_on",Module_On))
    {
      std::cout<<"GetECUData_on  is  not exist"<<std::endl;
    }
    else
    {
      if(Module_On)
        std::cout<<"GetECUData_on  On"<<std::endl;
      else
        std::cout<<"GetECUData_on Off"<<std::endl;
    }

    if(!config.Parse(configstr_.c_str(), "VehicleParam"))
    {
      std::cout<<"VehicleParam  is not exist in config xml  file"<<std::endl;
    }
    if(!config.GetModuleParam("steeringratio_l",steeringratio_l_))
    {
      std::cout<<"steeringratio_l num is incorrect"<<std::endl;
    }
    else
      std::cout<<"steeringratio_l is "<<steeringratio_l_<<std::endl;

    if(!config.GetModuleParam("steeringratio_r",steeringratio_r_))
    {
      std::cout<<"steeringratio_r num is incorrect"<<std::endl;
    }
    else
      std::cout<<"steeringratio_r is "<<steeringratio_r_<<std::endl;
  }

  const struct_ECU& getEcuData() const
  {
//    cout<<"m_AnalysisECU_.ECUData_struct steering_angle========"<<m_AnalysisECU_.ECUData_struct.steering_angle<<endl;
    return m_AnalysisECU_.ECUData_struct;
  }

private:
  // yaml files config
  ros::NodeHandle pnh_;
  double is_human_brake_threshold_;
  bool ecudata_ROS_INFO_enabled_;

  bool Module_On;
  XmlConf config;
  IvDataPlayback play_back_;
  int port;
  std::string configstr_;

  AnalysisECU m_AnalysisECU_;
  double steeringratio_l_;
  double steeringratio_r_;
  double petral_pressure;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "getecudata");
  ros::NodeHandle nh;

  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  ros::Publisher ECUDataReport_Pub;
  ECUDataReport_Pub = nh.advertise<control_msgs::GetECUReport>("ecudatareport",10);
  ros::ServiceClient configclient = nh.serviceClient<sensor_driver_msgs::startconfig>("startconfigsrv");
  sensor_driver_msgs::startconfig configsrv;

  while(!configclient.call(configsrv))
  {
    ros::Duration(0.01).sleep();
  }
  std::string startconfig = configsrv.response.configstr;
  ecuModule ecumodule(startconfig);

  ros::Rate rate(50);
  while(ros::ok())
  {
    bool succeed = ecumodule.process();
    if(!succeed)
    {
//      cout<<"[ERROR] Cannot get ecu data!!!"<<endl;
      ros::Duration(1).sleep();
      continue;
    }
    const struct_ECU& ecu_data = ecumodule.getEcuData();
    control_msgs::GetECUReport ecuReport;
    ecuReport.header.stamp=ros::Time::now();
    ecuReport.steer_cur.steer = (float)ecu_data.fFLRWheelAverAngle;
    ecuReport.brake_cur.brake_pedal = (float)ecu_data.pressure_petral_feedback;
    ecuReport.brake_cur.brake_ESC_left = (float)ecu_data.pressure_left;
    ecuReport.brake_cur.brake_ESC_right = (float)ecu_data.pressure_right;
    ecuReport.throttle_cur.throttle_pedal = (float)ecu_data.accelerator_petral_feedback;
    ecuReport.throttle_cur.throttle = (float)ecu_data.throttle_opening;

    ecuReport.speed.speed_wheel.front_left = -(float)ecu_data.FrontLeftWheelSpeed*M_PI*0.455/60;  ///将底层反馈的电机转速(rpm) —> 车速(m/s)；0.455为车轮直径(单位：m)；下同
    ecuReport.speed.speed_wheel.front_right = (float)ecu_data.FrontRightWheelSpeed*M_PI*0.455/60;
    ecuReport.speed.speed_wheel.rear_left = -(float)ecu_data.BackLeftWheelSpeed*M_PI*0.455/60;
    ecuReport.speed.speed_wheel.rear_right = (float)ecu_data.BackRightWheelSpeed*M_PI*0.455/60;
    ecuReport.speed.velocity.linear.x = ecu_data.fForwardVel;
    ecuReport.manual.is_human_brake = ecu_data.brake_pedal;

    ecuReport.shift_cur.gear = ecu_data.f_shift;
    ecuReport.shift1_cur.gear = ecu_data.f_shift1;
    ecuReport.throttle_cur.engine_status.engine_rpm = ecu_data.EnginRate;
    ecuReport.throttle_cur.engine_status.engine_load = (float)ecu_data.Engine_load;

    ecuReport.mode.speed_mode = (unsigned char)ecu_data.longitutdectrl_enabled;
    ecuReport.mode.steer_mode = (unsigned char)ecu_data.lateralctrl_enabled;
    ecuReport.mode.brake_enable = (unsigned char)false;
    ecuReport.mode.throttle_enable = (unsigned char)false;

    //发送话题
    ECUDataReport_Pub.publish(ecuReport);
    rate.sleep();
  }
}