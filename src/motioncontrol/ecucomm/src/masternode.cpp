/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :
* Copyright    :
* Descriptoin  : 读取配置文件,配置其他节点参数
* References   :
======================================================================*/
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/console.h>
#include <glog/logging.h>

#include "util/xmlconf/xmlconf.h"
#include <util/common.h>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <sensor_driver_msgs/startconfig.h>

std::string startconfigstr;
bool configsrv(sensor_driver_msgs::startconfig::Request  &req,
    sensor_driver_msgs::startconfig::Response &res)
{
  res.configstr = startconfigstr;
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ecuchannel");
  ros::NodeHandle nh;

  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_colorlogtostderr = true; //设置输出到屏幕的日志显示相应颜色
  std::string config_file;
  ros::param::get("~config_file",config_file);

  ros::Publisher pubStartConfig = nh.advertise<std_msgs::String>("startconfig", 2);

  XmlConf xml_conf;
  if (!xml_conf.LoadConf(config_file.c_str()))
  {
    std::cerr<<"no config file!"<<std::endl;
    return 0;
  }
  std::string record_path;
  xml_conf.GetSystemParam("record_path",record_path);
  bool record_on = false ;
  xml_conf.GetSystemParam("record_on",record_on);


  xml_conf.SetSystemParam("system_start_time",ros::Time::now().toSec());
  if(record_on)
  {
    record_path = createstampeddir(record_path);
    xml_conf.SetSystemParam("record_path",record_path.c_str());
  }
  bool log_on = false;
  xml_conf.GetSystemParam("log_on",log_on);
  if(log_on)
  {
    std::string logdatapath = "~/catkin_ws/logdata";
    xml_conf.GetSystemParam("log_path",logdatapath);
    logdatapath = createstampeddir(logdatapath);
    xml_conf.SetSystemParam("log_path",logdatapath.c_str());
  }
  xml_conf.String(startconfigstr);
  std::string xmlsavefile = record_path +"/config.xml";
  xml_conf.SaveConf(xmlsavefile.c_str());
  //  std_msgs::String xmlmsg;
  //  xmlmsg.data = startconfigstr;

  //  bool status = ros::ok();
  //  ros::Rate rate(1);
  //  while(status)
  //    {
  //      pubStartConfig.publish(xmlmsg);
  //      rate.sleep();
  //      status = ros::ok();
  //    }
  ros::ServiceServer service = nh.advertiseService("startconfigsrv", configsrv);
  ros::spin();
  return 1;
}
