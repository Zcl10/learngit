//
// Created by wangwei on 18-3-16.
//

#ifndef GETECUDATA_NODE_CANALYSISECU_H
#define GETECUDATA_NODE_CANALYSISECU_H

#include <dlfcn.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <string.h>
#include <netinet/in.h>
#include <math.h>
#include <fstream>
#include <unistd.h>
#include  <iostream>
#include "util/ToXml.hh"

#define BUFSIZE 650

struct struct_ECU			// 车辆状态
{
  //20180822 zhanghm add
  bool auto_brake_enabled;//自动驾驶状态制动使能
  bool auto_steering_enabled;//转向使能
  bool auto_ACC_enabled;//油门使能
  bool auto_vehicle_enabled; //整车使能

  float steering_angle;//方向盘转角
  /**********状态变量*********/
  double fForwardVel;					// 车辆纵向速度(Unit:m/s)
  double fDeForwardVel;
  double fFLRWheelAverAngle;			// 名义前轮偏角，对应电机或方向盘的角度(Unit:°)
  double fOdometer;
  double fRadius;
  double gear_line_displacement;

  int f_shift;				//档位 0无效1P2R3N4N5D6M7S8+9-
  unsigned char f_shift_Add;
  unsigned char f_shift1;				//具体档位
  unsigned char f_estop;				//紧急制动
  unsigned char f_leftlamp;			//左转向灯
  unsigned char f_rightlamp;			//右转向灯
  bool lateralctrl_enabled;
  bool longitutdectrl_enabled;
  bool brake_enabled;               //人工制动踏板是否被踩下
  double	lTimeStamp;			    // 时间戳(Unit:ms)

  char autodrive_status;
  double brake_pedal_signal;
  char switch_signal;


  double pressure_left;         //左路制动油压
  double pressure_right;        //右路制动油压
  double pressure_petral_feedback;      //制动踏板行程

  double accelerator_petral_feedback;   //油门踏板行程
  double throttle_opening;              //节气门开度
  char steerRx_err;
  char steerTx_err;
  char brakeRx_err;
  char brakeTx_err;
  char PC_Tx_err;

  char poweron_status;
  char start_status;
  char warning_status;
  char bugle_status;

  char light_far;
  char light_near;

  char Estop_enabled;

  int EnginRate;
  //qjy add 0918
  double FrontLeftWheelSpeed;
  double FrontRightWheelSpeed;
  double BackLeftWheelSpeed;
  double BackRightWheelSpeed;
  //*************************GHJ20171012增加车辆底层反馈信息*************************//
  unsigned char FrontLeftWheelSpeedStatus;
  unsigned char FrontRightWheelSpeedStatus;
  unsigned char BackLeftWheelSpeedStatus;
  unsigned char BackRightWheelSpeedStatus;

  double Yaw_Rate;
  double Yaw_Rate_Offset;
  unsigned char Yaw_Rate_Status;

  double AX;
  double AX_Offset;
  unsigned char AX_Status;

  double AY;
  double AY_Offset;
  unsigned char AY_Status;
  //*************************GHJ20171012增加车辆底层反馈信息*************************//
  //*************************霍钊20180103增加丰田底层反馈信息*************************//
  //    double T_lBrakePressure;
  //    double T_rBrakePressure;
  double T_throttlePedalPosition1;
  double T_throttlePedalPosition2;
  double T_lFBrakePressure;
  double T_rFBrakePressure;
  double T_lRBrakePressure;
  double T_rRBrakePressure;
  unsigned char T_escerrorCode;
  unsigned char T_bottomerrorCode;
  double Engine_load;
  //    unsigned char f_shift_num_OBD;      //具体档位


  /// BYD Ray
  unsigned char speed_status;
  unsigned char steer_status;
  unsigned char brake_pedal;
  unsigned char drive_status;
};

class AnalysisECU {
public:
  struct_ECU   ECUData_struct;
  AnalysisECU();
  ~AnalysisECU();
  bool Init(int port);
  bool Update();
  double steeringratio_l;//qjy,0829
  double steeringratio_r;
  double is_human_brake_threshold_;
private:
  char m_ECUDataFromReceiver[60];
  void ECU_DataProcFromVehicle(unsigned char* data);

  sockaddr_in myaddr; /* our address */
  sockaddr_in remaddr; /* remote address */
  socklen_t addrlen; /* length of addresses */

  int listen_port_;//监听端口
  int recvlen; /* # bytes received */
  int fd; /* our socket */
  unsigned char buf[BUFSIZE]; /* receive buffer */
  double petral_pressure;

  //霍钊添加
  int CANFound;

  //    std::fstream data_backup;
};


#endif //GETECUDATA_NODE_CANALYSISECU_H
