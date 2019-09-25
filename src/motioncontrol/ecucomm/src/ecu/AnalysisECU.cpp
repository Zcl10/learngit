//
// Created by wangwei on 18-3-16.
//

#include "../../include/AnalysisECU.h"
#include <dlfcn.h>
#include <arpa/inet.h>
using namespace std;
const int CAN0_Listen_Port = 8500;
//const int CAN1_Listen_Port = 8500;
const char* ECU_IP = "192.168.0.10";
AnalysisECU::AnalysisECU()
{
  //霍钊添加
  CANFound = -1;
}

AnalysisECU::~AnalysisECU()
{
  //	data_backup.close();
}

bool  AnalysisECU::Init(int port)
{
  cout<<"[INFO] AnalysisECU::Init, port number is: "<<port<<endl;
  this->listen_port_ = port;

  /* create a UDP socket */
  if((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
  {
    perror("cannot create ecu socket\n");
    return false;
  }

  /* bind the socket to any valid IP address and a specific port */
  memset((char*)&myaddr, 0, sizeof(myaddr));
  myaddr.sin_family = AF_INET;
  myaddr.sin_addr.s_addr =htonl(INADDR_ANY);
  myaddr.sin_port = htons(this->listen_port_);

//  enable address reuse
//  int ret,on=1;
//  ret = setsockopt(fd,SOL_SOCKET,SO_REUSEADDR,&on,sizeof(on));

  if(bind(fd, (sockaddr *)&myaddr, sizeof(myaddr)) < 0)
  {
    perror("ECU input  bind failed");
    return false;
  }

  memset(&remaddr,0,sizeof(remaddr));

  return true;
}

bool AnalysisECU::Update()
{
  addrlen = sizeof(remaddr); /* length of addresses */
  //没数据不等待
  recvlen = recvfrom(fd, buf, BUFSIZE, MSG_DONTWAIT, (struct sockaddr *)&remaddr, &addrlen);//20180115
  if(recvlen<0){
    cout<<"[ERROR] "<<this->listen_port_<<" cannot get ECU data!"<<endl;
  }
  else
  {
    if(this->listen_port_ == CAN0_Listen_Port)
      ECU_DataProcFromVehicle(buf);
//    else if(this->listen_port_ == CAN1_Listen_Port)
//      ECU_DataProcGear(buf);
    else{
      cout<<"[ERROR] Invalid listen port!!"<<endl;
    }
  }
  recvlen = recvfrom(fd, buf, BUFSIZE, MSG_DONTWAIT, (struct sockaddr *)&remaddr, &addrlen);//20180115
  if(recvlen<0){
    cout<<"[ERROR] "<<this->listen_port_<<" cannot get ECU data!"<<endl;
    
  }
  else
  {
    if(this->listen_port_ == CAN0_Listen_Port)
      ECU_DataProcFromVehicle(buf);
//    else if(this->listen_port_ == CAN1_Listen_Port)
//      ECU_DataProcGear(buf);
    else{
      cout<<"[ERROR] Invalid listen port!!"<<endl;
      return false;
    }
    
  }
  recvlen = recvfrom(fd, buf, BUFSIZE, MSG_DONTWAIT, (struct sockaddr *)&remaddr, &addrlen);//20180115
  if(recvlen<0){
    cout<<"[ERROR] "<<this->listen_port_<<" cannot get ECU data!"<<endl;
    
  }
  else
  {
    if(this->listen_port_ == CAN0_Listen_Port)
      ECU_DataProcFromVehicle(buf);
//    else if(this->listen_port_ == CAN1_Listen_Port)
//      ECU_DataProcGear(buf);
    else{
      cout<<"[ERROR] Invalid listen port!!"<<endl;
      return false;
    }
    
  }
  return true;
  
}

void AnalysisECU::ECU_DataProcFromVehicle(unsigned char* data)
{
  int can_frame_count = recvlen/13;//each can frame contains 13 bytes
  for(int i=0;i<can_frame_count;++i){//a udp data frame may contain numbers of CAN frames
    unsigned char* buf = &(data[i*13]);
    unsigned int tmpCanID = 0;
    unsigned char tmpdata[8] = {0};
    tmpCanID=buf[1]<<24|buf[2]<<16|buf[3]<<8|buf[4];
    tmpdata[0]=buf[5];tmpdata[1]=buf[6];tmpdata[2]=buf[7];
    tmpdata[3]=buf[8];tmpdata[4]=buf[9];tmpdata[5]=buf[10];
    tmpdata[6]=buf[11];tmpdata[7]=buf[12];

    if(tmpCanID == 0x217) {
      double MSB;
      MSB = (tmpdata[1] >> 7) & 0x01;
      if (MSB == 0) {
          ///左前轮速
          ECUData_struct.FrontLeftWheelSpeed = (tmpdata[1] * 256 + tmpdata[0]);
      }
      else{
//          ECUData_struct.FrontLeftWheelSpeed = ((tmpdata[1] * 256 + tmpdata[0])-0xffff);
          ECUData_struct.FrontLeftWheelSpeed = -((~(tmpdata[1] * 256 + tmpdata[0])+1)&0xefff); //负数以形式存储，此处计算出负数绝对值后加负号
      }
    }

    if(tmpCanID == 0x227) {
      double MSB;
      MSB = (tmpdata[1] >> 7) & 0x01;
      if (MSB == 0) {
          ///右前轮速
          ECUData_struct.FrontRightWheelSpeed = (tmpdata[1] * 256 + tmpdata[0]);
      }
      else{
//          ECUData_struct.FrontRightWheelSpeed = ((tmpdata[1] * 256 + tmpdata[0])-0xffff);
          ECUData_struct.FrontRightWheelSpeed = -((~(tmpdata[1] * 256 + tmpdata[0])+1)&0xefff);
      }
    }

    if(tmpCanID == 0x237) {
      double MSB;
      MSB = (tmpdata[1] >> 7) & 0x01;

      if (MSB == 0) {
          ///左后轮速
          ECUData_struct.BackLeftWheelSpeed = (tmpdata[1] * 256 + tmpdata[0]);
      }
      else{
//          ECUData_struct.BackLeftWheelSpeed = ((tmpdata[1] * 256 + tmpdata[0])-0xffff);
          ECUData_struct.BackLeftWheelSpeed = -((~(tmpdata[1] * 256 + tmpdata[0])+1)&0xefff);
      }
    }

    if(tmpCanID == 0x247) {
      double MSB;
      MSB = (tmpdata[1] >> 7) & 0x01;

      if (MSB == 0) {
          ///右后轮速
          ECUData_struct.BackRightWheelSpeed = (tmpdata[1] * 256 + tmpdata[0]);
      }
      else{
//          ECUData_struct.BackRightWheelSpeed = ((tmpdata[1] * 256 + tmpdata[0])-0xffff);
          ECUData_struct.BackRightWheelSpeed = -((~(tmpdata[1] * 256 + tmpdata[0])+1)&0xefff);
      }
    }

    if(tmpCanID == 0x160) {

      ///方向盘转角

      double MSB;
      MSB=(tmpdata[0]>>7)&0x01;

      if(MSB==0)
      {
            ECUData_struct.fFLRWheelAverAngle=(tmpdata[0]*256+tmpdata[1])*0.1587+0.1454;  //EPS实际转角  左打为正，右打为负,motorola,转化为实际转角
      }
      else
      {
//            ECUData_struct.fFLRWheelAverAngle=((tmpdata[1] * 256 + tmpdata[0])-0xffff)*0.1587+0.1454;     //EPS实际转角  左打为正，右打为负,motorola,转化为实际转角
            ECUData_struct.fFLRWheelAverAngle=-((~((tmpdata[0]*256+tmpdata[1]))+1)&0xefff)*0.1587+0.1454;    //EPS实际转角  左打为正，右打为负,motorola,转化为实际转角
      }

    }
    if(tmpCanID == 0x490) {
      ///车辆制动深度反馈
      double MSB;
      MSB=(tmpdata[0]>>7)&0x01;

      if(MSB==1)
      {
//          ECUData_struct.pressure_petral_feedback=((tmpdata[0]^0xff * 256 + tmpdata[1]^0xff)-0xffff);
          ECUData_struct.pressure_petral_feedback=-((~((tmpdata[0]*256+tmpdata[1]))+1)&0xefff)/12;
        cout<<"##################"<<ECUData_struct.pressure_petral_feedback<<endl;
      }
      else
      {
          ECUData_struct.pressure_petral_feedback=(tmpdata[0]*256+tmpdata[1])/12;    //小车制动减速度反馈,motorola,乘上10mA转化为A
          cout<<"&&&&&&&&&&&&&&"<<ECUData_struct.pressure_petral_feedback<<endl;
      }
    }

  }

}

