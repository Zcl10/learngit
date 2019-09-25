//
// Created by wangwei on 18-3-16.
//

#include "../../include/ECUDataOutPut.h"
using std::cout; using std::endl;
ECUDataOutPut::ECUDataOutPut()
{
    RunningStatus=0;
    longitutdectrl_enabled=false;
    autodriving=false;
    RunningTimerNum=0;
    counter=0;
    angle=-1.0;
}

ECUDataOutPut::~ECUDataOutPut()
{
}

bool ECUDataOutPut::Init(const char* to_ip,int port)
{
  cout<<"[INFO] Init Send port is: "<<port<<endl;
  ///定义senddata to ECUSOCK_DGRAM
  server_sockfd = socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
  memset(&myaddr, 0, sizeof(myaddr));
  myaddr.sin_family = AF_INET;
  myaddr.sin_port = htons(9003);
  myaddr.sin_addr.s_addr = htonl(INADDR_ANY);

  memset(&remaddr,0,sizeof(remaddr));
  ///定义sockaddr_in
  remaddr.sin_family = AF_INET;
  remaddr.sin_port = htons(port);
  remaddr.sin_addr.s_addr=inet_addr(to_ip);
  return true;
}

bool ECUDataOutPut::SendDeVehicleState()
{
    _intbyte intbytetemp;
    unsigned char FI=0b00001000;
    unsigned char tmpCanID1=0b00000000;
    unsigned char tmpCanID2=0b00000000;

    //Send info to ID 0x151
    unsigned char send_buf_4F0[13];
    memset(send_buf_4F0,0,sizeof(send_buf_4F0));
    send_buf_4F0[0]=FI;
    send_buf_4F0[1]=tmpCanID1;
    send_buf_4F0[2]=tmpCanID2;
    send_buf_4F0[3]=0x01;
    send_buf_4F0[4]=0x51;

    //Send info to ID 0x150
    unsigned char send_buf_4F1[13];
    memset(send_buf_4F1,0,sizeof(send_buf_4F1));
    send_buf_4F1[0]=FI;
    send_buf_4F1[1]=tmpCanID1;
    send_buf_4F1[2]=tmpCanID2;
    send_buf_4F1[3]=0x01;
    send_buf_4F1[4]=0x50;

    ///纵向控制
    if (!this->longitutdectrl_enabled)
    {
        SendECUData_struct.brake = 10;
        SendECUData_struct.throttle=0;
    }
    else
    {
        std::cout<<"纵向控制使能"<<std::endl;
    }

    int throttle =SendECUData_struct.throttle;
    intbytetemp._int = throttle;
    //// 设置左前轮转矩
    send_buf_4F1[5] = (throttle & 0xff00)>>8;
    send_buf_4F1[6] = throttle & 0x00ff;
//        send_buf_4F1[5] = intbytetemp._char[1];
//        send_buf_4F1[6] = intbytetemp._char[0];

    /// 设置右前轮转矩
    send_buf_4F1[7] = (throttle & 0xff00)>>8;
    send_buf_4F1[8] = throttle & 0x00ff;
//        send_buf_4F1[7] = intbytetemp._char[1];
//        send_buf_4F1[8] = intbytetemp._char[0];

    /// 设置左后轮转矩
    send_buf_4F1[9] = (throttle & 0xff00)>>8;
    send_buf_4F1[10] = throttle & 0x00ff;
//        send_buf_4F1[9] = intbytetemp._char[1];
//        send_buf_4F1[10] = intbytetemp._char[0];

    /// 设置右后轮转矩
    send_buf_4F1[11] = (throttle & 0xff00)>>8;
    send_buf_4F1[12] = throttle & 0x00ff;
//        send_buf_4F1[11] = intbytetemp._char[1];
//        send_buf_4F1[12] = intbytetemp._char[0];

    ///刹车
    if (SendECUData_struct.brake > 0) {
        intbytetemp._int = SendECUData_struct.brake;
        send_buf_4F0[9] = intbytetemp._char[1];
        send_buf_4F0[10] = intbytetemp._char[0];
    } else{
        /// 0xFFCE：-0.5A 为厂家建议的制动电机反转电流
        send_buf_4F0[9] = 0xFF;
        send_buf_4F0[10] = 0xCE;
    }

    ///横向控制
    int steer;
    if (!this->lateralctrl_enabled)
        steer = (0-0.1454)/0.1587;      //方向盘归零位
    else{
        std::cout<<"横向控制使能"<<std::endl;
        steer = (SendECUData_struct.steering_ctrl-0.1454)/0.1587;         //将轮胎转角转化为电机转角
    }
    send_buf_4F0[5] = (steer & 0xff00)>>8;
    send_buf_4F0[6] = steer & 0x00ff;
//    send_buf_4F0[7] = (steer & 0xff00)>>8;
//    send_buf_4F0[8] = steer & 0x00ff;


    cout << "send_buf_4F0: ";
    for (int i = 3; i < 13; i++) {
        printf(" %X",send_buf_4F0[i]);
    }
    cout << endl;

    cout << "send_buf_4F1: ";
    for (int i = 3; i < 13; i++) {
        printf(" %X",send_buf_4F1[i]);
    }
    cout << endl;

  //发送
    int send_len0 = sendto(server_sockfd,send_buf_4F0,sizeof(send_buf_4F0),0,(sockaddr*)&remaddr,sizeof(remaddr));
    if(send_len0<0){
        perror("[ERROR]cannot send CAN_ID 0x151 data!");
    }

    int send_len2 = sendto(server_sockfd,send_buf_4F1,sizeof(send_buf_4F1),0,(sockaddr*)&remaddr,sizeof(remaddr));
    if(send_len2<0){
        perror("[ERROR]cannot send CAN_ID 0x150 data!");
    }

    return true;

}

