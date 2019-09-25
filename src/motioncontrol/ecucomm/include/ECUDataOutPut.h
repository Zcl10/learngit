//
// Created by wangwei on 18-3-16.
//

#ifndef ECUCOMM_CECUDATAOUTPUT_H
#define ECUCOMM_CECUDATAOUTPUT_H

//#include "util/iv_inc.hh"
#include <arpa/inet.h>

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
#include "util/NOTATION.hh"
#include "AnalysisECU.h"

struct struct_SenToECU			// 车辆状态,udp通信用
{

    unsigned int shift;
    //	bool lateralctrl_enabled;
    //	bool longitutdectrl_enabled;
    bool leftlamp_turnedon;
    bool rightlamp_turnedon;
    long lTimeStamp;			// 时间戳(Unit:ms)
    int throttle;
    int brake;
    int e_stop_limit;
    int steering_ctrl;
    int lightmode;
    double fDeFLRWheelAverAngle;
    bool brake_enabled;
    //	int e_stop_limit;
};

class ECUDataOutPut {
public:
    struct_SenToECU SendECUData_struct;
    struct_ECU vehicle_status;
    ECUDataOutPut();
    ~ECUDataOutPut();
    bool Init(const char* to_ip,int port);
    void DataOutput();
    bool autodriving;
    bool longitutdectrl_enabled;
    bool lateralctrl_enabled;
    int RunningStatus;

    struct_VehicleStateToNet m_sVehicleStateToNet;
    struct_DeVehicleStateToNet sDeVehicleStateToNet; // TODO: need change to ROS msgs

    bool SendDeVehicleState2();
    bool SendDeVehicleState1();
    bool SendDeVehicleState();
private:
//    int RunningStatus;
    int RunningTimerNum;



    sockaddr_in myaddr; /* our address */
    sockaddr_in remaddr; /* remote address */
    socklen_t addrlen; /* length of addresses */
    int server_sockfd;
    int recvlen; /* # bytes received */
    int fd; /* our socket */
    unsigned char counter;
    double angle;

//    std::fstream data_backup;
};


#endif //ECUCOMM_CECUDATAOUTPUT_H
