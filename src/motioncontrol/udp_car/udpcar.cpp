#include "udpcar.hpp"

namespace carnet {
udp_car::udp_car(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private,
                 std::string node_name)
    : nh_(nh),
      longitutdectrl_enabled(false),
      lateralctrl_enabled(false),
      nh_private_(nh_private),
      node_name_(node_name) {
    initialize();
}

void udp_car::joyCallback(const sensor_msgs::Joy joy)
{
    cmd_steer.data = steer_gain*joy.axes[steer_position];      //方向盘转角
    cmd_brake.data = brake_gain*joy.axes[brake_position]+500;      //制动踏板行程
    cmd_throttle.data = throttle_gain*joy.axes[throttle_position]+50; //油门踏板行程
    auto_enable = joy.buttons[auto_enable_position];
    auto_enable_cancel = joy.buttons[auto_enable_cancel_position];
}

bool udp_car::initSocket() {

    this->addr_remote_len = sizeof(this->addr_local);

    //// create a socket
    this->CarNetSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (this->CarNetSocket < 0) {
        perror("create CarNetSocket failed!\n");
        return false;
    } else {
        std::cout << "create CarNetSocket succeed!" << std::endl;
    }
    //// set the local address
    memset((char *) &addr_local, 0, sizeof(addr_local));
    this->addr_local.sin_addr.s_addr = htonl(INADDR_ANY);
    this->addr_local.sin_family = AF_INET;
    this->addr_local.sin_port = htons(local_port);

    //// bind the socket with local address
    if (bind(CarNetSocket, (sockaddr *) &addr_local, sizeof(sockaddr)) < 0) {
        perror("bind the CarNetSocket failed!");
        return false;
    } else {
        std::cout << "bind the CarNetSocket succeed!" << std::endl;
        std::cout << "Local Port : " << this->local_port << std::endl;
    }
    //// set the remote address
    memset(&addr_remote, 0, sizeof(addr_remote));
    this->addr_remote.sin_addr.s_addr = inet_addr(remote_ip.c_str());
    this->addr_remote.sin_family = AF_INET;
    this->addr_remote.sin_port = htons(remote_port);

    memset(&addr_vel_display, 0, sizeof(addr_vel_display));
    this->addr_vel_display.sin_addr.s_addr = inet_addr(vel_display_ip.c_str());
    this->addr_vel_display.sin_family = AF_INET;
    this->addr_vel_display.sin_port = htons(vel_display_port);

    std::cout << "Remote IP  : " << this->remote_ip.c_str() << std::endl;
    std::cout << "Remote Port: " << this->remote_port << std::endl;

    return true;
}

void udp_car::sendmsgs() {
//    因为循环周期从原来的0.02s改为0.01s，所以隔一次发送。
    ROS_INFO("sending messages");

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
        cmd_steer.data = 0;
	//cmd_throttle.data = 0;
    }
    else
    {
        std::cout<<"纵向控制使能"<<std::endl;
    }

    int throttle = cmd_throttle.data;
    ROS_WARN("=======: %d",throttle);

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
    ROS_INFO("=======: %f",cmd_brake.data);
    if (cmd_brake.data > 0) {
        intbytetemp._int = cmd_brake.data;
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
        steer = (cmd_steer.data-0.1454)/0.1587;         //将轮胎转角转化为电机转角
    }

    ROS_ERROR("=======: %d",steer);
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

//    sendto(this->CarNetSocket, buffer, 12, 0, (struct sockaddr *) &addr_remote, addr_remote_len);
//    sendto(this->CarNetSocket, buffer, 12, 0, (struct sockaddr *) &addr_remote, addr_remote_len);
	
    if(longitutdectrl_enabled && lateralctrl_enabled){
    	sendto(this->CarNetSocket, send_buf_4F0, sizeof(send_buf_4F0), 0, (struct sockaddr *) &addr_remote, addr_remote_len);
    	sendto(this->CarNetSocket, send_buf_4F1, sizeof(send_buf_4F1), 0, (struct sockaddr *) &addr_remote, addr_remote_len);
    }

}

void udp_car::UpdateMsg() {
    //// Receive Other Car State
    unsigned char recvBuf[4096];  //// receive buffer
    int recvlen;                  //// bytes received
    recvlen = recvfrom(CarNetSocket, recvBuf, 4096, MSG_DONTWAIT, (struct sockaddr *) &addr_remote, &addr_remote_len);
    if (recvlen > 0 && this->addr_remote.sin_port == htons(this->remote_port)) {
        std::cout << " RECV DATA NOW!!! " << std::endl;
        cmddataupdate = true;

        if (recvBuf[0] == 0xF3 && recvBuf[1] == 0x01) {
            std::cout << "可以" << std::endl;
            if (recvBuf[9] == 1)
                std::cout << "手动遥控方式" << std::endl;
            else if (recvBuf[9] == 2)
                std::cout << "自主方式" << std::endl;
            else if (recvBuf[9] == 3)
                std::cout << "静默值守" << std::endl;

        }
    }
}

void udp_car::initialize() {
//    local_port = 8005;
//    remote_ip = "172.20.10.4";
//    remote_port = 13010;
    local_port = 8500;
    remote_ip = "192.168.0.10";
    remote_port = 8400;

    vel_display_ip = "127.0.0.1";
    vel_display_port = 9000;

    ecudataupdate = false;
    cmddataupdate = false;

    nh_.param("axis_steer", steer_position, steer_position);
    nh_.param("axis_brake", brake_position, brake_position);
    nh_.param("axis_throttle", throttle_position, throttle_position);
    nh_.param("steer_gain", steer_gain, steer_gain);
    nh_.param("brake_gain", brake_gain, brake_gain);
    nh_.param("throttle_gain", throttle_gain, throttle_gain);

    nh_.param("auto_enable_position", auto_enable_position, auto_enable_position);
    nh_.param("auto_enable_cancel_position", auto_enable_cancel_position, auto_enable_cancel_position);

    initSocket();
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &udp_car::joyCallback, this);
    loop_timer_ = nh_.createTimer(ros::Duration(0.05), boost::bind(&udp_car::timerCb, this));
}

void udp_car::timerCb() {
    ROS_INFO_ONCE("udp car start");
    if(auto_enable){
        longitutdectrl_enabled = true;
        lateralctrl_enabled = true;
    } else if (auto_enable_cancel){
        longitutdectrl_enabled = false;
        lateralctrl_enabled = false;
    } 
    sendmsgs();
}
}//namespace carnet

int main(int argc, char **argv) {
    std::string node_name = "udp_car";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");
    carnet::udp_car sender(nh, nh_private, node_name);
    ROS_INFO("Initialized sender node.");
    ros::spin();
}
