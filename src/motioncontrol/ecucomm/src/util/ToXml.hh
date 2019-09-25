#pragma once
//选择平台
//#define TOYOTAPLATFORM
#define BEIQIBJ80
//#define BYDRAY
//#define BYDTANG
//#define VREP
//#define PRESCAN
//#define FOTONBUS//冬奥会大巴项目,福田客车
//更换平台的时候，请相应更改GetECUData模块的make文件。

//导航信息是否准确。差别在于drpos是用惯导还是用里程计。dr作用：1用于跟踪部分的路网全局局部相互转换用drpos（注意仅限于短时间内切换）。
//slam定位初始化用intergration ，后面更新用dr
#define POSITIONISACCURATE

//前轮偏角
#define STEERINGANGLELIMIT 35.0//为适应福田项目，25改为35

//是否使用鲁棒性因素判断，可能影响通过性
//#define USEROBUSTNESS




//是否使用毫米波
#define ENABLERADAR
//#define MIDBUS_HAILIANG

#ifdef FOTONBUS
//车辆定平台义
#define steeringratio_l  177.167
#define steeringratio_r 177.167
#define kp1_1 300.0
#define ki1 2.0
#define kd1 5
#define gas_slope1 0.08
#define gas_entry_threshold 3
#define gas_init 5
#define gas_limit_low 25
#define gas_limit_high 35
#define brake_slope 1.5
#define brake_entry_threshold 9
#define brake_init1 12
#define brake_limit_low 18
#define brake_limit_high 25
#define estop_brake 25

#endif

#ifdef BYDRAY
//车辆定平台义
#define BYDRAYYUANZHENG
#define steeringratio_l  193.26
#define steeringratio_r 192.075
#define kp1_1 300.0
#define ki1 2.0
#define kd1 5
#define gas_slope1 0.08
#define gas_entry_threshold 3
#define gas_init 5
#define gas_limit_low 25
#define gas_limit_high 35
#define brake_slope 1.5
#define brake_entry_threshold 9
#define brake_init1 10
#define brake_limit_low 18
#define brake_limit_high 28
#define estop_brake 25

#endif

#ifdef BYDTANG
//#define TANGPLATFORM_76GF6/*********无人唐，此宏定义牵涉到GetGPSData、GetINSData、GetECUData、SendECUData模块***************/
#define TANGPLATFORM_KI0E5/*********有人唐，此宏定义牵涉到GetGPSData、GetINSData、GetECUData、SendECUData、IV_PathPlan模块***************/
#define steeringratio_l  177.167
#define steeringratio_r 177.167
#define kp1_1 300.0
#define ki1 2.0
#define kd1 5
#define gas_slope1 0.08
#define gas_entry_threshold 3
#define gas_init 5
#define gas_limit_low 2000
#define gas_limit_high 2200
#define brake_slope 1
#define brake_entry_threshold 9
#define brake_init1 12
#define brake_limit_low 18
#define brake_limit_high 25
#define estop_brake 25


#endif

#ifdef VREP
#define VREP_RAY//杨磊用
#endif

#ifdef VREP_RAY
//车辆定平台义
#define steeringratio_l  193.26
#define steeringratio_r 192.075
#define kp1_1 300.0
#define ki1 2.0
#define kd1 5
#define gas_slope1 0.08
#define gas_entry_threshold 3
#define gas_init 5
#define gas_limit_low 25
#define gas_limit_high 35
#define brake_slope 1.5
#define brake_entry_threshold 9
#define brake_init1 10
#define brake_limit_low 18
#define brake_limit_high 28
#define estop_brake 25

#endif
#ifdef PRESCAN
#define PRESCAN1
#endif
