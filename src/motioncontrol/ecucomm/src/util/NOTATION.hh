#pragma once
#include <vector>
#include "opencv/cv.h"
#include "opencv/cxcore.h"
#include "opencv/highgui.h"
#include "ToXml.hh"

using namespace std;
#define PI 3.1415926535897932384626433832795


//能够进行elegantdis判断的最大距离
#define MAXDETECTIONDIS 40.0

//换道宽度和车道内平移宽度
#define LANEOFFSETVALUE 4.0
#define LANE_IN_OFFSETVALUE	1.0

//超车计数器
#define con_safe_counter_threshold 7

//粒子滤波数
#define SAMPLENO 2000



#define MAXLATERALACC				1.0//最大侧向加速度
#define MAXLATERALACC_LOWERBOUND	0.0//0.8侧向加速度限制的最小前轮偏角wen
#define MAXLATERALACC_UPPERERBOUND	25//10.5侧向加速度限制的最小前轮偏角wen
 
#define INVALIDVALUE	1000000.0
#define EPSILON_ERROR	0.0000000001

//定时器周期

#define	PLANINTERVAL	100

#define COLORRED	cvScalar(0,0,255)
#define COLORBLUE	cvScalar(255,0,0)
#define COLORGREEN	cvScalar(0,255,0)
#define	COLORWHITE	cvScalar(255,255,255)
#define COLORCYAN	cvScalar(255,255,0)
#define COLORYELLOW	cvScalar(0,255,255)
#define COLORPINK	cvScalar(255,0,255)
#define	COLORGREY	cvScalar(122,122,122)
#define COLORORANGE cvScalar(0,128,255)
#define COLORPURPLE cvScalar(255,0,128)
#define	COLORDARKGREY	cvScalar(188,188,188)
#define	COLORLIGHTGREY	cvScalar(100,100,100)
#define	COLORBLACK	cvScalar(0,0,0)



//#define MAXSPEED		35.0 //10m/s

//激光网格
#define MAXPROB				0.9f//12.0
#define MINPROB				0.3f//2.0
#define UNKNOWNPROB			0.5f//4.0

#define PROB_OCC			0.7f//0.5
#define PROB_POSSIBLEOCC	0.6f//0.5
#define PROB_UNKNOWN		0.5f//0.0
#define PROB_EMP			0.3f//-0.5

#define OGM_PASS			0
#define OGM_UNKNOWN			1
#define OGM_POSSIBLENOPASS	2
#define OGM_NOPASS			3



/****激光接收*****/
#define SICKPTNUM	761
#define	SICKANGLERESOLUTION	0.25
#define	SICKANGLERESOLUTIONMINUSONE	4
#define SICKMAXDATANUM		5000
#define STARTANGLE -5 //相对于车辆航向方向 逆时针为正，顺时针为负。
#define ENDANGLE 185




/***********全局地图常量*************/
#define MAP_WIDTH_CELL 600
#define	MAP_HEIGHT_CELL	600


/**********OGM常量******************/
#define OGMDISPLAYZOOM	1
#define OGMRESOLUTION	0.2
#define OGMWIDTH_M		40	//OGM横轴长度m
#define OGMHEIGHT_M		80	//OGM纵轴长度m




#define	OGMWIDTH_CELL	cvRound(OGMWIDTH_M / OGMRESOLUTION + 1)
#define	OGMHEIGHT_CELL	cvRound(OGMHEIGHT_M / OGMRESOLUTION + 1)
#define VEHICLEPOSINOGM_Y_M 20//车后轴到OGM底边的距离
#define VEHICLEPOSINOGM_X_M 20//车后轴在OGM局部坐标系中的横坐标（m）
#define VEHICLEPOSINOGM_X_CELL cvRound(VEHICLEPOSINOGM_X_M/OGMRESOLUTION)


#define VEHICLEPOSINOGM_Y_CELL cvRound(VEHICLEPOSINOGM_Y_M/OGMRESOLUTION)      //	LXN 10*22 zwk
////////////////////2016-07-12 ltf////////////////////////////////
#define	VEHICLE_DIS_TO_BOTTOMInOGM_M	cvRound(OGMHEIGHT_M-VEHICLEPOSINOGM_Y_M)//后轴到bottom的距离10m
#define VEHICLE_DIS_TO_BOTTOMInOGM_CELL	cvRound(VEHICLE_DIS_TO_BOTTOMInOGM_M/OGMRESOLUTION)//后轴到bottom的距离cell
////////////////////2016-07-12 ltf////////////////////////////////


//类型定义
//根据路网提供的信息，可以进行一系列细分
enum VirtualLaneDecisionMakingMode{
	KeepCurrentLane,
	Change2LeftLane,
	Change2RightLane,

	Overtake2LeftLane,
	Overtake2RightLane,

	Continue2Change,
	Fail2Change
};

enum VirtualLaneDrivingMode{
	LaneKeeping,
	LaneChangingPreparing,
	LaneChanging,
	OvertakingPreparing,
	Overtaking
};



enum DRIVEMODE
{
	DRIVEMODE_UNKNOWN,
	DRIVEMODE_LANE,//车道内行驶
	DRIVEMODE_ROADMAP,//自由区域跟随路网行驶
	DRIVEMODE_FIELDMETHOD//纯环境感知行驶
};

enum ASTARPLANFOLLOWSTATE
{
	AStarPlanandFollowWaitForTarget,
	AStarPlanandFollowStopandWaitForTarget,
	AStarPlanandFollowSTARTPLAN,
	AStarPlanandFollowWAITFORPATH,
	AStarPlanandFollowPLANFAIL,
	AStarPlanandFollowFOLLOW,
	AStarPlanandFollowFOLLOWHESITATE
};

enum PCCLOUDPLANNERSTATE
{
	PCCLOUDPLANNER3DIDLE,
	PCCLOUDPLANNER3DWAITFORTARGET,
	PCCLOUDPLANNER3DLEFTBUTTONDOWN,
	PCCLOUDPLANNER3DTARGETGIVEN,
	PCCLOUDPLANNER3DINPLANNING,
	PCCLOUDPLANNER3DPLANFAIL,
	PCCLOUDPLANNER3DPLANSUCCESS
};

enum ASTARFOLLOWINGSTATE
{
	ASTARFOLLOWINGIDLE,
	ASTARFOLLOWINGNOGLOBALPATH,
	ASTARFOLLOWINGSUCCESS,
	ASTARFOLLOWINGFAIL,
	ASTARFOLLOWINGDANGEOUS,
	ASTARFOLLOWINGFINISH
};


//来自路网信息
enum ROADTYPE
{
	WAYPOINT,
	INTERSECTION,
	UTURN,
	AUTOPARKING,
	INTERSECTIONRELATIVE,
	LANEDRIVING,
	WAYPOINT_CHANNEL,
	SAA_TYPE,
	STATION
};
enum PLANNERMODE{
	VIRTUALLANEPLANNER,
	FREELOCALPLANNER,
	AutoParking,
	SEARCHBASEDPLANNER,
	CHANNELPLANNER,
	LaneDriving
};
//side
enum SIDETYPE
{
	CENTER,
	LEFT,
	RIGHT
};

enum
{
	NO,
	YES
};

enum STOPREASONTYPE
{
	NOSTOP,
	WAITFORSTART,
	NOATTACHMAP,
	OGMINVALID,
	FINISH_STOP,
	ESTOP,
	REDLIGHT,
	PEDESTRIAN,
	LOCALPLANFAIL
};




struct pose
{
	double x;
	double y;
	double weight;
	double theta;
};


union _uintbyte
{

	unsigned short	_uint;
	unsigned char	_uchar[2];
};

union _intbyte
{
	//INT16 _int;
	short _int;
	char	_char[2];
};
struct Cross_Intf //lzz,20170630，齐建永参照更改。
{
	int Orientation;
	float left_param0;
	float left_param1;
	float left_param2;
	float right_param0;
	float right_param1;
	float right_param2;
	float center_param0;
	float center_param1;
	float center_param2;
	bool Boundary_Valid;
	int Valid_Boundary_Num;
};
/**************结构体***************/
struct pos
{
	double x;
	double y;
};

struct pos_int
{
	int x;
	int y;
};

//struct state_struct
//{
//	pos position;
//	double s;//弧长
//	double theta;//弧度，车头与正东方向夹角，heading=pi/2-theta; heading_g=heading_l+heading_v
//	bool forward;//前进或者后退
//
//	double steering_angle;
//	double radius;
//
//	//来自路网的属性
//	int index;
//	char type;
//	double roadwidth;
//	double lanewidth;
//	int lanecount;
//	int lanecount_samedirection;
//	int laneside;
//	double maxvel;
//	int RoadCharactetics;//道路特征.qjy add 20171113
//
//	char side;
//	pos mapmatch_pos;//qjy,地图匹配路网 20171104
//
//	pos gps_pos;//gps路网20171104
//
//
//};

struct TrajPoint
{
	double x;//
	double y;//	
	double theta;//车头与正东方向夹角，而坐标系之间的夹角应该是车头与正北的夹角（局部坐标系中与横轴正向夹角，顺时针为正）
	double s;//到零点的弧长
	double ref_s;//对应的ref的弧长
	double k;//曲率，对应前轮偏角
	double steeringangle;//该路径点的转向角

	double maxvel_by_backstepping;
	double maxvel;//两个trajpoint之间的最大允许速度
	int acc;//该点的加速度，0匀速，-1减速，-2急减速，1加速

	double vel;//
	double t;//到零点的时间
};

struct Trajectory
{
	vector<TrajPoint> Traj_Points;//车体坐标局部路径

	int id;
	double lateral_offset;//用于局部路径时，表示与期望路径的关系
	double width;

	//属性
	double s;//路径长度
	double s_afterturning;

	double cost;

	double max_vel;
	double terminalvel;
	int truncated;
	int dangerour_millimeter_obj_index;

	double safety_dis;
	double elegant_distance;//elegant_distance是需要进行避障的距离，与速度有关。障碍物小于elegant_distance时只需要沿路网前进。在障碍物距离小于elegant_distance之前，
	//必须能够造成车辆减速。只有障碍物距离减小大于车辆减速造成距离小于elegant_distance，才会造成换道。此外，最小的elegant_distance应该能够保证车辆在静止时还有
	//足够的转向空间
	//备注：生成轨迹的长度必须必elegant长，至少保证车辆能够在当前速度下加速超过5m/s，但是从计算量考虑，又不能过长。

	//轨迹对应的控制量
	double steeringangle;
	double vel;
	int use_directcommand;//是直接使用trajectory的控制量还是要进行局部路径跟踪

//	int motor_n;                        //6t平台轨迹跟踪 hjm 20180131
	int steer_degree;					//6t平台轨迹跟踪
	//unsigned char steer_dir;			//6t平台轨迹跟踪
};

struct virtuallanestate_struct
{
	bool selectable;//车道可选，不可选可能由于车道线虚实或者完全不安全（路沿占据）
	unsigned int clearance;//车道线安全情况，0 完全安全， 1 有障碍，2完全挡住
	double v;
	double max_safety_range;//局部规划的最长距离（车道内，不靠边）
};

typedef struct struct_VehicleStateToNet			// 车辆状态
{
	double fPos_east;					// 车辆东向位置(Unit:m)	绝对的GPS坐标，定时初始化integrated位置，用于全局定位，更新周期1s
	double fPos_north;					// 车辆北向位置(Unit:m)	

	double fDRPos_E;					// 航迹推算位置东向(m) 绝对坐标系下的DR结果（从起点起），用于生成全局期望路径路径跟踪控制
	double fDRPos_N;					// 航迹推算位置北向(m)

	double fold_Pos_east;				//记录上次gps定位结果			
	double fold_Pos_north;	
	double fold_DRPos_E;				//记录上次DR定位结果
	double fold_DRPos_N;

	double f_integrated_east;			//用于车辆在全局路径中定位，并从roadmap中确定全局期望路径的起点，更新周期100ms。
	double f_integrated_north;

	/*******地图匹配相关定位结果，不用于规划控制，只用于地图更新********/
	double f_slamcorrected_theta;				
	double f_slamcorrected_east;
	double f_slamcorrected_north;

	///***根据里程计和转弯半径估算的位置，仅作显示用***/
	double Odometer_pre;
	double Odometer_theta;
	double Odometer_x;
	double Odometer_y;


	/**********状态变量*********/
	unsigned char FONSValid ;

	double fForwardVel;					// 车辆纵向速度(Unit:m/s)		
	double fDeForwardVel;
	double fFLRWheelAverAngle;			// 名义前轮偏角，对应电机或方向盘的角度(Unit:°)
	double fHeading;						// 车辆真北航向角(Unit:弧度)
	double fTheta;						//车辆到正东的夹角(Unit:弧度)
	double fOdometer;
	double fRadius;

	unsigned char f_shift;				//档位 0无效1P2R3N4N5D6M7S8+9-
	unsigned char f_shift1;				//具体档位
	unsigned char f_estop;				//紧急制动
	unsigned char f_leftlamp;			//左转向灯
	unsigned char f_rightlamp;			//右转向灯
	bool lateralctrl_enabled;
	bool longitutdectrl_enabled;
	bool brake_enabled;

	double lTimeStamp;			// 时间戳(Unit:ms)

	//************    BTV AND MSH *******************//
	//***********************************************//
	//***********************************************//

	unsigned char autodrive_status;   //add
	unsigned char brake_pedal_signal;  //add
	unsigned char switch_signal;  //add


	double pressure_back;         //后油路压力
	double petral_pressure;       //制动踏板压力 

	int throtle_feedback;
	unsigned char steerRx_err;
	unsigned char steerTx_err;
	unsigned char brakeRx_err;
	unsigned char brakeTx_err;
	unsigned char PC_Tx_err;

	unsigned char poweron_status;
	unsigned char start_status;
	unsigned char warning_status;
	unsigned char bugle_status;

	unsigned char light_far;
	unsigned char light_near;

	unsigned char Estop_enabled;

	unsigned short EnginRate;
	/*************************///qjy 20180126
	double pitch;
	double roll;

	double dAccx;
	double dAccy;
	double dAccz;

	double dArx;	//Pear
	double dAry;
	double dArz;

	double ve;
	double vn;
	double vu;

}SVehicleStateToNet,*PSVehicleStateToNet;

typedef struct struct_DeVehicleStateToNet		// 期望的车辆状态
{
	unsigned char shift;
	double fDeForwardVel;				// 期望车辆纵向速度(Unit:m/s)		
	double fDeFLRWheelAverAngle;			// 期望车辆左右前轮平均偏角(Unit:°)
	double fDeHeading;					// 期望车辆真北航向角(Unit:°)
	bool lateralctrl_enabled;
	bool longitutdectrl_enabled;
	bool leftlamp_turnedon;
	bool rightlamp_turnedon;
	double lTimeStamp;			// 时间戳(Unit:ms)


	double kp , ki , kd ;
	double slope_gas , slope_brake;

	double u_gas;//油门初始值
	double u_brake;//制动初始值

	unsigned short throttle_upper_threshold_low;
	unsigned short throttle_upper_threshold_high;
	unsigned short throttle_actual_threshold;
	unsigned short throttle_init;

	unsigned short brake_upper_threshold_high;
	unsigned short brake_upper_threshold_low;
	unsigned short brake_init;

	unsigned short throttle;
	unsigned short brake;

	unsigned short e_stop_limit;

	int steering_ctrl;

	double delta_u;
	double u;

	//*********   BTV AND MSH ************//
	//************** add *****************//
	//************************************//

	unsigned char auto_enable;
	unsigned char poweron_enable;
	unsigned char engineon_enable;
	unsigned char warning_enable;
	unsigned char bugle_enable;
	unsigned char lightn_enable;
	unsigned char lightf_enable;
	unsigned char shift_enable;

	// hjm 0131
	int steer_degree;//转向程度
	int steer_direction;//转向方向
	int motor_n;//电机转速


}SDeVehicleStateToNet,*PSDeVehicleStateToNet;


struct Straightline
{
	double x1;
	double y1;
	double x2;
	double y2;
	double theta;
	char continuous;
	char color;
	double len;
};

struct _StraightLaneStruct
{	
	//x1,x2,y1,y2: mm
	double x1;
	double y1;
	double x2;
	double y2;
	double x3;
	double y3;
	double x4;
	double y4;
	char rightcon;//0没有//1间断//2连续
	char leftcon;//0没有//1间断//2连续
	int lanestatus;//车道数
	int prev_lanestatus;
	double lTimeStamp;
};

struct _OGMDataStruct
{
	unsigned char* m_OccupiedMap;
	double lTimeStamp;
};

struct _RNDFInfoStruct
{
	//其中都用的弧度表示航向
	double distointer;// 到下一个路口节点的距离
	//double distoway;// 到下一个路口节点的距离
	double distonextnode;
//	state_struct prev_node_state;//当前节点，用于判断路点属性判断速度
//	state_struct next_node_state;//下一节点的位置和期望航向，注意对于路口出点，航向是点的航向，而不是路的，对于路口入点，航向是路的
//	state_struct third_node_state;
//	state_struct next_intersection_state;
};

struct trajsegmentstruct
{
	int startindex;
	int endindex;
	double width;
	int id;
};

struct segmentstruct
{
	int startindex;
	int endindex;
	double anglerange;
	int id;
	double dis;

	double gap;
	unsigned char feasible;
	char outlet;
	int targetindex;
	double score;
};

struct points2D
{
	double distance[SICKPTNUM];
	double achievable_distance[SICKPTNUM];
	double tmpdistance[SICKPTNUM];
	double intensity[SICKPTNUM];  //20130305添加 保存回波强度
	int id[SICKPTNUM];
	segmentstruct segment[SICKPTNUM];
	double timestamp;
};

struct moving_object_millimeter
{
	//原始数据
	double range;
	int angle;
	double v;
	double x;
	double y;
	double vx;
	double vy;
	int valid_count;

	//kalman滤波结果
	//double x_range_prob;//包含车宽信息
	double x_range_right;
	double x_range_left;

};

struct moving_object_millimeter_prob
{
	//kalman滤波结果
	CvKalman* kalman;
	double occupancyprob_grid_x[101];//20 / OGMRESOLUTION + 1 : +-10米
};
