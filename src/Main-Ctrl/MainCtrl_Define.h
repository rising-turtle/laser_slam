#pragma once
//AGMap
#define _1M 1024*1024
#define _10M 10*1024*1024
#define _100M 100*1024*1024
#define MAX_RAWDATA_FILE_NUM 100
#define PROB_MAP_W 50
#define PROB_MAP_H 50

#define RTN_OK 0
#define RTN_LOSS_RS_CNC -11
#define RTN_RS_CNC_NOR 11
#define RTN_RS_CNC_RECNC 12

#define RTN_LOSS_LASER_CNC -21


#define LASER_NUM 2

#define LOG_NET 0
#define LOG_SLAM 1
#define LOG_IOA 2
#define LOG_SUBCTRL 3

//IOA
#define SECURITY_AREA_NUM 5
#define LELVEL4 500
#define LELVEL3 400
#define LELVEL2 150
#define LELVEL1 150
#define LELVEL0 50

#define BLEND_LEVEL 0.3

#define MAX_SPEED_L4  400
#define MAX_SPEED_L3  500
#define MAX_SPEED_L2  500
#define MAX_SPEED_L1  400
#define MAX_SPEED_L0  300

typedef int (*CallBack_BNLctAndOdometry)(char *pcData);
typedef int (*CallBack_LogFile)(char* pcContent,int nModuleIdx);//2012 11 14 add

typedef int (*CallBack_Forward)();
typedef int (*CallBack_Backward)();
typedef int (*CallBack_Turn)();
typedef int (*CallBack_Break)();



typedef int (*CallBack_)(char *pcData);

typedef struct  SimpleCtrlCmd
{
	CallBack_Forward cbForward;
	CallBack_Backward cbBackward;
	CallBack_Turn cbTurn;
	CallBack_Break cbBreak;
}SimpleCtrlCmd;






typedef int (*CallBack_NetUpload)(char *pcData,int nDataLen);



typedef int (*CallBack_Update2DGridMap)();



typedef int (*CallBack_LaserData)(char *pcData,int nDataLen);
typedef int (*CallBack_LaserDtct)(char *pcDtctRslt);



typedef int (*CallBack_TaskIn)(int nTaskType,char *pcData,int nDataLen);

//Message Define
#define NEW_TASK_PATH  0x0001
#define RE_TASK_PATH   0x0002
#define ROB_REACH_MIL  0x0003
#define GRID_MAP_IN    0x0010
#define SLOW_BREAK     0x1000
#define HEART_BIT      0xFFF0
#define OLD_PC_DATA    0x0013
#define CLEAR_PC_DATA    0x0011



#define ERR_TOLERATE 0.3


typedef struct Point_f
{
	float fX;
	float fY;
	float fLen;
}Point_f;

typedef struct Pos_f
{
	Point_f stPoint;
	float fTheta;
}Pos_f;

typedef struct RectilinearCtrl
{
	char cType;
	float fTAcc;
	float fTUni;
	float fTDeacc;
	float fVm;
	float fA;
	float fCurSpd;
}RectilinearCtrl;


typedef struct AccTable
{
	float fAccLen;
	float fDeacclen;
}AccTable;

#define MOTION_ERR 0.5
#define WHEEL_RADIOUS 200
#define PI 3.1415926

#define MAX_ACC_SPD 0.8

#define MAX_DEACC_SPD -0.4

#define MAX_TURNNING_SPD 0.1
//#define MAX_TURNNING_SPD 0.5

#define CMD_SLICE_LEN 0.05
#define CMD_SLICE_HZ 1/CMD_SLICE_LEN

#define MIN_RECTIL_LEN 1
#define IDEAL_RADIUS 1.5
#define IDEAL_PRE_RADIUS 2
#define TURNNING_TIME 1


#define SICK_LINES 541
#define INIT_ANGLE 225
#define SICK_RESOLUTION 0.5
#define SECURTY_WIDTH 50
#define SPD_LMT_NUM 4


//Robot Size
#define ROBOT_L 540





typedef struct IOA_Pos
{
	float fX;
	float fY;
	int nIdxX;
	int nIdxY;

	float fDis;
}IOA_Pos;


#define MAP_INIT_WEIGHT 0
#define MAP_UNFREE_AREA 255
#define MAP_FREE_AREA 200
#define MAP_TMP_OBSTACLE 100
#define MAP_STATIC_OBSTACLE 1

#define ROBOT_WIDTH_IN_2DMAP 6

#define MIN(a,b)((a)<(b))?(a):(b)

//Channel B Protocol



typedef int (*CallBack_SendNKJCMD)(float fVL,float fVR,int nLTime,int nRTime);
typedef int (*CallBack_SendNKJCMD_Rot)(float RotDegree, float Rot_Velocity);

#define MAX_SPD 0.70001

#define ADJUST_LEN_THRS 2
#define ADJUST_ANGLE_THRS 0.05
#define ADJUST_Len 1.5


#define OPEN_IOA
#define OPEN_ADJUST
#define SLAM_LOCATION
#define KEEP_4_SLAM_LEN 0.5

#define TURN_POS_ERR 0.4

#define OPEN_BN
#define USING_SLAM_DATA_THRS 0.5
#define TURNNING_TIME 3

#define X_OFFSET -0.12
#define Y_OFFSET 0.02

#define _90DEGREE 3.1415926/2

#define CTRL_BATTERY_SAFE_VOLT 20
#define POWER_BATTERY_SAFE_VOLT 40


#define SYS_ERR_CTRL_BATTERY_LOW 1
#define SYS_ERR_POWER_BATTERY_LOW 2
#define SYS_LOST_CNC_SICK_A 3
#define SYS_LOST_CNC_SICK_B 4
#define SYS_LOST_BN_SERIAL 5
#define SYS_LOST_LOW_CTRL_SERIAL 6

#define PT_LINE_ERR 0.3

#define SPIN_ANGLE_VEL 3.1415926/180*10

#define MAX_MAINTAIN_TIME 9999


//Zhanghe
#define BLIND_TIME 50


//#define OPEN_SELF_CHECK


//IOA
#define MIN_SERITY_LEN 0.5

#define MAX_STORAGE_LINES 10000
#define ONE_LINE_LEN (541+3)*4


//Trust Location Data
#define TRUST_DATA_FUSION 0
#define TRUST_SLAM_RSLT 1
#define TRUST_ODO_RSLT 2
#define TRUST_BN_RSLT 3

//Send Location Data 2 RS
#define FUSION_DATA 0
#define SLAM_DATA 1
#define ODO_DATA 2
#define BN_DATA 3

#define OFFSET_ORI_2DGRIDMAP_X  9.8
#define OFFSET_ORI_2DGRIDMAP_Y  27.1

