// The following ifdef block is the standard way of creating macros which make exporting 
// from a DLL simpler. All files within this DLL are compiled with the IICTRL_EXPORTS
// symbol defined on the command line. this symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see 
// IICTRL_API functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.
/*
#ifdef IICTRL_EXPORTS
#define IICTRL_API __declspec(dllexport)
#else
#define IICTRL_API __declspec(dllimport)
#endif
*/
#pragma once
#define IICTRL_API

#define VERSION ("20121010.V.1.1")

#define    UINT32     unsigned int
#define    UINT16     unsigned short
#define    UINT8      unsigned char
#define    INT8       char
#define    INT16      short
#define    INT32      int

#define    BYTE		char

//车体2轮之间间距，即车体直径
#define    ROBOT_TREAD  500
//车轮直径
#define    ROBOT_WHEEL_DIAM 300
#define    PI 3.142
//轮子周长
#define    CIRCUMFERENCE  (PI * ROBOT_WHEEL_DIAM)

typedef enum{
	PUSH_DATA_OUT_OF_LEN = -10,
	SPEED_OUT_OF_RANGE = -20,
	PORT_OPEN_ERROR = -30,
	PRARAM_VALUE_ERROR = -100,
	SUCCESS = 0,
}ERRLIST;


typedef enum{
	SENSOR_LASER = 0x1,
}SENSORFLAG;

typedef enum{
	SENSOR_CLOSE = 0x0,
	SENSOR_OPEN  = 0x1,
}SENSORSTATE;

typedef enum{
	TURN_LEFT_DIRC   = 0x0,
	TURN_RIGHT_DIRC  = 0x1,
}TURNDIRC;

typedef enum{
	LEFT_VOLTAGE = 0x01,
	RIGHT_VOLTAGE = 0x02,
	LEFT_CURRENT = 0x08,
	RIGHT_CURRENT = 0x10,
	ROBOT_TEMPERATURE = 0x20,

}ROBOTSTATE;

typedef enum{
	READ_LEFT_ENCODER = 0x01,
	READ_RUGHT_ENCODER = 0x02,
	READ_IMU_VALUE = 0x04,
}READMOTION;

typedef enum{
	READ_LASER_STATE = 0x01,
	READ_IMU_STATE = 0x02,
	READ_IR_STATE = 004,
}READSENSOR;


IICTRL_API INT32   IICtrl_Init(unsigned int port);
IICTRL_API INT32   IICtrl_Uninit();

IICTRL_API INT8    IICtrl_OpenSensor (const INT32 nSensorFlag, const INT32 nHoldTime);
IICTRL_API INT8    IICtrl_CloseSensor(const INT32 nSensorFlag, const INT32 nHoldTime);

IICTRL_API INT8    IICTRL_Wheel_nW  (const  INT32  nWL, const  INT32  nWR,  const  INT32  nHoldTime );
IICTRL_API INT8    IICTRL_Wheel_nV  (const  INT32  nVL, const  INT32  nVR,  const INT32  nHoldTime );

IICTRL_API INT8    IICTRL_Motion    (const INT32 nV,    const INT32   nR,   const INT8 cDirect, const INT32  nHoldTime);


IICTRL_API INT8    IICTRL_PushBackWheelArray_nW  (const  INT32 *pnWL,  const INT32  *pnWR, const  INT32  nArrayLen, const INT32 nTimeSlice, const INT32  nHoldTime);
IICTRL_API INT8    IICTRL_PushBackWheelArray_nV  (const INT32  *pnVL,  const INT32  *pnVR,  const INT32  nArrayLen, const INT32 nTimeSlice,  const INT32  nHoldTime);

IICTRL_API INT8    IICTRL_RunWheelArray          (const  INT32  nHoldTime);


IICTRL_API INT8    IICTRL_HaltWheelArray_nW   (const  INT32  nWL, const  INT32  nWR,   const  INT32  nHoldTime);
IICTRL_API INT8    IICTRL_HaltWheelArray_nV   (const  INT32  nVL, const  INT32  nVR,  const  INT32  nHoldTime);

IICTRL_API INT8    IICTRL_CleanWheelArray     (const  INT32  nHoldTime);

IICTRL_API INT8    IICTRL_FreezeRobot         (const  INT32  nHoldTime);
IICTRL_API INT8    IICTRL_RecoverRobot        (const  INT32  nHoldTime);


IICTRL_API INT8    IICTRL_ReadRobotState     (INT8 *pcState, const  INT32 nBuffsize,  const  INT32 nRobotStateFlag );
IICTRL_API INT8    IICTRL_ReadMotion         (INT8 *pcState, const INT32 nBuffsize, const  INT32 nFlag);
IICTRL_API INT8    IICTRL_ReadSensorState    (INT8 *pcState,  const INT32 nBuffsize, INT32 nFlag);
