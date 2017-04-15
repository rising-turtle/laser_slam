#pragma once
#include "../MainCtrl_Define.h"

#include "./SerialCom/SerialCom.h"
class SubCtrlCom
{
public:
	SubCtrlCom(void);
	~SubCtrlCom(void);

	int SubCtrlComInit(char *pcUSBPort);
	int SubCtrlComRun();
	int SubCtrlComStop();
	int SubCtrlComUninit();

	CallBack_BNLctAndOdometry m_cbBNlctAndOdometry;
	CallBack_LaserDtct m_cbLaserDtct[2];

	static int Forward();
	static int Backward();
	static int Break();
	static int Turn();

	static int Odometry(float *pfOdometry);

	CallBack_LogFile m_cbLogFile;

	static SerialCom *m_pCSerialCom;

	static int SendNKJCmd(float fVL,float fVR,int nLTime,int nRTime);
	static int SendNKJCmd_Rot(float RotDegree, float Rot_Velocity);
	static int SendNKJCmd_ClearOdo();
	static int SendGetRobotStatus();
	static int ReadData(char *pcData,int nDataLen);
	
	static int GetRobotOdo();
	static int GetRobotStatus();

};	
