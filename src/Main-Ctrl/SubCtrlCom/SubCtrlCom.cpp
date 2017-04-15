#include "SubCtrlCom.h"


SerialCom *SubCtrlCom::m_pCSerialCom;
SubCtrlCom::SubCtrlCom(void)
{
}

SubCtrlCom::~SubCtrlCom(void)
{
}
int SubCtrlCom::ReadData(char *pcData,int nDataLen)
{
	m_pCSerialCom->ReadData(pcData,nDataLen);
	return 0;
}
int SubCtrlCom::SendNKJCmd(float fVL,float fVR,int nLTime,int nRTime)
{
	m_pCSerialCom->SendControlCMD(fVL,fVR,nLTime,nRTime);
	return 0;
}

int SubCtrlCom::SendNKJCmd_Rot(float RotDegree, float Rot_Velocity)
{
	m_pCSerialCom->SendControlCMD_Rot(RotDegree,Rot_Velocity);
	return 0;
}

int SubCtrlCom::GetRobotOdo()
{
	m_pCSerialCom->GetRobotOdo();
	return 0;
}
int SubCtrlCom::GetRobotStatus()
{
	m_pCSerialCom->GetRobotStatus();
	return 0;
}

int SubCtrlCom::SendGetRobotStatus()
{
	m_pCSerialCom->SendGetRobotStatus();
	return 0;
}
int SubCtrlCom::SubCtrlComInit(char *pcUSBPort)
{
	//char dev[] ={"/dev/ttyUSB0"};
	m_pCSerialCom=new SerialCom(pcUSBPort,115200,0);
	//char dev[] ={"/dev/ttyUSB0"};
	//m_pCSerialCom=new SerialCom(dev,115200,0);
	return RTN_OK;
}


int SubCtrlCom::SubCtrlComRun()
{
	m_pCSerialCom->Run();

	return RTN_OK;
}


int SubCtrlCom::SubCtrlComStop()
{
	//m_pCSerialCom->close();
	return RTN_OK;
}


int SubCtrlCom::SubCtrlComUninit()
{
	return RTN_OK;
}

int SubCtrlCom::Forward()
{
	return RTN_OK;
}
int SubCtrlCom::Backward()
{
	return RTN_OK;
}
int SubCtrlCom::Break()
{
	return RTN_OK;
}
int SubCtrlCom::Turn()
{
	return RTN_OK;
}

int SubCtrlCom::Odometry(float *pfOdometry)
{
	return RTN_OK;
}

int SubCtrlCom::SendNKJCmd_ClearOdo()
{
	m_pCSerialCom->SendClearOdo();
	return RTN_OK;
}

