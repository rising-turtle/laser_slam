#include "SLAM.h"
#include <string.h>
#include <unistd.h>
//#include <Windows.h>


#include "../ReadSLAMResult/ReadSLAMResult.h"
SLAM::SLAM()
{
}

SLAM::~SLAM()
{
}



int SLAM::Init(SLAM_CallBack stCallBackSet,SLAMParams stParams)
{
	memcpy(&m_stSLAM_CallBack,&stCallBackSet,sizeof(SLAM_CallBack));
	return 0;
}


int SLAM::Run()
{
	int i;
	char *pcData=new char [5*1024];
	ReadSLAMResult CReadSLAMResult;
	CReadSLAMResult.ReadSLAMResultRun("ReadSLAMResult/indesks_psm.txt");
	printf("SLAM  Run!!!!!!!!!!!!!!!!!!\n");
	while(1)
	{
		//cbNetUpload("1234",4);

		while(1)
		{
			float fBNLocation[3];
			m_stSLAM_CallBack.cbBNLocation(fBNLocation);
			printf("x: %f  y:  %f  theta:  %f  \n",fBNLocation[0],fBNLocation[1],fBNLocation[2]);
			for (i=0;i<CReadSLAMResult.m_vctRobotPos.size();i=i+3)
			{
				memcpy(pcData,&CReadSLAMResult.m_vctRobotPos[i],4);
				memcpy(pcData+4,&CReadSLAMResult.m_vctRobotPos[i+1],4);
				memcpy(pcData+8,&CReadSLAMResult.m_vctRobotPos[i+2],4);
				m_cbNetUpload(pcData,12);
				usleep(10);
			}
			sleep(1);
		}


		sleep(1);
	}
	printf("finish!!!!\n");
	return 0;
}


int SLAM::Stop()
{
	return 0;
}


int SLAM::Uninit()
{
	return 0;
}

/*
int SLAM::BNLctAndOdemetry(char *pcData)
{
	return 0;
}



int SLAM::GetCurPos(char *pcPos)
{
	return 0;
}*/
