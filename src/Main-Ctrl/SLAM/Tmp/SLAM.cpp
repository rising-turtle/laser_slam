#include "SLAM.h"
#include <string.h>
#include <unistd.h>
//#include <Windows.h>


#include "../ReadSLAMResult/ReadSLAMResult.h"
SLAM::SLAM(void)
{
}

SLAM::~SLAM(void)
{
}



int SLAM::SLAMInit()
{
	return 0;
}


int SLAM::SLAMRun()
{
	int i;
	char *pcData=new char [5*1024];
	ReadSLAMResult CReadSLAMResult;
	CReadSLAMResult.ReadSLAMResultRun("ReadSLAMResult/indesks_psm.txt");
	while(1)
	{
		//cbNetUpload("1234",4);

		while(1)
		{
			for (i=0;i<CReadSLAMResult.m_vctRobotPos.size();i=i+3)
			{
				memcpy(pcData,&CReadSLAMResult.m_vctRobotPos[i],4);
				memcpy(pcData+4,&CReadSLAMResult.m_vctRobotPos[i+1],4);
				memcpy(pcData+8,&CReadSLAMResult.m_vctRobotPos[i+2],4);
				cbNetUpload(pcData,12);
				usleep(10);
			}
			sleep(1);
		}


		sleep(1);
	}
	printf("finish!!!!\n");
	return 0;
}


int SLAM::SLAMStop()
{
	return 0;
}


int SLAM::SLAMUninit()
{
	return 0;
}


int SLAM::BNLctAndOdemetry(char *pcData)
{
	return 0;
}



int SLAM::GetCurPos(char *pcPos)
{
	return 0;
}
