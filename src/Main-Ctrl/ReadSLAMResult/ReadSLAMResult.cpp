#include "ReadSLAMResult.h"
#include <math.h>
#include "string.h"
ReadSLAMResult::ReadSLAMResult(void)
{
}

ReadSLAMResult::~ReadSLAMResult(void)
{
}

int ReadSLAMResult::ReadSLAMResultInit()
{
	int i;
//	float fStart=-3.1415926*45/180;

	float fStart=3.1415926*225/180;
	float fSetp=0.5/180*3.1415926;
	for (i=0;i<541;i++)
	{
		m_fAngle[i][0]=cos(fStart);
		m_fAngle[i][1]=sin(fStart);
		fStart-=fSetp;
	}
	return 0;
}
int ReadSLAMResult::ReadSLAMResultRun(const char *pcPath)
{
	ReadSLAMResultInit();
	ReadTraFile(pcPath);
	ReadCPFile("ReadSLAMResult/indesks.txt");
	TranslateCP();
	return 0;
}
int ReadSLAMResult::TranslateCP()
{
	int i,j;
	float x,y,x2,y2;
	vector<float> vctLocal;
	for (i=0;i<m_vctPC.size();i++)
	{
		for (j=0;j<m_vctPC[i].size();j++)
		{
			if(m_vctPC[i][j]<10&&m_vctPC[i][j]>0.1)
			{
				x=m_vctPC[i][j]*m_fAngle[j][0];
				y=m_vctPC[i][j]*m_fAngle[j][1];
			}
			else
			{
				x=0;
				y=0;
			}
			
			

			x2=cos(m_vctRobotPos[i*3+2])*x-sin(m_vctRobotPos[i*3+2])*y+m_vctRobotPos[i*3]/100.0;
			y2=sin(m_vctRobotPos[i*3+2])*x+cos(m_vctRobotPos[i*3+2])*y+m_vctRobotPos[i*3+1]/100.0;

			vctLocal.push_back(x2);
			vctLocal.push_back(y2);
		}
		m_vctPCLocal.push_back(vctLocal);
		vctLocal.clear();
	}
	return 0;
}

bool ReadSLAMResult::ReadCPFile(const char *pcPath)
{
	int i,j;
	FILE * pFile;   
	size_t result;
	char *pcFileBuff1,*pcFileBuff2;
	int nLen1,nLen2;
	char cTmp[100];
	bool bJump;
	float fTmp;
	//* 若要一个byte不漏地读入整个文件，只能采用二进制方式打开 
	pFile = fopen (pcPath, "rb" );  
	if (pFile==NULL)  
	{  
		return false;
	}  

	//* 获取文件大小 
	fseek (pFile , 0 , SEEK_END);  
	nLen1 = ftell (pFile);  //tell the pointer drift number
	rewind (pFile);  //re-point to buff head
	//fseek (pFile , 0 , SEEK_SET);  



	//* 分配内存存储整个文件 
	//pcFileBuff1 = (char*) malloc (sizeof(char)*nLen1);  
	//pcFileBuff2 = (char*) malloc (sizeof(char)*nLen1); 
	
	pcFileBuff1=new char[sizeof(char)*nLen1];
	pcFileBuff2=new char[sizeof(char)*nLen1];
	
	memset(pcFileBuff1,0,sizeof(char)*nLen1);
	if (pcFileBuff1 == NULL)  
	{  
		return false;  
	}  

	//* 将文件拷贝到buffer中 
	result = fread (pcFileBuff1,1,nLen1,pFile); 


	if (result != nLen1)  
	{  
		return false;  
	}  
	nLen2=0;
	j=0;

	int nCount1=0;
	vector<float> vctPC;
	for (i=0;i<nLen1;i++)
	{
		if (pcFileBuff1[i]==0x0D)
		{	
			m_vctPC.push_back(vctPC);
			vctPC.clear();
			nCount1=0;
			i++;
		}
		else
		{
			if (pcFileBuff1[i]==0x9)
			{
				nCount1++;
				if (j!=0)
				{
					fTmp=atof(&cTmp[0]);
					vctPC.push_back(fTmp);
				}
				j=0;
				memset(cTmp,0,100);
			}
			else
			{
				if (nCount1>=9)
				{
					cTmp[j++]=pcFileBuff1[i];
				}
			}
		}
	}


	//free(pcFileBuff1);
	delete [] pcFileBuff1;
	pcFileBuff1=NULL;

	//free(pcFileBuff2);
	delete [] pcFileBuff2;
	pcFileBuff1=NULL;

	//* 结束演示，关闭文件并释放内存   
	fclose (pFile);  
	return true;
}
bool ReadSLAMResult::ReadTraFile(const char *pcPath)
{
	int i,j;
	FILE * pFile;   
	size_t result;
	char *pcFileBuff1,*pcFileBuff2;
	int nLen1,nLen2;
	char cTmp[100];
	bool bJump;
	float fTmp;
	//* 若要一个byte不漏地读入整个文件，只能采用二进制方式打开 
	pFile = fopen (pcPath, "rb" );  
	if (pFile==NULL)  
	{  
		return false;
	}  

	//* 获取文件大小 
	fseek (pFile , 0 , SEEK_END);  
	nLen1 = ftell (pFile);  //tell the pointer drift number
	rewind (pFile);  //re-point to buff head
	//fseek (pFile , 0 , SEEK_SET);  



	//* 分配内存存储整个文件 
	//pcFileBuff1 = (char*) malloc (sizeof(char)*nLen1);  
	//pcFileBuff2 = (char*) malloc (sizeof(char)*nLen1); 

	pcFileBuff1 =new char[sizeof(char)*nLen1];
	pcFileBuff2 =new char[sizeof(char)*nLen1];
	memset(pcFileBuff1,0,sizeof(char)*nLen1);
	if (pcFileBuff1 == NULL)  
	{  
		return false;  
	}  

	//* 将文件拷贝到buffer中 
	result = fread (pcFileBuff1,1,nLen1,pFile); 


	if (result != nLen1)  
	{  
		return false;  
	}  

	nLen2=0;
	j=0;
	for (i=0;i<nLen1;i++)
	{

		if (pcFileBuff1[i]!=0x9&&pcFileBuff1[i]!=0x0D&&pcFileBuff1[i]!=0x0A&&pcFileBuff1[i]!=32)
		{
			cTmp[j++]=pcFileBuff1[i];
		}
		else
		{
			if (j!=0)
			{
				if (cTmp[0]==0x2d)
				{
					fTmp=atof(&cTmp[1]);
					m_vctRobotPos.push_back(-fTmp);
				}
				else
				{
					fTmp=atof(cTmp);
					m_vctRobotPos.push_back(fTmp);
				}
			}
			j=0;
			memset(cTmp,0,100);
		}
	}


	//free(pcFileBuff1);
	delete [] pcFileBuff1;
	pcFileBuff1=NULL;

	//free(pcFileBuff2);
	delete [] pcFileBuff2;
	pcFileBuff1=NULL;

	//* 结束演示，关闭文件并释放内存   
	fclose (pFile);  
	return true;
}
