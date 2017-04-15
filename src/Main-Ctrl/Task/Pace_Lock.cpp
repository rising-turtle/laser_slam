#include "Pace_Lock.h"
#include <math.h>
#include <string.h>
#include <stdlib.h> 
#include <stdio.h>
Pace_Lock::Pace_Lock(void)
{
}

Pace_Lock::~Pace_Lock(void)
{
}
int Pace_Lock::Pace_LockRun(float *pfPath,int nPathLen,char *pcPaceLockData,int &nDataLen)
{
	int i;
	int nInflectionPointsNum=0;
	int *pnInflectionPointsIdxArray=new int [nPathLen];
	printf("path len :  %d\n",nPathLen);
	GenerateInflectionPoint(pfPath,0,nPathLen/2-1,5,9999999,pnInflectionPointsIdxArray,nInflectionPointsNum);
	pnInflectionPointsIdxArray[nInflectionPointsNum++]=0;
	pnInflectionPointsIdxArray[nInflectionPointsNum++]=nPathLen/2-1;
	bubble_sort(pnInflectionPointsIdxArray,nInflectionPointsNum);

	char *pcDataHead=pcPaceLockData;
	char cData[1000];
	unsigned int uiDatalen=0;
	for (i=0;i<nInflectionPointsNum;i++)
	{
	

		*pcDataHead='$';
		pcDataHead++;
		memset(cData,0,10);
		sprintf(cData,"%d",pnInflectionPointsIdxArray[i]);
		//itoa(pnInflectionPointsIdxArray[i],cData,10);
		memcpy(pcDataHead,cData,strlen(cData));
		pcDataHead+=strlen(cData);
		uiDatalen+=(strlen(cData)+1);
	}
	nDataLen=uiDatalen;
	nInflectionPointsNum=0;
	delete [] pnInflectionPointsIdxArray;
	return 0;

}

int Pace_Lock::Pace_LockRun_Dest(float *pfPath,int nPathLen,vector<float> &vctPath)
{
	int i;
	float y,x;
	int nInflectionPointsNum=0;
	int *pnInflectionPointsIdxArray=new int [nPathLen];
	printf("path len :  %d\n",nPathLen);
	GenerateInflectionPoint(pfPath,0,nPathLen/2-1,5,9999999,pnInflectionPointsIdxArray,nInflectionPointsNum);
	printf("pace lock 444444");
	pnInflectionPointsIdxArray[nInflectionPointsNum++]=0;
	pnInflectionPointsIdxArray[nInflectionPointsNum++]=nPathLen/2-1;
	printf("pace lock 222222");
	bubble_sort(pnInflectionPointsIdxArray,nInflectionPointsNum);
	

	printf("pace lock 111111");
	//char *pcDataHead=pcPaceLockData;
	char cData[1000];
	unsigned int uiDatalen=0;
	for (i=0;i<nInflectionPointsNum;i++)
	{
		x=pfPath[pnInflectionPointsIdxArray[i]*2];
		y=pfPath[pnInflectionPointsIdxArray[i]*2+1];

		vctPath.push_back(x);
		vctPath.push_back(y);

	/*	*pcDataHead='$';
		pcDataHead++;
		memset(cData,0,10);
		itoa(x,cData,10);
		memcpy(pcDataHead,cData,strlen(cData));
		pcDataHead+=strlen(cData);
		uiDatalen+=(strlen(cData)+1);

		*pcDataHead='$';
		pcDataHead++;
		memset(cData,0,10);
		itoa(y,cData,10);
		memcpy(pcDataHead,cData,strlen(cData));
		pcDataHead+=strlen(cData);
		uiDatalen+=(strlen(cData)+1);*/
	}
	//nDataLen=uiDatalen;
	nInflectionPointsNum=0;

	printf("pace lock 22222");
	delete [] pnInflectionPointsIdxArray;
	return 0;

}
int Pace_Lock::Pace_LockRun(float *pfPath,int nPathLen,float *pfPaceLock,int &nPaceLockLen)
{
	int i;
	static int nInflectionPointsNum=0;
	static int *pnInflectionPointsIdxArray=new int [nPathLen];
	GenerateInflectionPoint(pfPath,0,nPathLen/2-1,0.3,0.5,pnInflectionPointsIdxArray,nInflectionPointsNum);
	pnInflectionPointsIdxArray[nInflectionPointsNum++]=0;
	pnInflectionPointsIdxArray[nInflectionPointsNum++]=nPathLen/2-1;
	bubble_sort(pnInflectionPointsIdxArray,nInflectionPointsNum);
	nPaceLockLen=nInflectionPointsNum*2;
	for (i=0;i<nInflectionPointsNum;i++)
	{
		pfPaceLock[i*2]=pfPath[pnInflectionPointsIdxArray[i]*2];
		pfPaceLock[i*2+1]=pfPath[pnInflectionPointsIdxArray[i]*2+1];
	}

	delete [] pnInflectionPointsIdxArray;
	return 0;
}

void Pace_Lock::bubble_sort(int *x, int n)
{
	int j, k=0, h,t;
	for (h=n-1,k=h; h>0; h--)
	{ 
		for (j=0, k=0; j<h; j++)
		{
			if (*(x+j) > *(x+j+1))
			{ 
				t = *(x+j);
				*(x+j) = *(x+j+1);
				*(x+j+1) = t;
				k = j;
			}
		}
	}
} 

float Pace_Lock::TwoPointsDis(float x1,float y1,float x2,float y2)
{
	float x,y,fTmp;
	x=x2-x1;
	y=y2-y1;
	fTmp=sqrt((double)x*x+y*y);
	return fTmp;
}

int Pace_Lock::GenerateInflectionPoint(float *fPointArray,int nStartIdx,
									   int nEndIdx,float fInflexionThres,float fMaxOneStep,
									   int *pnInflectionPointsIdxArray,int &nInflectionPointsNum)
{
	float fMaxVal;
	int nMaxValIdx;

	float fVal1,fVal2;;

	fVal1=LookForInflectionPoint(fPointArray,nStartIdx,nEndIdx,&nMaxValIdx,&fMaxVal);
	if (nMaxValIdx!=0)
	{
		fVal2=TwoPointsDis(fPointArray[nStartIdx*2],fPointArray[nStartIdx*2+1],fPointArray[nEndIdx*2],fPointArray[nEndIdx*2+1]);
	}
	else
	{
		fVal2=0;
	}

	if(fVal1<fInflexionThres
		&&fVal2<fMaxOneStep)
	{
		return 0;
	}
	else
	{
		pnInflectionPointsIdxArray[nInflectionPointsNum]=nMaxValIdx;
		nInflectionPointsNum++;
		GenerateInflectionPoint(fPointArray,nStartIdx,nMaxValIdx,fInflexionThres,fMaxOneStep,pnInflectionPointsIdxArray,nInflectionPointsNum);
		GenerateInflectionPoint(fPointArray,nMaxValIdx,nEndIdx,fInflexionThres,fMaxOneStep,pnInflectionPointsIdxArray,nInflectionPointsNum);
	}
	return 0;
}

float Pace_Lock::LookForInflectionPoint(float *fPointArray,int nStartIdx,int nEndIdx,int *pnMaxValIdx,float *pfMaxVal)
{
	float k,A,B,C,D;
	float fMaxVal,fVal;
	int nMaxValIdx=0,i,nMidPointNum,nCurIdx,j,m,nMidIdx;
	if (nStartIdx>nEndIdx)
	{
		int ddddd=1000;
	}
	if (fabs(fPointArray[nStartIdx*2+1]-fPointArray[nEndIdx*2+1])<0.04)
	{
		fMaxVal=0;


		nMidPointNum=(nEndIdx-nStartIdx);
		if (nMidPointNum==3)
		{
			int gggg=10;
		}
		nMidIdx=nStartIdx+nMidPointNum/2;
		j=0;
		m=-1;
		i=0;
		nCurIdx=nMidIdx;
		nMidPointNum-=2;
		while (i<nMidPointNum)
		{
			if (nCurIdx!=nStartIdx&&nCurIdx!=nEndIdx)
			{
				fVal=fabs(fPointArray[nCurIdx*2+1]-fPointArray[nStartIdx*2+1]);
				if(fVal>=fMaxVal)
				{
					fMaxVal=fVal;
					nMaxValIdx=nCurIdx;
				}
				m*=-1;
				j=(i/2)*m;
				nCurIdx=j+nMidIdx;

			}
			i++;
		}
	}
	else
	{
		k=(fPointArray[nStartIdx*2]-fPointArray[nEndIdx*2])/(fPointArray[nStartIdx*2+1]-fPointArray[nEndIdx*2+1]);
		A=1.0;
		B=-k;
		C=k*fPointArray[nStartIdx*2+1]-fPointArray[nStartIdx*2];
		D=sqrt(A*A+B*B);
		fMaxVal=0;

		nMidPointNum=(nEndIdx-nStartIdx);
		if (nMidPointNum==3)
		{
			int gggg=10;
		}
		nMidIdx=nStartIdx+nMidPointNum/2;
		j=0;
		m=-1;
		i=0;
		nCurIdx=nMidIdx;
		nMidPointNum-=2;
		while (i<nMidPointNum)
		{
			if (nCurIdx!=nStartIdx&&nCurIdx!=nEndIdx)
			{
				fVal=fabs(A*fPointArray[nCurIdx*2]+B*fPointArray[nCurIdx*2+1]+C)/D;
				if(fVal>=fMaxVal)
				{
					fMaxVal=fVal;
					nMaxValIdx=nCurIdx;
				}
				m*=-1;
				j=(i/2)*m;
				nCurIdx=j+nMidIdx;

			}
			i++;
		}
	}

	*pnMaxValIdx=nMaxValIdx;
	*pfMaxVal=fMaxVal;


	return fMaxVal;
}
