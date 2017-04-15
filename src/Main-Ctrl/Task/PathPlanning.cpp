#include "PathPlanning.h"
#include <math.h>
#include <vector>
#include <string.h>
using namespace std;
#include "IOA.h"
pthread_mutex_t g_Mutex_InstantView;


PathPlanning::PathPlanning(void)
{
	m_pucInstantView=new unsigned char[IOA_VIEWWIDTH*IOA_VIEWHEIGHT];
	pthread_mutex_init(&g_Mutex_InstantView,NULL);
}

PathPlanning::~PathPlanning(void)
{
	if(m_pucInstantView!=NULL)
	{
		delete [] m_pucInstantView;
		m_pucInstantView=NULL;
	}
	//pthread_mutex_destory(&g_Mutex_InstantView);
}


int PathPlanning::SeedGrowing(int nStartPosIdx,unsigned char *pucLocalMap)
{
	vector<int> vctTmp;
	int nTmpIdx,i,nNewIdx;
	vctTmp.push_back(nStartPosIdx);

	int nMask[4]={-1,1,20,20};

	while (vctTmp.size()!=0)
	{
		nTmpIdx=vctTmp.back();
		vctTmp.pop_back();
		pucLocalMap[nTmpIdx]=255;
		for (i=0;i<4;i++)
		{
			nNewIdx=nTmpIdx+nMask[i];

			if(nNewIdx>=0&&nNewIdx<1000)
			{
				if (pucLocalMap[nNewIdx]==0)
				{
					vctTmp.push_back(nNewIdx);
				}
			}

		}
	}
	return 0;
}


int PathPlanning::MergeGridsBasedOnRobotSize(int nRobotSize,unsigned char *pucLocalMap)
{
	int nMask[25],i,j,idx=0;
	//int *pnTmp=new int[1000];

	//memcpy(pnTmp,pnLocalMap,4000);

	unsigned char *pucTmp=new unsigned char[1000];
	memcpy(pucTmp,pucLocalMap,1000);

	//unsigned char *pucTmpGridMap=new unsigned char[pst2DGridMapData->n2DGridMapDataLen];
	//memcpy(pucTmpGridMap,pst2DGridMapData->puc2DGridMapData,pst2DGridMapData->n2DGridMapDataLen);

	for (i=-2;i<=2;i++)
	{
		for (j=-2;j<=2;j++)
		{
			nMask[idx]=j*20+i;
			idx++;
		}
	}

	for (i=0;i<1000;i++)
	{
		//if(pst2DGridMapData->puc2DGridMapData[i]==200)
		if(pucLocalMap[i]==255)
		{
			for (j=0;j<25;j++)
			{
				idx=i+nMask[j];
				if (idx>=0&&idx<1000)
				{
					//if (pucTmpGridMap[idx]!=200)
					if (pucTmp[idx]!=255)
					{
						//pst2DGridMapData->puc2DGridMapData[i]=255;
						pucLocalMap[i]=200;
						break;
					}
				}
			}
		}
	}


	delete [] pucTmp;
	return 0;
}




int PathPlanning::InstantView(IOA_Pos *pstSICKView_A,IOA_Pos *pstSICKView_B)
{
//	printf("InstantView!!!!\n");
	//4m*10m
	//2m*5m
/*	unsigned char ucLocalMap[1000],*pucLocalMap=NULL;
	pucLocalMap=ucLocalMap;
	int j,i,x,y,nOldX,nOldY,nIdxX,nIdxY,nXStep,nYStep,nTmpX,nTmpY;
	bool bJump=false;

		nOldX=0;
		nOldY=0;
		//memset(ucBMP,0,3000);

		memset(ucLocalMap,IOA_FREEAREA,1000);


		for (i=94;i<450;i=i+2)
		{
			if (fabs(pstSICKView_A[i].fDis-pstSICKView_A[i-1].fDis)>10*pstSICKView_A[i].fDis
			    &&fabs(fabs(pstSICKView_A[i].fDis-pstSICKView_A[i+1].fDis)>10*pstSICKView_A[i].fDis))
			{
				nIdxX=pstSICKView_A[i-1].nIdxX;
				nIdxY=pstSICKView_A[i-1].nIdxY;
			}
			else
			{
				nIdxX=pstSICKView_A[i-1].nIdxX;
				nIdxY=pstSICKView_A[i-1].nIdxY;
			}


			if (nIdxX<-10)nIdxX=0;
			else if(nIdxX>10)nIdxX=19;
			else nIdxX+=10;

			if (nIdxY<0)nIdxY=0;
			else if(nIdxY>49)nIdxY=49;

			pucLocalMap[nIdxY*20+nIdxX]=IOA_OBSTACLE;
			if (nIdxX>=nOldX)
			{
				nXStep=1;
			}
			else nXStep=-1;

			if (nIdxY>=nOldY)
			{
				nYStep=1;
			}
			else nYStep=-1;


			if(nOldX==nIdxX&&nOldY==nIdxY)
			{
				bJump=true;
			}
			else
			{
				bJump=false;
				nTmpX=nOldX;
				nTmpY=nOldY;
			}


			while(!bJump)
			{
				if (nTmpX!=nIdxX)
				{
					nTmpX+=nXStep;
				}
				if (nTmpY!=nIdxY)
				{
					nTmpY+=nYStep;
				}

				if(nTmpX==nIdxX&&nTmpY==nIdxY)
				{
					bJump=true;
				}

				pucLocalMap[nTmpY*20+nTmpX]=IOA_OBSTACLE;

			}
			nOldX=nIdxX;
			nOldY=nIdxY;

		}


	//if(pthread_mutex_trylock(&g_Mutex_InstantView)==0)
	{
		memcpy(m_pucInstantView,ucLocalMap,1000);
	//	pthread_mutex_unlock(&g_Mutex_InstantView);
	}*/
	return 0;
}
int PathPlanning::DodgePath(Pos_f *pstDest,vector<Point_f> &vctPath)
{
	return 0;
}

/*int PathPlanning::DodgePath(Pos_f *pstDest,vector<Point_f> &vctPath)
{
	int nCurPosInMapX,nCurPosInMapY;
	float fCurPos[3];

	m_fDest[0]=pstDest->stPoint.fX;
	m_fDest[1]=pstDest->stPoint.fY;


	IOA::GetCurPose(fCurPos);
	memcpy(m_fPosBefIOA,fCurPos,12);

	MileStoneSlct(m_pucInstantView,IOA_VIEWHEIGHT,vctPath);
	return 0;
}
*/

int PathPlanning::GetCurPose(float *pfPos)
{
	return 0;
}

int PathPlanning::GetPosBeforeIOA(float *pfPos)
{
	memcpy(pfPos,m_fPosBefIOA,12);
	return 0;
}

int PathPlanning::GetMileStonePos(float *pfPos)
{
	memcpy(pfPos,m_fDest,12);
	return 0;
}
int PathPlanning::Back2OriPath()
{
	float fCurPos[3],fPosBefIOA[3];
	float fMat[9],fMileStonePos[3],fPosInCurOrd[3];
	float fI;
	IOA::GetCurPose(fCurPos);
	GetPosBeforeIOA(fPosBefIOA);
	GetTransMat(fPosBefIOA,fCurPos,fMat);
	GetMileStonePos(fMileStonePos);

	int nIdxX,nIdxY;

	float fTmp[3],fTmp2[3];
	fTmp[0]=0;
	fTmp[2]=1;
	float fDes=MIN(fMileStonePos[1],IOA_Y_RANGE);

	if(fMileStonePos[1]-fCurPos[1]<2)
	{
		//gai bian wan dao
	}
	else
	{
		for(fI=(fDes-fCurPos[1])/2;fI<fDes;fI=fI+0.1)
			{
				fTmp[1]=fI;
				MulVec(fMat,fTmp,fTmp2);
				nIdxX=fTmp2[0]/0.1;
				nIdxY=fTmp2[1]/0.1;

				if(m_pucInstantView[nIdxY*IOA_VIEWWIDTH+nIdxX-10]==IOA_FREEAREA)
				{
					break;
				}
			}


		//calculate control cmd
		//	Point_f stPoint;
		//	vector<Point_f> vctPath;
		//	m_CTrajectory.CalWheelVel(vctPath);
			return 0;
	}



}


int PathPlanning::GetTransMat(float *pfOriPos,float *pfCurPos,float *pfMat)
{
	float fAngle=pfOriPos[2]-pfCurPos[2];
	float fX=pfOriPos[0]-pfCurPos[0];
	float fY=pfOriPos[1]-pfCurPos[1];

	float fMat[9];

	fMat[0]=cos(fAngle);
	fMat[1]=-sin(fAngle);
	fMat[3]=fX;
	fMat[4]=-sin(fAngle);
	fMat[5]=cos(fAngle);
	fMat[6]=fY;
	fMat[7]=0;
	fMat[8]=0;
	fMat[9]=1;

	return Inverse(fMat,pfMat);
}


int PathPlanning::MileStoneSlct(unsigned char *pucInstantView,
		int nArrayLen,
		vector<Point_f> &vctPath)
{
	int i,j,k,nX=IOA_VIEWWIDTH-1,nY=IOA_VIEWHEIGHT-1,nCount=0,nXAcc=0;;
	int nMidX,nMidY,nEndY;
	float fK;

	printf("MileStoneSlct!!!!\n");
	nArrayLen=1000;

	for(j=nY;j>=0;j--)
	{
		for(i=nX;i>=0;i--)
		{
			nArrayLen--;
			if(pucInstantView[nArrayLen]==IOA_FREEAREA)
			{
				k=i;
				for(k=i;k>=0;k--)
				{
					if(pucInstantView[k]==IOA_FREEAREA)
					{
						nXAcc+=k;
						nCount++;
					}
					else
					{
						nMidY=j;
						break;
					}
				}
				j=-1;
				i=-1;
			}
		}
	}
	nMidX=nXAcc/nCount;
	printf("MileStoneSlct222222:%d  ,  %d   ,%d!!!!\n",
			nMidX,nXAcc,nCount);
	int nStartX=10,nStartY=10,nEndX=nMidX;
	if(nMidY>10)
	{
		nEndY=(nMidY>>1);
	}
	else
	{
		nEndY=nMidY;
	}

	fK=(float)(nEndY-nStartY)/(nEndX-nStartX);

	bool bJump=false;

	printf("MileStoneSlct33333333333  %f!!!!\n",fK);
	while(!bJump)
	{
		bool bMeetObstacle=false;
		if(nStartX>nEndX)
		{
			for(i=nEndX;i<=nStartX;i++)
			{
				nY=(float)fK*i+0.5;
				if(pucInstantView[nY*IOA_VIEWWIDTH+i]!=IOA_FREEAREA)
				{
					bMeetObstacle=true;
					break;
				}
			}
		}
		else
		{
			for(i=nStartX;i<=nEndX;i++)
			{
				nY=(float)fK*i+0.5;
				if(pucInstantView[nY*IOA_VIEWWIDTH+i]!=IOA_FREEAREA)
				{
					bMeetObstacle=true;
					break;
				}
			}
		}
		printf("MileStoneSlct55555555555!!!!\n");
		if(bMeetObstacle)
		{
			nEndY--;
			fK=(float)(nEndY-nStartY)/(nEndX-nStartX);
		}
		else
		{
			bJump=true;
		}
	}
	printf("MileStoneSlct44444444444!!!!\n");

	//calculate control cmd
	//	Point_f stPoint;
	//	vector<Point_f> vctPath;
	//	m_CTrajectory.CalWheelVel(vctPath);
/*	Point_f stPoint;
	vector<Point_f> vctPath;
	stPoint.fX=0;
	stPoint.fY=0;
	vctPath.push_back(stPoint);
	stPoint.fX=0;
	stPoint.fY=5*10;
	vctPath.push_back(stPoint);
	stPoint.fX=(nEndX-10)*10;
	stPoint.fY=nEndY*10;
	vctPath.push_back(stPoint);
	stPoint.fX=(nMidX-10)*10;
	stPoint.fY=nMidY*10;
	vctPath.push_back(stPoint);
	m_CTrajectory.CalWheelVel(vctPath);*/




	Point_f stPoint;
	stPoint.fX=0;
	stPoint.fY=0;

	vctPath.push_back(stPoint);
	stPoint.fX=0;
	stPoint.fY=5*0.1;

	vctPath.push_back(stPoint);
	stPoint.fX=(nEndX-10)*0.1;
	stPoint.fY=nEndY*0.1;

	printf("EndX:%f  EndY:%f  \n",stPoint.fX,stPoint.fY);
	vctPath.push_back(stPoint);
	stPoint.fX=(nMidX-10)*0.1;
	stPoint.fY=nMidY*0.1;
	printf("nMidX:%f  nMidY:%f  \n",stPoint.fX,stPoint.fY);
	vctPath.push_back(stPoint);


	sleep(10000);
	return 0;
}

int PathPlanning::Inverse(float *pfMat,float *pfInv)
{
	float fDet=pfMat[0]*pfMat[4]*pfMat[8]+pfMat[1]*pfMat[5]*pfMat[6]+pfMat[2]*pfMat[3]*pfMat[7]
				-pfMat[2]*pfMat[4]*pfMat[6]-pfMat[1]*pfMat[3]*pfMat[8]-pfMat[0]*pfMat[5]*pfMat[7];
	if (fDet==0)
	{
		return -1;
	}
	else
	{
		fDet=1/fDet;

		float fTmp;
		fTmp=pfMat[4]*pfMat[8]-pfMat[5]*pfMat[7];
		pfInv[0]=(pfMat[4]*pfMat[8]-pfMat[5]*pfMat[7])*fDet;
		pfInv[3]=-(pfMat[3]*pfMat[8]-pfMat[5]*pfMat[6])*fDet;
		pfInv[6]=(pfMat[3]*pfMat[7]-pfMat[4]*pfMat[6])*fDet;
		pfInv[1]=-(pfMat[1]*pfMat[8]-pfMat[2]*pfMat[7])*fDet;
		pfInv[4]=(pfMat[0]*pfMat[8]-pfMat[2]*pfMat[6])*fDet;
		pfInv[7]=-(pfMat[0]*pfMat[7]-pfMat[1]*pfMat[6])*fDet;
		pfInv[2]=(pfMat[1]*pfMat[5]-pfMat[2]*pfMat[4])*fDet;
		pfInv[5]=-(pfMat[0]*pfMat[5]-pfMat[2]*pfMat[3])*fDet;
		pfInv[8]=(pfMat[0]*pfMat[4]-pfMat[1]*pfMat[3])*fDet;

		return 0;

	}
}

int PathPlanning::MulMat(float *pfMatA,float *pfMatB,float *pfMatC)
{
	pfMatC[0]=pfMatA[0]*pfMatB[0]+pfMatA[1]*pfMatB[3]+pfMatA[2]*pfMatB[6];
	pfMatC[1]=pfMatA[0]*pfMatB[1]+pfMatA[1]*pfMatB[4]+pfMatA[2]*pfMatB[7];
	pfMatC[2]=pfMatA[0]*pfMatB[2]+pfMatA[1]*pfMatB[5]+pfMatA[2]*pfMatB[8];
	pfMatC[3]=pfMatA[3]*pfMatB[0]+pfMatA[4]*pfMatB[3]+pfMatA[5]*pfMatB[6];
	pfMatC[4]=pfMatA[3]*pfMatB[1]+pfMatA[4]*pfMatB[4]+pfMatA[5]*pfMatB[7];
	pfMatC[5]=pfMatA[3]*pfMatB[2]+pfMatA[4]*pfMatB[5]+pfMatA[5]*pfMatB[8];
	pfMatC[6]=pfMatA[6]*pfMatB[0]+pfMatA[7]*pfMatB[3]+pfMatA[8]*pfMatB[6];
	pfMatC[7]=pfMatA[6]*pfMatB[1]+pfMatA[7]*pfMatB[4]+pfMatA[8]*pfMatB[7];
	pfMatC[8]=pfMatA[6]*pfMatB[2]+pfMatA[7]*pfMatB[5]+pfMatA[8]*pfMatB[8];
	return 0;
}

int PathPlanning::MulVec(float *pfMat,float *pfVec,float *pfVecRslt)
{
	pfVecRslt[0]=pfMat[0]*pfVec[0]+pfMat[1]*pfVec[1]+pfMat[2]*pfVec[2];
	pfVecRslt[1]=pfMat[3]*pfVec[0]+pfMat[4]*pfVec[1]+pfMat[5]*pfVec[2];
	pfVecRslt[2]=pfMat[6]*pfVec[0]+pfMat[7]*pfVec[1]+pfMat[8]*pfVec[2];
	return 0;
}


/*void* PathPlanning::Path2Destination(float fStartX,float fStartY,float fEndX,float fEndY)
{



	fStartX+=OFFSET_ORI_2DGRIDMAP_X;
	fStartY+=OFFSET_ORI_2DGRIDMAP_Y;

	fEndX+=OFFSET_ORI_2DGRIDMAP_X;
	fEndY+=OFFSET_ORI_2DGRIDMAP_Y;


	nStartPosIdx=(int)(fStartY*pst2DGridMapData->fMapScale/pst2DGridMapData->fGridResolution)*
							pst2DGridMapData->n2DGridMapW+
							(int)(fStartX*pst2DGridMapData->fMapScale/pst2DGridMapData->fGridResolution);

	nEndX=fEndX*pst2DGridMapData->fMapScale/pst2DGridMapData->fGridResolution;
	nEndY=fEndY*pst2DGridMapData->fMapScale/pst2DGridMapData->fGridResolution;
	nEndPosIdx=CheckDest(nEndX,nEndY,pst2DGridMapData);

	printf("fStartX :%f  ,fStartY:  %f ,fEndX:%d  , fEndY:%d \n",
			fStartX*pst2DGridMapData->fMapScale/pst2DGridMapData->fGridResolution,
			fStartY*pst2DGridMapData->fMapScale/pst2DGridMapData->fGridResolution,nEndX,nEndY);
	if(nEndPosIdx!=-1)
	{
		vector<float> vctPath;
		char cRobotMask=(1<<(stDataCarrier.nRobotIdx));
		nRtn=m_pCPathPlanning->GeneratePath(nStartPosIdx,nEndPosIdx,pst2DGridMapData,vctPath,cRobotMask);
		if(nRtn==0)
		{


			vector<vector<int> > vctTmp22;
			vector<float> vctNewPath;
			 MileStoneReduce2nd(vctPath,
					 vctNewPath,
					 vctTmp22,
					 pst2DGridMapData);


			 vctPath.clear();
			 vctPath=vctNewPath;

			 m_pCPathPlanning->PackSendData(pcDataSpace+30,stDataCarrier.nRobotIdx,
			 				1,vctPath,pst2DGridMapData->n2DGridMapW,nDataLen);



			printf("stDataCarrier.nRobotIdx:  %d  \n",stDataCarrier.nRobotIdx);
			if (stDataCarrier.nRobotIdx!=-1)
			{
				int nSendDataLen=(vctPath.size()<<2)+4;
				char *pcPath=new char[nSendDataLen];
				char *pcTmpPath=pcPath;
				int nCmd=NEW_TASK_PATH;
				memcpy(pcPath,&nCmd,4);
				pcTmpPath+=4;
				for (i=0;i<vctPath.size();i++)
				{
					memcpy(pcTmpPath,&vctPath[i],4);
					pcTmpPath+=4;
				}

				m_stCallBackSet.cbPortB(nSendDataLen,pcPath,m_pstRegisteredRobotInfo[stDataCarrier.nRobotIdx].nSockB);

				delete [] pcPath;
				pcPath=NULL;
				pcTmpPath=NULL;
			}

			//send to webpage
			printf("nThreadNum  %d  \n",stDataCarrier.nThreadNum);
			pcDataSpace[0]=1;
			memcpy(pcDataSpace+1,&nDataLen,4);
			memcpy(pcDataSpace+18,stDataCarrier.pcCmd,2);
			m_stCallBackSet.cbSendData(stDataCarrier.nThreadNum,pcDataSpace);
		}
	}

	delete [] stDataCarrier.pfPath;


	return 0;
}*/

