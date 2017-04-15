#include "Trajectory.h"
#include <time.h>
#include <string.h>
#include <stdio.h>

Trajectory::Trajectory(void)
{

}

Trajectory::~Trajectory(void)
{
}


/*
int Trajectory::MatPlusVec(float *fA, float *fB, float *fR)
{
fR[0]=fA[0]+fB[0];
fR[1]=fA[1]+fB[1];
return 0;
}

int Trajectory::MatMinusVec(float *fA, float *fB, float *fR)
{
fR[0]=fA[0]-fB[0];
fR[1]=fA[1]-fB[1];
return 0;
}
int Trajectory::MatMul(float *fA, float *fB, float *fR)
{
fR[0]=fA[0]*fB[0]+fA[1]*fB[2];
fR[1]=fA[0]*fB[1]+fA[1]*fB[3];
fR[2]=fA[2]*fB[0]+fA[3]*fB[2];
fR[3]=fA[2]*fB[1]+fA[3]*fB[3];
return 0;
}	

int Trajectory::MatMulVec(float *fA,float *fB,float *fR)
{
fR[0]=fA[0]*fB[0]+fA[1]*fB[1];
fR[1]=fA[2]*fB[0]+fA[3]*fB[1];
return 0;
}
int Trajectory::GenTraj()
{
//�����켣
//float fQ0[3]={2.61799,1,3};
//float fQT[3]={3.1415926/2,0,0};


//ת��켣
float fQ0[3]={3.1415926/2,0,0};
float fQT[3]={3.1415926,-8,4};

//�˶�ʱ�� 3s
//����ʱ���� 100ms

float fZ0[2],fZT[2];

float fT=3;
float fB0=(fQT[0]-fQ0[0])/fT;

float fM[4];
float fD[4];
float fInvM[4];
float fTmp[2],fTmp1[2],fC[2];

float fDet;
float Z1[31],Z2[31],Z3[31],X[31],Y[31],V[31],Theta[31],FiL[31],FiR[31];
int j,i;
float fTheta[31];
float fU2;

fM[0]=fT;
fM[1]=fT*fT/2;
fM[2]=fB0*fT*fT/2;
fM[3]=fB0*fT*fT*fT/6;

fDet=fM[0]*fM[3]-fM[1]*fM[2];
fInvM[0]=fM[3]/fDet;
fInvM[1]=-fM[1]/fDet;
fInvM[2]=-fM[2]/fDet;
fInvM[3]=fM[0]/fDet;
fD[0]=1;
fD[1]=0;
fD[2]=fB0*fT;
fD[3]=1;

fZ0[0]=cos(fQ0[0])*fQ0[1]+sin(fQ0[0])*fQ0[2];
fZ0[1]=sin(fQ0[0])*fQ0[1]-cos(fQ0[0])*fQ0[2];

fZT[0]=cos(fQT[0])*fQT[1]+sin(fQT[0])*fQT[2];
fZT[1]=sin(fQT[0])*fQT[1]-cos(fQT[0])*fQT[2];

MatMul(fD,fZ0,fTmp);
MatMinusVec(fZT,fTmp,fTmp1);
MatMulVec(fInvM,fTmp1,fC);

j=0;

float t;
for (t=0;t<=3;t=t+0.1)
{
Z1[j]=fB0*t+fQ0[0];
Z2[j]=fC[0]*t+0.5*fC[1]*t*t+fZ0[0];
Z3[j]=fB0*(fC[0]*t*t/2+fC[1]*t*t*t/6)+fB0*fZ0[0]*t+fZ0[1];

X[j]=cos(Z1[j])*Z2[j]+sin(Z1[j])*Z3[j];
Y[j]=sin(Z1[j])*Z2[j]-cos(Z1[j])*Z3[j];

j++;
}

V[0]=0;
for (i=1;i<=30;i++)
{
Theta[i]=fB0/10;
V[i]=fC[0]+fC[1]*i/10+Z3[i]*Theta[i];
}

FiR[0]=0;
FiL[0]=0;
for (i=1;i<=30;i++)
{
FiR[i]=(V[i]+Theta[i])/2;
FiL[i]=(V[i]-Theta[i])/2;
}
return 0;
}
*/

/*
int main()
{
Trajectory test;
vector<Point_f> vctPath;
Point_f stPT;

stPT.fX=0;
stPT.fY=0;
vctPath.push_back(stPT);

stPT.fX=12;
stPT.fY=11;
vctPath.push_back(stPT);


stPT.fX=21;
stPT.fY=5;
vctPath.push_back(stPT);


stPT.fX=15;
stPT.fY=-3.5;
vctPath.push_back(stPT);

stPT.fX=24;
stPT.fY=-11;
vctPath.push_back(stPT);


stPT.fX=17.3;
stPT.fY=-23.5;
vctPath.push_back(stPT);


stPT.fX=10;
stPT.fY=-17;
vctPath.push_back(stPT);

stPT.fX=-8;
stPT.fY=-17;
vctPath.push_back(stPT);

stPT.fX=-20;
stPT.fY=16;
vctPath.push_back(stPT);

stPT.fX=-19;
stPT.fY=17;
vctPath.push_back(stPT);

stPT.fX=-20;
stPT.fY=16;
vctPath.push_back(stPT);

stPT.fX=-19;
stPT.fY=17;
vctPath.push_back(stPT);



clock_t start, finish;
start = clock();
printf("start time  :%d  \n",clock());
test.CalWheelVel(vctPath);
printf("end time  :%d  \n",clock());
printf("cost time  :%d  \n",clock()-start);
return 0;
}
*/

int Trajectory::SegmentBlend(Point_f *stPt,vector<float> &vctWL,vector<float> &vctWR)
{

	int i,nSlice=100;
	float fRotX,fRotY,fTheta,fA0,fA1,fA2,fA3;
	float fBlendT1=(float)0.1,fBlendT2=(float)0.9;
	float fT=2,fTStep=fT/nSlice,fK1,fK2,fB2,fXL;
	float fX0,fX1,fY0,fY1,fXDiff,fMid;

	Point_f stNewPt[3];
	fRotX=stPt[2].fX-stPt[0].fX;
	fRotY=stPt[2].fY-stPt[0].fY;

	if (fRotY!=0)
	{
		if (fRotX==0)
		{
			fTheta=(float)1.5707963;
		}
		else
		{
			fTheta=-atan2(fRotY,fRotX);
		}


		stNewPt[0].fX=0;
		stNewPt[0].fY=0;


		for (i=1;i<3;i++)
		{
			stNewPt[i].fX=cos(fTheta)*(stPt[i].fX-stPt[0].fX)
				-sin(fTheta)*(stPt[i].fY-stPt[0].fY);
			stNewPt[i].fY=sin(fTheta)*(stPt[i].fX-stPt[0].fX)
				+cos(fTheta)*(stPt[i].fY-stPt[0].fY);
		}
	}
	else
	{
		memcpy(&stNewPt,stPt,sizeof(Point_f)*3);
	}

	fK1=(stNewPt[0].fY-stNewPt[1].fY)/(stNewPt[0].fX-stNewPt[1].fX);
	fK2=(stNewPt[2].fY-stNewPt[1].fY)/(stNewPt[2].fX-stNewPt[1].fX);
	fB2=stNewPt[1].fY-fK2*stNewPt[1].fX;

	//fMid=stNewPt[1].fY/fK1;
	fXL=stNewPt[2].fX;


	fX0=fXL*fBlendT1;
	fY0=fK1*fX0;


	fX1=fBlendT2*fXL;
	//fX1=(fXL-fMid)*fBlendT2+fMid;
	fY1=fK2*fX1+fB2;



	//if (fabs(fY0)>fabs(fB2))
	//{
	//	fY0=fK2*fX0+fB2;
	//}



	//if (fabs(fY1)>fabs(fB2))
	//{
	//	fY1=fK1*fX1;
	//}

	fXDiff=fX1-fX0;

	fA0=fY0;
	fA1=fK1;
	fA2=3*(fY1-fY0)/fXDiff/fXDiff-2*fK1/fXDiff-fK2/fXDiff;
	fA3=-2*(fY1-fY0)/fXDiff/fXDiff/fXDiff+(fK1+fK2)/fXDiff/fXDiff;

	float fXStep=(stNewPt[2].fX-stNewPt[0].fX)/nSlice,fXAcc=0,fTmpX;


	vector<float> vctBlendPos;
	vector<float>  vctWheelL;
	vector<float> vctWheelR;
	vector<float> vctBlendPosX,vctBlendPosY;

	float fXD0,fXD1,fYD0,fYD1;
	for (i=0;i<nSlice;i++)
	{
		if (fXAcc<fX0)
		{
			vctBlendPos.push_back(fXAcc);
			vctBlendPos.push_back(fK1*fXAcc);

			vctBlendPosX.push_back(fXAcc);
			vctBlendPosY.push_back(fK1*fXAcc);
		}
		else if (fXAcc>=fX0&&fXAcc<=fX1)
		{
			fTmpX=fXAcc-fX0;
			vctBlendPos.push_back(fXAcc);
			vctBlendPos.push_back(fA0+fA1*fTmpX+fA2*fTmpX*fTmpX+fA3*fTmpX*fTmpX*fTmpX);

			vctBlendPosX.push_back(fXAcc);
			vctBlendPosY.push_back(fA0+fA1*fTmpX+fA2*fTmpX*fTmpX+fA3*fTmpX*fTmpX*fTmpX);
		}
		else
		{
			//	printf("Idx  :%d  \n",i);
			vctBlendPos.push_back(fXAcc);
			vctBlendPos.push_back(fK2*fXAcc+fB2);

			vctBlendPosX.push_back(fXAcc);
			vctBlendPosY.push_back(fK2*fXAcc+fB2);
		}
		fXAcc+=fXStep;
	}

	float fTheta1,fTheta2,fVel,fWL,fWR;
	for (i=2;i<nSlice;i++)
	{
		fXD0=vctBlendPos[i*2-2]-vctBlendPos[i*2-4];
		fXD1=vctBlendPos[i*2]-vctBlendPos[i*2-2];

		fYD0=vctBlendPos[i*2-1]-vctBlendPos[i*2-3];
		fYD1=vctBlendPos[i*2+1]-vctBlendPos[i*2-1];

		fTheta1= atan2(fYD0,fXD0);
		fTheta2= atan2(fYD1,fXD1);
		fTheta= fTheta2-fTheta1;
		fVel=cos(fTheta1)*fXD0+sin(fTheta1)*fYD0;

		fWL=(fVel-fTheta)/2;
		fWR=(fVel+fTheta)/2;

	//	printf("Idx :%d  ,Vel:%f  ,Theta:%f  ,fWL:%f \n",i,fVel,fTheta,fWL);

		vctWL.push_back(fWL);
		vctWR.push_back(fWR);

	}
	return 0;
}

int Trajectory::CalWheelVel(vector<Point_f> vctPath)
{
	int i;
	float fTmp,fXD,fYD,fTheta,fXD1,fYD1,fXD2,fYD2,fTmpX1,fTmpY1,fTmpX2,fTmpY2,fLen;
	vector<float> vctSegmentLen;
	vector<float> vctTheta,vctXD,vctYD,vctXD2,vctYD2;

	vector<vector<float> > WheelL;
	vector<vector<float> > WheelR;

	vector<float> vctWL,vctWR;
	Point_f stPt[3];

	float fBlendLen=IDEAL_RADIUS;
	float fBlendLen2=IDEAL_PRE_RADIUS;

	m_vctMileStone.clear();


	for (i=1;i<(int)vctPath.size();i++)
	{
		fXD=vctPath[i].fX-vctPath[i-1].fX;
		fYD=vctPath[i].fY-vctPath[i-1].fY;

		fTmp=sqrt(fXD*fXD+fYD*fYD);
		fTheta=atan2(fYD,fXD);

		if (fTmp>2*IDEAL_PRE_RADIUS)
		{
			fXD1=cos(fTheta)*fBlendLen;
			fYD1=sin(fTheta)*fBlendLen;
			fXD2=cos(fTheta)*fBlendLen2;
			fYD2=sin(fTheta)*fBlendLen2;
		}
		else
		{
			fXD1=cos(fTheta)*fTmp*(float)0.3;
			fYD1=sin(fTheta)*fTmp*(float)0.3;
			fXD2=cos(fTheta)*fTmp*(float)0.4;
			fYD2=sin(fTheta)*fTmp*(float)0.4;
		}
		vctTheta.push_back(fTheta);
		vctXD.push_back(fXD1);
		vctYD.push_back(fYD1);
		vctXD2.push_back(fXD2);
		vctYD2.push_back(fYD2);
		vctSegmentLen.push_back(fTmp);
	}

	for (i=1;i<(int)vctPath.size()-1;i++)
	{
		vctWL.clear();
		vctWR.clear();


		stPt[0].fX=vctPath[i].fX-vctXD[i-1];
		stPt[0].fY=vctPath[i].fY-vctYD[i-1];

		stPt[1].fX=vctPath[i].fX;
		stPt[1].fY=vctPath[i].fY;

		stPt[2].fX=vctPath[i].fX+vctXD[i];
		stPt[2].fY=vctPath[i].fY+vctYD[i];

		SegmentBlend(stPt,vctWL,vctWR);

		WheelL.push_back(vctWL);
		WheelR.push_back(vctWR);

	}

	//��
	fTmpX1=vctPath[0].fX;
	fTmpY1=vctPath[0].fY;

	fTmpX2=vctPath[1].fX-vctXD2[0];
	fTmpY2=vctPath[1].fY-vctYD2[0];

	m_vctMileStone.push_back(fTmpX1);
	m_vctMileStone.push_back(fTmpY1);

	m_vctMileStone.push_back(fTmpX2);
	m_vctMileStone.push_back(fTmpY2);


	vctWL.clear();
	vctWR.clear();
	fLen=sqrt((fTmpX1-fTmpX2)*(fTmpX1-fTmpX2)+(fTmpY2-fTmpY1)*(fTmpY2-fTmpY1));
	SegmentRectilinear(0,WheelL[0][0],fLen,vctWL,vctWR);

	for (i=1;i<(int)vctPath.size()-2;i++)
	{
		vctWL.clear();
		vctWR.clear();


		stPt[0].fX=vctPath[i].fX-vctXD2[i-1];
		stPt[0].fY=vctPath[i].fY-vctYD2[i-1];

		stPt[1].fX=vctPath[i].fX+vctXD[i];
		stPt[1].fY=vctPath[i].fY+vctYD[i];

		fTmpX1=vctPath[i].fX+vctXD2[i];
		fTmpY1=vctPath[i].fY+vctXD2[i];

		fTmpX2=vctPath[i+1].fX-vctXD2[i];
		fTmpY2=vctPath[i+1].fY-vctXD2[i];


		m_vctMileStone.push_back(fTmpX1);
		m_vctMileStone.push_back(fTmpY1);

		m_vctMileStone.push_back(fTmpX2);
		m_vctMileStone.push_back(fTmpY2);

		fLen=sqrt((fTmpX1-fTmpX2)*(fTmpX1-fTmpX2)+(fTmpY2-fTmpY1)*(fTmpY2-fTmpY1));
		SegmentRectilinear(WheelL[i-1][0],WheelL[i][0],fLen,vctWL,vctWR);
	}

	fTmpX1=vctPath[vctPath.size()-2].fX-vctXD2[vctXD2.size()-1];
	fTmpY1=vctPath[vctPath.size()-2].fY-vctYD2[vctXD2.size()-1];

	fTmpX2=vctPath[vctPath.size()-1].fX;
	fTmpY2=vctPath[vctPath.size()-1].fY;

	m_vctMileStone.push_back(fTmpX1);
	m_vctMileStone.push_back(fTmpY1);

	m_vctMileStone.push_back(fTmpX2);
	m_vctMileStone.push_back(fTmpY2);

	fLen=sqrt((fTmpX1-fTmpX2)*(fTmpX1-fTmpX2)+(fTmpY2-fTmpY1)*(fTmpY2-fTmpY1));
	SegmentRectilinear(WheelL[WheelL.size()-1][0],0,fLen,vctWL,vctWR);

	return 0;
}

int Trajectory::DisMaxAccSpd()
{
	m_fDisMaxAccSpd=MAX_SPD*MAX_SPD/2/MAX_ACC_SPD;
	return 0;
}

int Trajectory::SegmentRectilinear(float fInitSpd,float fEndSpd,float fLen,vector<float> &vctWL,vector<float> &vctWR)
{
	float fLen1=(MAX_SPD*MAX_SPD-fInitSpd*fInitSpd)/(2*MAX_ACC_SPD);
	float fLen2=(fEndSpd*fEndSpd-MAX_SPD*MAX_SPD)/(2*MAX_DEACC_SPD);
	float fVm,fTAcc,fTDeacc,fTUni,fA;
	float fUp,fDown;
	RectilinearCtrl stRectilinearCtrl;
	memset(&stRectilinearCtrl,0,sizeof(RectilinearCtrl));

	fInitSpd*=100;
	fEndSpd*=100;



	/*if (fLen<MIN_RECTIL_LEN)
	{
	if (fInitSpd==fEndSpd)
	{
	stRectilinearCtrl.cType=1;
	stRectilinearCtrl.fTUni=fLen/fInitSpd;

	}
	else
	{
	stRectilinearCtrl.cType=2;
	stRectilinearCtrl.fA=(fEndSpd*fEndSpd-fInitSpd*fInitSpd)/(2*fLen);
	stRectilinearCtrl.fTAcc=(fEndSpd-fInitSpd)/fA;
	}
	}
	else if ((fLen1+fLen2)<fLen)
	{
	stRectilinearCtrl.cType=3;
	stRectilinearCtrl.fTAcc=(MAX_SPD-fInitSpd)/MAX_ACC_SPD;
	stRectilinearCtrl.fTUni=(fLen-fLen1-fLen2)/MAX_SPD;
	stRectilinearCtrl.fTDeacc=(fEndSpd-MAX_SPD)/MAX_DEACC_SPD;

	}
	else
	{
	stRectilinearCtrl.cType=4;
	fUp=2*MAX_ACC_SPD*MAX_DEACC_SPD*fLen+MAX_DEACC_SPD*fInitSpd*fInitSpd-MAX_ACC_SPD*fEndSpd*fEndSpd;
	fDown=MAX_DEACC_SPD-MAX_SPD;
	stRectilinearCtrl.fVm=sqrt((2*MAX_ACC_SPD*MAX_DEACC_SPD*fLen+MAX_DEACC_SPD*fInitSpd*fInitSpd-MAX_ACC_SPD*fEndSpd*fEndSpd)/(MAX_DEACC_SPD-MAX_SPD));
	stRectilinearCtrl.fTAcc=(fVm-fInitSpd)/MAX_ACC_SPD;
	stRectilinearCtrl.fTDeacc=(fEndSpd-fVm)/MAX_DEACC_SPD;
	}
	m_vctRectilinearCtrl.push_back(stRectilinearCtrl);*/
	return 0;
}
/*
int main()
{
	Trajectory CTrajectory;
	float fMidSpd,fT1,fT2;
	CTrajectory.CalMidSpd(1,0.5,1,-3,10,fMidSpd,fT1,fT2);
	vector<Point_f> vctPath,vctPath1;
	vector<float> vctSpdLmt;
	Point_f fPoint;
	int i;

	float fSpdLmt;
	fPoint.fX=0;
	fPoint.fY=0;
	vctPath.push_back(fPoint);


	fPoint.fX=5;
	fPoint.fY=0;
	vctPath.push_back(fPoint);
	fSpdLmt=2;
	vctSpdLmt.push_back(fSpdLmt);


	fPoint.fX=5;
	fPoint.fY=10;
	vctPath.push_back(fPoint);
	fSpdLmt=1;
	vctSpdLmt.push_back(fSpdLmt);


	fPoint.fX=6;
	fPoint.fY=10;
	vctPath.push_back(fPoint);
	fSpdLmt=2;
	vctSpdLmt.push_back(fSpdLmt);


	fPoint.fX=6;
	fPoint.fY=9;
	vctPath.push_back(fPoint);
	fSpdLmt=2;
	vctSpdLmt.push_back(fSpdLmt);



	fPoint.fX=7;
	fPoint.fY=9;
	vctPath.push_back(fPoint);
	fSpdLmt=2;
	vctSpdLmt.push_back(fSpdLmt);


	fPoint.fX=7;
	fPoint.fY=19;
	vctPath.push_back(fPoint);
	fSpdLmt=2;
	vctSpdLmt.push_back(fSpdLmt);


	fPoint.fX=8;
	fPoint.fY=19;
	vctPath.push_back(fPoint);
	fSpdLmt=2;
	vctSpdLmt.push_back(fSpdLmt);


	fPoint.fX=8;
	fPoint.fY=1;
	vctPath.push_back(fPoint);
	fSpdLmt=2;
	vctSpdLmt.push_back(fSpdLmt);


	fPoint.fX=10;
	fPoint.fY=1;
	vctPath.push_back(fPoint);
	fSpdLmt=2;
	vctSpdLmt.push_back(fSpdLmt);


	fPoint.fX=10;
	fPoint.fY=2;
	vctPath.push_back(fPoint);
	fSpdLmt=2;
	vctSpdLmt.push_back(fSpdLmt);
	CTrajectory.NewTrajectory(vctPath,vctSpdLmt);
	return 0;
}*/
/*int Trajectory::GetMileStone(vector<Point_f> vctPath,vector<Point_f> &vctSegmentLen,vector<Point_f> &vctBlendSegmentLen)
{
	int i;
	clock_t time1,time2,nSpdLmtCount=0;;
	float fBlendLen=IDEAL_RADIUS;
	float fBlendLen2=IDEAL_PRE_RADIUS;
	float fTmp,fXD,fYD,fTheta,fXD1,fYD1,fXD2,fYD2,fTmpX1,fTmpY1,fTmpX2,fTmpY2,fLen;
	float fAngle1,fAngle2,fXDD1,fXDD2,fYDD1,fYDD2;
	float fSLAMLen,fTmp2;
	vector<float> vctTmp,vctSLAMLen;
	vector<float> vctTheta;
	Point_f stTmpPoint1,stTmpPoint2,stTmpPoint,stTmpPoint3;
	vector<float> vctVL;
	vector<float> vctVR;
	if (vctPath.size()==2)
	{
		vctSegmentLen=vctPath;
	}
	else
	{
		for (i=1;i<(int)vctPath.size();i++)
		{
			fXD=vctPath[i].fX-vctPath[i-1].fX;
			fYD=vctPath[i].fY-vctPath[i-1].fY;

			fTmp=sqrt(fXD*fXD+fYD*fYD);


			fTheta=atan2(fYD,fXD);
			fTmp*=0.4;

						//·����,�͸�������ת��뾶
			if(fTmp>IDEAL_RADIUS)
			{
				fTmp=IDEAL_RADIUS;
			}

			//fSLAMLen=fTmp*0.1;
			fTmp*=0.4;

			if(fBlendLen>IDEAL_RADIUS)
			{
				fBlendLen=IDEAL_RADIUS;
			}

			if(fSLAMLen>KEEP_4_SLAM_LEN)
			{
				fSLAMLen=KEEP_4_SLAM_LEN;
			}


			vctTmp.push_back(fTmp);
			vctTheta.push_back(fTheta);
			vctSLAMLen.push_back(fSLAMLen);
		}

		fAngle1=vctTheta[0];
		fAngle2=vctTheta[1];


		fTmp=MIN(vctTmp[0],vctTmp[1]);
		fTmp2=MIN(vctSLAMLen[0],vctSLAMLen[1]);

		fXD1=cos(fAngle1)*fTmp;
		fYD1=sin(fAngle1)*fTmp;
		fXD2=cos(fAngle2)*fTmp;
		fYD2=sin(fAngle2)*fTmp;

		fXDD1=cos(fAngle1)*(fTmp+fTmp2);
		fYDD1=cos(fAngle1)*(fTmp+fTmp2);
		fXDD2=cos(fAngle2)*(fTmp+fTmp2);
		fYDD2=cos(fAngle2)*(fTmp+fTmp2);

	//	stTmpPoint1.fX=vctPath[1].fX-fXDD1;
	//	stTmpPoint1.fY=vctPath[1].fY-fYDD1;

		stTmpPoint1.fX=vctPath[i].fX-fXD1;
		stTmpPoint1.fY=vctPath[i].fY-fYD1;

		stTmpPoint2.fX=vctPath[1].fX+fXD2;
		stTmpPoint2.fY=vctPath[1].fY+fYD2;

		stTmpPoint=vctPath[0];
		vctSegmentLen.push_back(stTmpPoint);
		vctSegmentLen.push_back(stTmpPoint1);
		vctSegmentLen.push_back(stTmpPoint2);


		stTmpPoint1.fX=vctPath[1].fX-fXD1;
		stTmpPoint1.fY=vctPath[1].fY-fYD1;

		stTmpPoint2.fX=vctPath[1].fX+fXD2;
		stTmpPoint2.fY=vctPath[1].fY+fYD2;
		stTmpPoint1.fLen=fTmp;
		stTmpPoint2.fLen=fTmp;
		vctBlendSegmentLen.push_back(stTmpPoint1);
		vctBlendSegmentLen.push_back(vctPath[1]);
		vctBlendSegmentLen.push_back(stTmpPoint2);


		for(i=2;i<vctPath.size()-1;i++)
		{
			fAngle1=vctTheta[i-1];
			fAngle2=vctTheta[i];

			fTmp=MIN(vctTmp[i-1],vctTmp[i]);
			fTmp2=MIN(vctSLAMLen[i-1],vctSLAMLen[i]);

			fXD1=cos(fAngle1)*fTmp;
			fYD1=sin(fAngle1)*fTmp;
			fXD2=cos(fAngle2)*fTmp;
			fYD2=sin(fAngle2)*fTmp;


			fXDD1=cos(fAngle1)*(fTmp+fTmp2);
			fYDD1=cos(fAngle1)*(fTmp+fTmp2);
			fXDD2=cos(fAngle2)*(fTmp+fTmp2);
			fYDD2=cos(fAngle2)*(fTmp+fTmp2);

			//stTmpPoint1.fX=vctPath[i].fX-fXDD1;
			//stTmpPoint1.fY=vctPath[i].fY-fYDD1;

			stTmpPoint1.fX=vctPath[i].fX-fXD1;
			stTmpPoint1.fY=vctPath[i].fY-fYD1;

			stTmpPoint2.fX=vctPath[i].fX+fXD2;
			stTmpPoint2.fY=vctPath[i].fY+fYD2;

			vctSegmentLen.push_back(stTmpPoint1);
			vctSegmentLen.push_back(stTmpPoint2);


			stTmpPoint1.fLen=fTmp;
			stTmpPoint2.fLen=fTmp;
			stTmpPoint1.fX=vctPath[i].fX-fXD1;
			stTmpPoint1.fY=vctPath[i].fY-fYD1;

			stTmpPoint2.fX=vctPath[i].fX+fXD2;
			stTmpPoint2.fY=vctPath[i].fY+fYD2;

			vctBlendSegmentLen.push_back(stTmpPoint1);
			vctBlendSegmentLen.push_back(vctPath[i]);
			vctBlendSegmentLen.push_back(stTmpPoint2);
		}

		stTmpPoint=vctPath[vctPath.size()-1];
		vctSegmentLen.push_back(stTmpPoint);
	}
}*/
int Trajectory::GetMileStone(vector<Point_f> vctPath,
		vector<Point_f> &vctSegmentLen,
		vector<Point_f> &vctBlendSegmentLen,
		vector<bool> &vctUsingSLAMData,
		vector<float> &vctTurningSPD)
{
		int i;
		clock_t time1,time2,nSpdLmtCount=0;;
		float fBlendLen=IDEAL_RADIUS;
		float fBlendLen2=IDEAL_PRE_RADIUS;
		float fTmp,fXD,fYD,fTheta,fXD1,fYD1,fXD2,fYD2,fTmpX1,fTmpY1,fTmpX2,fTmpY2,fLen;
		float fAngle1,fAngle2,fXDD1,fXDD2,fYDD1,fYDD2;
		float fSLAMLen,fTmp2;
		vector<float> vctTmp,vctSLAMLen;
		vector<float> vctTheta;
		Point_f stTmpPoint1,stTmpPoint2,stTmpPoint,stTmpPoint3;
		vector<float> vctVL;
		vector<float> vctVR;
		bool bUsingSLAMData;
		float fTurnningSpd;

		vctSegmentLen.clear();
		vctBlendSegmentLen.clear();
		vctUsingSLAMData.clear();
		vctTurningSPD.clear();


		if (vctPath.size()==2)
		{
			vctSegmentLen=vctPath;
		}
		else
		{
			for (i=1;i<(int)vctPath.size();i++)
			{
				fXD=vctPath[i].fX-vctPath[i-1].fX;
				fYD=vctPath[i].fY-vctPath[i-1].fY;

				fTmp=sqrt(fXD*fXD+fYD*fYD);
				fTheta=atan2(fYD,fXD);

				fSLAMLen=fTmp*0.1;
				fBlendLen=fTmp*0.3;

				if(fBlendLen>IDEAL_RADIUS)
				{
					fBlendLen=IDEAL_RADIUS;
				}

				if(fSLAMLen>KEEP_4_SLAM_LEN)
				{
					fSLAMLen=KEEP_4_SLAM_LEN;
				}


				vctTmp.push_back(fBlendLen);
				vctTheta.push_back(fTheta);
				vctSLAMLen.push_back(fSLAMLen);
			}

			fAngle1=vctTheta[0];
			fAngle2=vctTheta[1];


			fTmp=MIN(vctTmp[0],vctTmp[1]);
			//fTmp2=MIN(vctSLAMLen[0],vctSLAMLen[1]);
			fTmp2=vctSLAMLen[0];

			fXD1=cos(fAngle1)*fTmp;
			fYD1=sin(fAngle1)*fTmp;
			fXD2=cos(fAngle2)*fTmp;
			fYD2=sin(fAngle2)*fTmp;

			fXDD1=cos(fAngle1)*(fTmp+fTmp2);
			fYDD1=sin(fAngle1)*(fTmp+fTmp2);
			fXDD2=cos(fAngle2)*(fTmp+fTmp2);
			fYDD2=sin(fAngle2)*(fTmp+fTmp2);


			stTmpPoint1.fX=vctPath[1].fX-fXDD1;
			stTmpPoint1.fY=vctPath[1].fY-fYDD1;

			stTmpPoint2.fX=vctPath[1].fX+fXD2;
			stTmpPoint2.fY=vctPath[1].fY+fYD2;

			stTmpPoint=vctPath[0];
			vctSegmentLen.push_back(stTmpPoint);
			vctSegmentLen.push_back(stTmpPoint1);
			vctSegmentLen.push_back(stTmpPoint2);


			stTmpPoint1.fX=vctPath[1].fX-fXD1;
			stTmpPoint1.fY=vctPath[1].fY-fYD1;

			stTmpPoint2.fX=vctPath[1].fX+fXD2;
			stTmpPoint2.fY=vctPath[1].fY+fYD2;
			stTmpPoint1.fLen=fTmp;
			stTmpPoint2.fLen=fTmp;


			vctBlendSegmentLen.push_back(stTmpPoint1);
			vctBlendSegmentLen.push_back(vctPath[1]);
			vctBlendSegmentLen.push_back(stTmpPoint2);

			if(fTmp>USING_SLAM_DATA_THRS)
			{
				bUsingSLAMData=true;
			}
			else
			{
				bUsingSLAMData=false;
			}
			vctUsingSLAMData.push_back(bUsingSLAMData);
			fTurnningSpd=fTmp*2/TURNNING_TIME;
			vctTurningSPD.push_back(fTurnningSpd);

			for(i=2;i<vctPath.size()-1;i++)
			{
				fAngle1=vctTheta[i-1];
				fAngle2=vctTheta[i];

				fTmp=MIN(vctTmp[i-1],vctTmp[i]);
				//fTmp2=MIN(vctSLAMLen[i-1],vctSLAMLen[i]);
				fTmp2=vctSLAMLen[i-1];

				fXD1=cos(fAngle1)*fTmp;
				fYD1=sin(fAngle1)*fTmp;
				fXD2=cos(fAngle2)*fTmp;
				fYD2=sin(fAngle2)*fTmp;


				fXDD1=cos(fAngle1)*(fTmp+fTmp2);
				fYDD1=sin(fAngle1)*(fTmp+fTmp2);
				fXDD2=cos(fAngle2)*(fTmp+fTmp2);
				fYDD2=sin(fAngle2)*(fTmp+fTmp2);

				//stTmpPoint1.fX=vctPath[i].fX-fXDD1;
				//stTmpPoint1.fY=vctPath[i].fY-fYDD1;

				stTmpPoint1.fX=vctPath[i].fX-fXDD1;
				stTmpPoint1.fY=vctPath[i].fY-fYDD1;

				stTmpPoint2.fX=vctPath[i].fX+fXD2;
				stTmpPoint2.fY=vctPath[i].fY+fYD2;

				vctSegmentLen.push_back(stTmpPoint1);
				vctSegmentLen.push_back(stTmpPoint2);


				stTmpPoint1.fLen=fTmp;
				stTmpPoint2.fLen=fTmp;
				stTmpPoint1.fX=vctPath[i].fX-fXD1;
				stTmpPoint1.fY=vctPath[i].fY-fYD1;

				stTmpPoint2.fX=vctPath[i].fX+fXD2;
				stTmpPoint2.fY=vctPath[i].fY+fYD2;

				vctBlendSegmentLen.push_back(stTmpPoint1);
				vctBlendSegmentLen.push_back(vctPath[i]);
				vctBlendSegmentLen.push_back(stTmpPoint2);

				if(fTmp>USING_SLAM_DATA_THRS)
				{
					bUsingSLAMData=true;
				}
				else
				{
					bUsingSLAMData=false;
				}
				vctUsingSLAMData.push_back(bUsingSLAMData);
				fTurnningSpd=fTmp*2/TURNNING_TIME;
				vctTurningSPD.push_back(fTurnningSpd);
			}

			stTmpPoint=vctPath[vctPath.size()-1];
			vctSegmentLen.push_back(stTmpPoint);
		}

		return 0;
}
int Trajectory::GetMileStone(vector<Point_f> vctPath,vector<Point_f> &vctSegmentLen,vector<Point_f> &vctBlendSegmentLen)
{
	int i;
	clock_t time1,time2,nSpdLmtCount=0;;
	float fBlendLen=IDEAL_RADIUS;
	float fBlendLen2=IDEAL_PRE_RADIUS;
	float fTmp,fXD,fYD,fTheta,fXD1,fYD1,fXD2,fYD2,fTmpX1,fTmpY1,fTmpX2,fTmpY2,fLen;
	float fAngle1,fAngle2,fXDD1,fXDD2,fYDD1,fYDD2;
	float fSLAMLen,fTmp2;
	vector<float> vctTmp,vctSLAMLen;
	vector<float> vctTheta;
	Point_f stTmpPoint1,stTmpPoint2,stTmpPoint,stTmpPoint3;
	vector<float> vctVL;
	vector<float> vctVR;
	if (vctPath.size()==2)
	{
		vctSegmentLen=vctPath;
	}
	else
	{
		for (i=1;i<(int)vctPath.size();i++)
		{
			fXD=vctPath[i].fX-vctPath[i-1].fX;
			fYD=vctPath[i].fY-vctPath[i-1].fY;

			fTmp=sqrt(fXD*fXD+fYD*fYD);


			fTheta=atan2(fYD,fXD);
		/*	fTmp*=0.4;

			//¡€????,?????????????
			if(fTmp>IDEAL_RADIUS)
			{
				fTmp=IDEAL_RADIUS;
			}*/

			fSLAMLen=fTmp*0.1;
			fBlendLen=fTmp*0.3;

			if(fBlendLen>IDEAL_RADIUS)
			{
				fBlendLen=IDEAL_RADIUS;
			}

			if(fSLAMLen>KEEP_4_SLAM_LEN)
			{
				fSLAMLen=KEEP_4_SLAM_LEN;
			}


			vctTmp.push_back(fBlendLen);
			vctTheta.push_back(fTheta);
			vctSLAMLen.push_back(fSLAMLen);
		}

		fAngle1=vctTheta[0];
		fAngle2=vctTheta[1];


		fTmp=MIN(vctTmp[0],vctTmp[1]);
		//fTmp2=MIN(vctSLAMLen[0],vctSLAMLen[1]);
		fTmp2=vctSLAMLen[0];

		fXD1=cos(fAngle1)*fTmp;
		fYD1=sin(fAngle1)*fTmp;
		fXD2=cos(fAngle2)*fTmp;
		fYD2=sin(fAngle2)*fTmp;

		fXDD1=cos(fAngle1)*(fTmp+fTmp2);
		fYDD1=sin(fAngle1)*(fTmp+fTmp2);
		fXDD2=cos(fAngle2)*(fTmp+fTmp2);
		fYDD2=sin(fAngle2)*(fTmp+fTmp2);


		stTmpPoint1.fX=vctPath[1].fX-fXDD1;
		stTmpPoint1.fY=vctPath[1].fY-fYDD1;

		stTmpPoint2.fX=vctPath[1].fX+fXD2;
		stTmpPoint2.fY=vctPath[1].fY+fYD2;

		stTmpPoint=vctPath[0];
		vctSegmentLen.push_back(stTmpPoint);
		vctSegmentLen.push_back(stTmpPoint1);
		vctSegmentLen.push_back(stTmpPoint2);


		stTmpPoint1.fX=vctPath[1].fX-fXD1;
		stTmpPoint1.fY=vctPath[1].fY-fYD1;

		stTmpPoint2.fX=vctPath[1].fX+fXD2;
		stTmpPoint2.fY=vctPath[1].fY+fYD2;
		stTmpPoint1.fLen=fTmp;
		stTmpPoint2.fLen=fTmp;
		vctBlendSegmentLen.push_back(stTmpPoint1);
		vctBlendSegmentLen.push_back(vctPath[1]);
		vctBlendSegmentLen.push_back(stTmpPoint2);


		for(i=2;i<vctPath.size()-1;i++)
		{
			fAngle1=vctTheta[i-1];
			fAngle2=vctTheta[i];

			fTmp=MIN(vctTmp[i-1],vctTmp[i]);
			//fTmp2=MIN(vctSLAMLen[i-1],vctSLAMLen[i]);
			fTmp2=vctSLAMLen[i-1];

			fXD1=cos(fAngle1)*fTmp;
			fYD1=sin(fAngle1)*fTmp;
			fXD2=cos(fAngle2)*fTmp;
			fYD2=sin(fAngle2)*fTmp;


			fXDD1=cos(fAngle1)*(fTmp+fTmp2);
			fYDD1=sin(fAngle1)*(fTmp+fTmp2);
			fXDD2=cos(fAngle2)*(fTmp+fTmp2);
			fYDD2=sin(fAngle2)*(fTmp+fTmp2);

			//stTmpPoint1.fX=vctPath[i].fX-fXDD1;
			//stTmpPoint1.fY=vctPath[i].fY-fYDD1;

			stTmpPoint1.fX=vctPath[i].fX-fXDD1;
			stTmpPoint1.fY=vctPath[i].fY-fYDD1;

			stTmpPoint2.fX=vctPath[i].fX+fXD2;
			stTmpPoint2.fY=vctPath[i].fY+fYD2;

			vctSegmentLen.push_back(stTmpPoint1);
			vctSegmentLen.push_back(stTmpPoint2);


			stTmpPoint1.fLen=fTmp;
			stTmpPoint2.fLen=fTmp;
			stTmpPoint1.fX=vctPath[i].fX-fXD1;
			stTmpPoint1.fY=vctPath[i].fY-fYD1;

			stTmpPoint2.fX=vctPath[i].fX+fXD2;
			stTmpPoint2.fY=vctPath[i].fY+fYD2;

			vctBlendSegmentLen.push_back(stTmpPoint1);
			vctBlendSegmentLen.push_back(vctPath[i]);
			vctBlendSegmentLen.push_back(stTmpPoint2);
		}

		stTmpPoint=vctPath[vctPath.size()-1];
		vctSegmentLen.push_back(stTmpPoint);
	}

	return 0;
}
int Trajectory::NewTrajectory(vector<Point_f> vctPath,vector<float> vctSpdLmt,float fInitV)
{
	clock_t time1,time2,nSpdLmtCount=0;
	time1=clock();
	float fBlendLen=IDEAL_RADIUS;
	float fBlendLen2=IDEAL_PRE_RADIUS;
	float fTmp,fXD,fYD,fTheta,fXD1,fYD1,fXD2,fYD2,fTmpX1,fTmpY1,fTmpX2,fTmpY2,fLen;
	float fAngle1,fAngle2;
	vector<float> vctTmp;
	vector<float> vctTheta;
	vector<Point_f> vctSegmentLen;
	vector<Point_f> vctBlendSegmentLen;
	Point_f stTmpPoint1,stTmpPoint2,stTmpPoint;
	vector<float> vctVL;
	vector<float> vctVR;
	int i,j;

	m_vctSegmentLen.clear();
	m_vctBlendSegmentLen.clear();

	float fEndSpd=0.5;
	float fInitSpd=0;
	
	Point_f sBlendPoint[3];

	int k;
	float fV,fThetaAcc=0,fXS,fYS,fXAcc=0,fYAcc=0;

	m_vctRecvLinearVL.clear();
	m_vctRecvLinearVR.clear();
	m_vctBlendVL.clear();
	m_vctBlendVR.clear();

	if (vctPath.size()==2)
	{
		fEndSpd=0;
		fInitSpd=0;
		vctVL.clear();
		vctVR.clear();
		NewSegmentRectilinear(fInitSpd,fEndSpd,vctSpdLmt[0],vctPath[0],vctPath[1],vctVL,vctVR);
		m_vctRecvLinearVL.push_back(vctVL);
		m_vctRecvLinearVR.push_back(vctVR);
		//ֱ���˶�
	}
	else//�Ǽ�ֱ���˶�
	{
		//����ÿ���������ת������߶γ���
		for (i=1;i<(int)vctPath.size();i++)
		{
			fXD=vctPath[i].fX-vctPath[i-1].fX;
			fYD=vctPath[i].fY-vctPath[i-1].fY;

			fTmp=sqrt(fXD*fXD+fYD*fYD);


			fTheta=atan2(fYD,fXD);
			fTmp*=0.4;

			//·����,�͸�������ת��뾶
			if(fTmp>IDEAL_RADIUS)
			{
				fTmp=IDEAL_RADIUS;
			}

			vctTmp.push_back(fTmp);
			vctTheta.push_back(fTheta);
		}

		//�����ʼ��������ת��뾶
		fAngle1=vctTheta[0];
		fAngle2=vctTheta[1];

		//ǰ��ת��뾶һ��,��������ٶ�һ��,ѡȡ����С��
		fTmp=MIN(vctTmp[0],vctTmp[1]);

		//��������
		fXD1=cos(fAngle1)*fTmp;
		fYD1=sin(fAngle1)*fTmp;
		fXD2=cos(fAngle2)*fTmp;
		fYD2=sin(fAngle2)*fTmp;

		stTmpPoint1.fX=vctPath[1].fX-fXD1;
		stTmpPoint1.fY=vctPath[1].fY-fYD1;

		stTmpPoint2.fX=vctPath[1].fX+fXD2;
		stTmpPoint2.fY=vctPath[1].fY+fYD2;

		stTmpPoint=vctPath[0];
		vctSegmentLen.push_back(stTmpPoint);
		vctSegmentLen.push_back(stTmpPoint1);
		vctSegmentLen.push_back(stTmpPoint2);


		stTmpPoint1.fLen=fTmp;
		stTmpPoint2.fLen=fTmp;
		vctBlendSegmentLen.push_back(stTmpPoint1);
		vctBlendSegmentLen.push_back(vctPath[1]);
		vctBlendSegmentLen.push_back(stTmpPoint2);

		//��������ʼ���յ�����λ�õ�ת��λ�õ��Լ�����

		for(i=2;i<vctPath.size()-1;i++)
		{
			fAngle1=vctTheta[i-1];
			fAngle2=vctTheta[i];

			fTmp=MIN(vctTmp[i-1],vctTmp[i]);

			fXD1=cos(fAngle1)*fTmp;
			fYD1=sin(fAngle1)*fTmp;
			fXD2=cos(fAngle2)*fTmp;
			fYD2=sin(fAngle2)*fTmp;

			stTmpPoint1.fX=vctPath[i].fX-fXD1;
			stTmpPoint1.fY=vctPath[i].fY-fYD1;

			stTmpPoint2.fX=vctPath[i].fX+fXD2;
			stTmpPoint2.fY=vctPath[i].fY+fYD2;

			vctSegmentLen.push_back(stTmpPoint1);
			vctSegmentLen.push_back(stTmpPoint2);


			stTmpPoint1.fLen=fTmp;
			stTmpPoint2.fLen=fTmp;
			vctBlendSegmentLen.push_back(stTmpPoint1);
			vctBlendSegmentLen.push_back(vctPath[i]);
			vctBlendSegmentLen.push_back(stTmpPoint2);
		}

		stTmpPoint=vctPath[vctPath.size()-1];
		vctSegmentLen.push_back(stTmpPoint);
	//GetMileStone(vctPath,vctSegmentLen,vctBlendSegmentLen);
	m_vctSegmentLen=vctSegmentLen;
	m_vctBlendSegmentLen=vctBlendSegmentLen;
		j=0;
		fEndSpd=MAX_TURNNING_SPD;
		fInitSpd=fInitV;
		
		for (i=0;i<vctSegmentLen.size()-3;i=i+2)
		{
			vctVL.clear();
			vctVR.clear();
			NewSegmentRectilinear(fInitSpd,fEndSpd,vctSpdLmt[nSpdLmtCount],vctSegmentLen[i],vctSegmentLen[i+1],vctVL,vctVR);

			printf("fEndSpd:%f  \n",fEndSpd);
			m_vctRecvLinearVL.push_back(vctVL);
			m_vctRecvLinearVR.push_back(vctVR);
			fInitSpd=fEndSpd;
			fEndSpd=MAX_TURNNING_SPD;
			nSpdLmtCount++;

			sBlendPoint[0]=vctBlendSegmentLen[j++];
			sBlendPoint[1]=vctBlendSegmentLen[j++];
			sBlendPoint[2]=vctBlendSegmentLen[j++];
			vctVL.clear();
			vctVR.clear();
			NewSegmentBlend(sBlendPoint,vctVL,vctVR,fInitSpd);

		//	fInitSpd=fInitSpd;


		/*	for (k=0;k<vctVL.size();k++)
			{
				fTheta=(vctVR[k]-vctVL[k])/ROBOT_L;
				fV=(vctVR[k]+vctVL[k])/2;
				fThetaAcc+=fTheta;
				fXS=cos(fThetaAcc)*fV;
				fYS=sin(fThetaAcc)*fV;
				fXAcc+=fXS;
				fYAcc+=fYS;
				printf("fTheta  %f  \n",fTheta);
			}*/


			m_vctBlendVL.push_back(vctVL);
			m_vctBlendVR.push_back(vctVR);
		}
		fEndSpd=0;
		NewSegmentRectilinear(fInitSpd,fEndSpd,vctSpdLmt[nSpdLmtCount],vctSegmentLen[vctSegmentLen.size()-2],vctSegmentLen[vctSegmentLen.size()-1],vctVL,vctVR);
		m_vctRecvLinearVL.push_back(vctVL);
		m_vctRecvLinearVR.push_back(vctVR);
		/*for (k=0;k<vctVL.size();k++)
		{
			fV=vctVR[k]*CMD_SLICE_LEN;
			fXS=cos(fThetaAcc)*fV;
			fYS=sin(fThetaAcc)*fV;
			fXAcc+=fXS;
			fYAcc+=fYS;
			printf("fTheta  %f  \n",fTheta);
		}*/
	}


//	time2=clock();
//	printf("time cost: %d\n",time2-time1);
	return 0;

}

int Trajectory::NewSegmentRectilinear(float fInitSpd,float &fEndSpd,float fSpdlmt,Point_f stStartPoint,Point_f stEndPoint,vector<float> &vctVL,vector<float> &vctVR)
{
	float fTmp,fXD,fYD,fAccT,fS1,fS2,fS3,fT1,fT2;
	float fT,fV;
	int nSlice,i;
	float fTerSpd,fMidSpd;


//	float fLen=0;
	//����
	fXD=stStartPoint.fX-stEndPoint.fX;
	fYD=stStartPoint.fY-stEndPoint.fY;
	fTmp=sqrt(fXD*fXD+fYD*fYD);

	fTerSpd=sqrt(2*MAX_ACC_SPD*fTmp-fInitSpd*fInitSpd);

	vctVL.clear();
	vctVR.clear();

	printf("Total Line Len:%f  \n",fTmp);

	if (fTerSpd<=fEndSpd)
	{
		printf("fEndSpd!!!!\n");
		fT=(fTerSpd-fInitSpd)/MAX_ACC_SPD;
		nSlice=fT/CMD_SLICE_LEN;
		fAccT=CMD_SLICE_LEN;
		for (i=0;i<nSlice;i++)
		{
			fV=fAccT*MAX_ACC_SPD;
			vctVL.push_back(fV);
			vctVR.push_back(fV);
		}
		fEndSpd=fTerSpd;
	}
	else
	{
		float fTmp2;
		bool bCalMidSpd=false;
		if(fInitSpd==fEndSpd)
			{
				bCalMidSpd=true;
			}
		else if(fInitSpd>fEndSpd)
		{
			//printf("only Deacc:%f,  %f  \n",fInitSpd,fEndSpd);
			fTmp2=(fEndSpd*fEndSpd-fInitSpd*fInitSpd)/(2*MAX_DEACC_SPD);
			printf("only Decc:%f,  %f  ,%f,%f\n",fInitSpd,fEndSpd,fTmp,fTmp2);
			if(fTmp2<fTmp+0.5)
			{
				bCalMidSpd=true;
			}
			else
			{
				fEndSpd=sqrt(fTmp*2*MAX_DEACC_SPD+fInitSpd*fInitSpd);
				fT=(fEndSpd-fInitSpd)/MAX_DEACC_SPD;
				nSlice=fT/CMD_SLICE_LEN;
				fAccT=0;
				printf("Actual Deacc End Spd:%f, %f  ,%d  \n",fEndSpd,fT,nSlice);
				for (i=0;i<=nSlice;i++)
				{
					fV=fInitSpd+fAccT*MAX_DEACC_SPD;
					vctVL.push_back(fV);
					vctVR.push_back(fV);
					fAccT+=CMD_SLICE_LEN;
							//	fLen+=fV*CMD_SLICE_LEN;
				}

			}
		}
		else if(fInitSpd<fEndSpd)
		{

			fTmp2=(fEndSpd*fEndSpd-fInitSpd*fInitSpd)/(2*MAX_ACC_SPD);
			printf("only Acc:%f,  %f  ,%f,  %f\n",fInitSpd,fEndSpd,fTmp,fTmp2);
			if(fTmp2<fTmp+0.5)
			{
				bCalMidSpd=true;
			}
			else
			{
				fEndSpd=sqrt(fTmp*2*MAX_ACC_SPD+fInitSpd*fInitSpd);
				fT=(fEndSpd-fInitSpd)/MAX_ACC_SPD;
				nSlice=fT/CMD_SLICE_LEN;

				printf("Actual Acc End Spd:%f, %f  ,%d  \n",fEndSpd,fT,nSlice);
				fAccT=0;
				for (i=0;i<=nSlice;i++)
				{
					fV=fInitSpd+fAccT*MAX_ACC_SPD;
					vctVL.push_back(fV);
					vctVR.push_back(fV);
					fAccT+=CMD_SLICE_LEN;
											//	fLen+=fV*CMD_SLICE_LEN;
				}
			}
		}

		if(bCalMidSpd)
		{
			CalMidSpd(fInitSpd,fEndSpd,MAX_ACC_SPD,MAX_DEACC_SPD,fTmp,fMidSpd,fT1,fT2);
				printf("fInitSpd:%f,fEndSpd:%f,fTmp:%f,fMidSpd:%f,fT1:%f,fT2:%f \n",
						fInitSpd,fEndSpd,fTmp,fMidSpd,fT1,fT2);
				if (fMidSpd<=fSpdlmt)
				{
					fS1=(fMidSpd*fMidSpd-fInitSpd*fInitSpd)/(2*MAX_ACC_SPD);
					fS2=(fEndSpd*fEndSpd-fMidSpd*fMidSpd)/(2*MAX_DEACC_SPD);

					fT=(fMidSpd-fInitSpd)/MAX_ACC_SPD;
					nSlice=fT/CMD_SLICE_LEN;
					printf("nSlice:%d \n",nSlice);
					fAccT=0;
					for (i=0;i<=nSlice;i++)
					{
						fV=fInitSpd+fAccT*MAX_ACC_SPD;
						vctVL.push_back(fV);
						vctVR.push_back(fV);
						fAccT+=CMD_SLICE_LEN;
					//	fLen+=fV*CMD_SLICE_LEN;
					}

					fT=(fEndSpd-fMidSpd)/MAX_DEACC_SPD;
					nSlice=fT/CMD_SLICE_LEN;
					fAccT=0;
					for (i=0;i<=nSlice;i++)
					{
						fV=fMidSpd+fAccT*MAX_DEACC_SPD;
						vctVL.push_back(fV);
						vctVR.push_back(fV);
						fAccT+=CMD_SLICE_LEN;
					//	fLen+=fV*CMD_SLICE_LEN;
					}

				}
				else
				{
					fS1=(fSpdlmt*fSpdlmt-fInitSpd*fInitSpd)/(2*MAX_ACC_SPD);
					fS2=(fEndSpd*fEndSpd-fSpdlmt*fSpdlmt)/(2*MAX_DEACC_SPD);
					fS3=fTmp-fS1-fS2;

					printf(" fS1:%f  fS2:%f fS3:%f \n",fS1,fS2,fS3);

					fAccT=0;
					fT=(fSpdlmt-fInitSpd)/MAX_ACC_SPD;
					nSlice=fT/CMD_SLICE_LEN+0.5;

					printf("nSlice:%d \n",nSlice);
					for (i=0;i<=nSlice;i++)
					{
						fV=fInitSpd+fAccT*MAX_ACC_SPD;
						vctVL.push_back(fV);
						vctVR.push_back(fV);
					//	fLen+=fV*CMD_SLICE_LEN;
						fAccT+=CMD_SLICE_LEN;
					}

					fT=fS3/fSpdlmt;
					nSlice=fT/CMD_SLICE_LEN+0.5;

					printf("nSlice:%d \n",nSlice);

					for (i=0;i<=nSlice;i++)
					{
						vctVL.push_back(fSpdlmt);
						vctVR.push_back(fSpdlmt);
					//	fLen+=fSpdlmt*CMD_SLICE_LEN;
					}

					fT=(fEndSpd-fSpdlmt)/MAX_DEACC_SPD;
					nSlice=fT/CMD_SLICE_LEN+0.5;
					printf("nSlice:%d \n",nSlice);
					fAccT=0;
					for (i=0;i<=nSlice;i++)
					{
						fV=fSpdlmt+fAccT*MAX_DEACC_SPD;
						vctVL.push_back(fV);
						vctVR.push_back(fV);
					//	fLen+=fV*CMD_SLICE_LEN;
						fAccT+=CMD_SLICE_LEN;
					}

				}
		}
	}
	return 0;
}

int Trajectory::CalMidSpd(float fV0,float fVt,float fAcc,float fDeacc,float fDis,float &fMidSpd,float &fT1,float &fT2)
{
	float fTmp=fDeacc/fAcc;
	float fTmp2=fTmp-1;

	float fE=fVt*fVt-fV0*fV0;
	float fF=fE/(2*fAcc);
	float fD=fF-fDis;
	float fA=sqrt(fTmp2*fTmp2*fVt*fVt-2*fTmp2*fDeacc*((fVt*fVt-fV0*fV0)/(2*fAcc)-fDis));
	float fB=fTmp2*fVt;
	float fC=fDeacc*fTmp2;
	fT2=(fB+fA)/fC;
	fMidSpd=fVt-fDeacc*fT2;
	fT1=(fMidSpd-fV0)/fAcc;
	return 0;
}


int Trajectory::NewSegmentBlend(Point_f *stPt,vector<float> &vctWL,vector<float> &vctWR,float fInitSpd)
{
	int i,nSlice=100;
	float fRotX,fRotY,fTheta,fA0,fA1,fA2,fA3;
	float fBlendT1=(float)0.1,fBlendT2=(float)0.9;
	float fT=2,fTStep=fT/nSlice,fK1,fK2,fB2,fXL;
	float fX0,fX1,fY0,fY1,fXDiff,fMid;

	Point_f stNewPt[3];
	fRotX=stPt[2].fX-stPt[0].fX;
	fRotY=stPt[2].fY-stPt[0].fY;

	vctWL.clear();
	vctWR.clear();
	//nSlice=stPt[0].fLen/fInitSpd/CMD_SLICE_LEN;

	float fTmp1X=stPt[0].fX-stPt[1].fX;
	float fTmp1Y=stPt[0].fY-stPt[1].fY;
	
	float fTmp2X=stPt[2].fX-stPt[1].fX;
	float fTmp2Y=stPt[2].fY-stPt[1].fY;

	float fTmp1=sqrt(fTmp1X*fTmp1X+fTmp1Y*fTmp1Y);
	float fTmp2=sqrt(fTmp2X*fTmp2X+fTmp2Y*fTmp2Y);

	//nSlice=(stPt[0].fX-stPt[1].fX)*(stPt[0].fX-stPt[1].fX)+(stPt[0].fY-stPt[1].fY)*(stPt[0].fY-stPt[1].fY)
	nSlice=(fTmp1+fTmp2)/fInitSpd/CMD_SLICE_LEN;
	
	//printf("I wana know:  %f   %f   %d \n",stPt[0].fLen,fInitSpd,CMD_SLICE_LEN);

	if (fRotY!=0)
	{
		if (fRotX==0)
		{
			fTheta=(float)1.5707963;
		}
		else
		{
			fTheta=-atan2(fRotY,fRotX);
		}


		stNewPt[0].fX=0;
		stNewPt[0].fY=0;


		for (i=1;i<3;i++)
		{
			stNewPt[i].fX=cos(fTheta)*(stPt[i].fX-stPt[0].fX)
				-sin(fTheta)*(stPt[i].fY-stPt[0].fY);
			stNewPt[i].fY=sin(fTheta)*(stPt[i].fX-stPt[0].fX)
				+cos(fTheta)*(stPt[i].fY-stPt[0].fY);
		}
	}
	else
	{
		memcpy(&stNewPt,stPt,sizeof(Point_f)*3);
	}

	fK1=(stNewPt[0].fY-stNewPt[1].fY)/(stNewPt[0].fX-stNewPt[1].fX);
	fK2=(stNewPt[2].fY-stNewPt[1].fY)/(stNewPt[2].fX-stNewPt[1].fX);
	fB2=stNewPt[1].fY-fK2*stNewPt[1].fX;

	fXL=stNewPt[2].fX;


	fX0=fXL*fBlendT1;
	fY0=fK1*fX0;


	fX1=fBlendT2*fXL;
	fY1=fK2*fX1+fB2;


	fXDiff=fX1-fX0;

	fA0=fY0;
	fA1=fK1;
	fA2=3*(fY1-fY0)/fXDiff/fXDiff-2*fK1/fXDiff-fK2/fXDiff;
	fA3=-2*(fY1-fY0)/fXDiff/fXDiff/fXDiff+(fK1+fK2)/fXDiff/fXDiff;

	float fXStep=(stNewPt[2].fX-stNewPt[0].fX)/nSlice,fXAcc=0,fTmpX;


	vector<float> vctBlendPos;
	vector<float>  vctWheelL;
	vector<float> vctWheelR;
	vector<float> vctBlendPosX,vctBlendPosY;

	float fXD0,fXD1,fYD0,fYD1;
	for (i=0;i<nSlice;i++)
	{
		if (fXAcc<fX0)
		{
			vctBlendPos.push_back(fXAcc);
			vctBlendPos.push_back(fK1*fXAcc);

			vctBlendPosX.push_back(fXAcc);
			vctBlendPosY.push_back(fK1*fXAcc);
		}
		else if (fXAcc>=fX0&&fXAcc<=fX1)
		{
			fTmpX=fXAcc-fX0;
			vctBlendPos.push_back(fXAcc);
			vctBlendPos.push_back(fA0+fA1*fTmpX+fA2*fTmpX*fTmpX+fA3*fTmpX*fTmpX*fTmpX);

			vctBlendPosX.push_back(fXAcc);
			vctBlendPosY.push_back(fA0+fA1*fTmpX+fA2*fTmpX*fTmpX+fA3*fTmpX*fTmpX*fTmpX);
		}
		else
		{
			//	printf("Idx  :%d  \n",i);
			vctBlendPos.push_back(fXAcc);
			vctBlendPos.push_back(fK2*fXAcc+fB2);

			vctBlendPosX.push_back(fXAcc);
			vctBlendPosY.push_back(fK2*fXAcc+fB2);
		}
		fXAcc+=fXStep;
	}

	float fTheta1,fTheta2,fVel,fWL,fWR;
	for (i=2;i<nSlice;i++)
	{
		fXD0=vctBlendPos[i*2-2]-vctBlendPos[i*2-4];
		fXD1=vctBlendPos[i*2]-vctBlendPos[i*2-2];

		fYD0=vctBlendPos[i*2-1]-vctBlendPos[i*2-3];
		fYD1=vctBlendPos[i*2+1]-vctBlendPos[i*2-1];

		fTheta1= atan2(fYD0,fXD0);
		fTheta2= atan2(fYD1,fXD1);
		fTheta= fTheta2-fTheta1;
		fVel=cos(fTheta1)*fXD0+sin(fTheta1)*fYD0;

		//fWL=(fVel-fTheta)/2;
		//fWR=(fVel+fTheta)/2;

		fWL=(2*fVel-fTheta*ROBOT_L)/2;
		fWR=(2*fVel+fTheta*ROBOT_L)/2;

	//	printf("Idx :%d  ,Vel:%f  ,Theta:%f  ,fWL:%f ,fWR:%f\n",i,fVel,fTheta,fWL,fWR);

		vctWL.push_back(fWL);
		vctWR.push_back(fWR);

	}
	return 0;
}


int Trajectory::Spin(float fAngle)
{
	float fW=SPIN_ANGLE_VEL;
	float fSpinTime=fAngle/fW;

	float fWL,fWR;

	if(fAngle>0)
	{
		fWR=SPIN_ANGLE_VEL*ROBOT_L/2;
		fWL=-fWR;
	}
	else
	{
		fWL=SPIN_ANGLE_VEL*ROBOT_L/2;
		fWR=-fWL;
	}



	return 0;
}
