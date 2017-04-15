#include "BN.h"
BN *BN::m_PCBN=NULL;
BN::BN()
{
	m_pCBNpos=NULL;
	m_PCBN=this;
}

BN::~BN()
{
}

int BN::Init(BNParams stParams)
{

/*	printf("ROBOT ID:  %d, B:%d   ,Port:%s  \n",
				stParams.ucRobotID,stParams.uiBaudrate,stParams.cDev);
	m_pCBNpos=new BNpos((char*)&stParams.cDev[0],stParams.uiBaudrate,stParams.ucRobotID);
	m_pCBNpos->thread_close=0;*/
	RTN_OK;
}


int BN::Run()
{
	/*printf("BN run!!!!!\n");
	m_pCBNpos->listenUART();*/
	RTN_OK;
}


int BN::Stop()
{
//	m_pCBNpos->thread_close=1;
	RTN_OK;
}


int BN::Uninit()
{
	/*if(m_pCBNpos!=NULL)
	{
		delete m_pCBNpos;
		m_pCBNpos=NULL;
	} */
	RTN_OK;
}

int BN::BNLocation(float *pfLocation)
{

	float fX=0;
	float fY=0;
	float fTheta=0;
	memcpy(pfLocation,&fX,sizeof(float));
	memcpy(pfLocation+1,&fY,sizeof(float));
	memcpy(pfLocation+2,&fTheta,sizeof(float));


/*	float fX=m_PCBN->m_pCBNpos->robot.positionx;
	float fY=m_PCBN->m_pCBNpos->robot.positiony;
	float fTheta=m_PCBN->m_pCBNpos->robot.angle;

	memcpy(pfLocation,&fX,sizeof(float));
	memcpy(pfLocation+1,&fY,sizeof(float));

	memcpy(pfLocation+2,&fTheta,sizeof(float));

*/

	return RTN_OK;
}
