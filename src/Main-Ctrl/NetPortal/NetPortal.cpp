#include "NetPortal.h"
#include <unistd.h>
#include <string.h>

LaserCom NetPortal::m_CLaserCom[LASER_NUM];
CallBack_TaskIn NetPortal::m_cbTaskIn;
CallBack_LogFile NetPortal::m_cbLogFile;

pthread_t NetPortal::m_hThreadListen[3];
pthread_t NetPortal::m_hThreadReCNC[3];


ClientNet NetPortal::m_CClientNetA;
ClientNet NetPortal::m_CClientNetB;
ClientNet NetPortal::m_CClientNetMapBuilder;
char NetPortal::m_cCncStatus[3][2];
pthread_mutex_t NetPortal::m_MutexSendBLock;
pthread_mutex_t NetPortal::m_MutexSendALock;
pthread_mutex_t NetPortal::m_MutexSendMLock;
NetPortal *NetPortal::m_pCNetPortal;
NetPortal::NetPortal(void)
{
	pthread_mutex_init(&m_MutexSendALock,0);
	pthread_mutex_init(&m_MutexSendBLock,0);
	pthread_mutex_init(&m_MutexSendMLock,0);
	m_pCNetPortal=this;
}

NetPortal::~NetPortal(void)
{
}



int NetPortal::NetPortalInit(char *pcNetConfig)
{
	int i;
	char *pcLaserNetConfig=NULL;

	m_CClientNetA.m_cbNetUpload=ParseNetDataA;
	m_CClientNetB.m_cbNetUpload=ParseNetDataB;
	m_CClientNetMapBuilder.m_cbNetUpload=ParseNetDataM;

	m_CClientNetA.m_cbLogFile=m_cbLogFile;
	m_CClientNetB.m_cbLogFile=m_cbLogFile;

	for(i=0;i<3;i++)
	{
		memcpy(m_cNetConfig[i],pcNetConfig+16*i,16);
	}

	memset(m_cCncStatus,0,sizeof(m_cCncStatus));
	if(InitNetByClass(&m_CClientNetA,m_cNetConfig[0])==RTN_OK)
	{
		m_cCncStatus[0][0]=1;
		m_cCncStatus[0][1]=0;
	}

	sleep(2);

	if(InitNetByClass(&m_CClientNetB,m_cNetConfig[1])==RTN_OK)
	{
		m_cCncStatus[1][0]=1;
		m_cCncStatus[1][1]=0;
	}
	sleep(2);

	if(InitNetByClass(&m_CClientNetMapBuilder,m_cNetConfig[2])==RTN_OK)
	{
		m_cCncStatus[2][0]=1;
		m_cCncStatus[2][1]=0;
	}
	sleep(2);

	//printf("A Adrr:%d,B Adrr:%d,M Adrr:%d  \n",&m_CClientNetA,&m_CClientNetB,&m_CClientNetMapBuilder);


/*	if(m_CClientNetA.ClientNetInit(pcNetConfig)==RTN_OK)
	{
		usleep(1000);
	}
	else
	{
		printf("A Net Init failed!!!!\n");
	}

	if(m_CClientNetB.ClientNetInit(pcNetConfig+16)==RTN_OK)
	{

	}
	else
	{
		printf("B Net Init failed!!!!\n");
	}

	if(m_CClientNetMapBuilder.ClientNetInit(pcNetConfig+32)==RTN_OK)
	{

	}
	else
	{
		printf("B Net Init failed!!!!\n");
	}
*/
/*	j=32;
	for (i=0;i<LASER_NUM;i++)
	{
		if(m_CLaserCom[i].LaserComInit(pcNetConfig+j+i*16))
		{
			m_CLaserCom[i].m_cbLaserData=m_cbLaserData[i];
			CreateThread(NULL,0,
				(LPTHREAD_START_ROUTINE)ThreadLaserCom,
				&i,
				0,0);
		}
	}*/
	return RTN_OK;
}

int NetPortal::InitNetByClass(ClientNet *pCClientNet,char *pcNetConfig)
{
	return pCClientNet->ClientNetInit(pcNetConfig);
}


int NetPortal::NetPortalStop(char *pcNetConfig)
{
	m_CClientNetA.m_bStopListen=true;
	m_CClientNetB.m_bStopListen=true;
	m_CClientNetMapBuilder.m_bStopListen=true;

	printf("start 2 uninit net!!!!\n");
	m_CClientNetA.ClientNetUninit();
	m_CClientNetB.ClientNetUninit();
	m_CClientNetMapBuilder.ClientNetUninit();
	printf("start 2 uninit net OK!!!!\n");
	return RTN_OK;
}



int NetPortal::UploadA(char *pcData,int nDataLen)
{
	int nRtn=RTN_OK;
	pthread_mutex_lock(&m_MutexSendALock);
	if(m_cCncStatus[0][1]==0)
	{

		char cLen[4];
		memcpy(cLen,&nDataLen,4);

		if(m_CClientNetA.SendSLAMData(cLen,4)==0)
		{	
			if(m_CClientNetA.SendSLAMData(pcData,nDataLen)==0)
			{

			}
			else nRtn=-1;
		}
		else nRtn=-1;
	}
	else nRtn=-1;
	pthread_mutex_unlock(&m_MutexSendALock);
	return nRtn;

}

int NetPortal::UploadMapBuilder(char *pcData,int nDataLen)
{
	//printf("UploadMapBuilder11111\n");
	int nRtn=RTN_OK;
	pthread_mutex_lock(&m_MutexSendMLock);
	if(m_cCncStatus[2][1]==0)
	{
		char cLen[4];
		memcpy(cLen,&nDataLen,4);
		if(m_CClientNetMapBuilder.SendSLAMData(cLen,4)==0)
		{
			if(m_CClientNetMapBuilder.SendSLAMData(pcData,nDataLen)==0)
			{
			}
			else nRtn=-1;
		}
		else nRtn=-1;
	}
	else nRtn=-1;
	pthread_mutex_unlock(&m_MutexSendMLock);
	return nRtn;
}

int NetPortal::UploadB(char *pcData,int nDataLen)
{
	int nRtn=RTN_OK;
	pthread_mutex_lock(&m_MutexSendBLock);
	if(m_cCncStatus[1][1]==0)
	{
		char cLen[4];
		memcpy(cLen,&nDataLen,4);
		if(m_CClientNetB.SendSLAMData(cLen,4)==0)
		{
			if(m_CClientNetB.SendSLAMData(pcData,nDataLen)==0)
			{

			}
			else nRtn=-1;
		}
		else nRtn=-1;
	}
	else return nRtn=-1;
	pthread_mutex_unlock(&m_MutexSendBLock);
	return nRtn;
}

void* NetPortal::ThreadListen(void* lpParam)
{
	int nRtn;
	NP_CNCParams stCNCParams;
	memcpy(&stCNCParams,lpParam,sizeof(NP_CNCParams));

	pthread_detach(pthread_self());
	printf("listen ID %d \n",stCNCParams.cID);
//	stCNCParams.pCClientNet->m_bStopListen=false;

	if(stCNCParams.pCClientNet->Listen2Host()!=-1)
	{
			nRtn=RTN_OK;
			return &nRtn;
	}
	else
	{
			//������ʧȥ��ϵ,����ֹͣ�����˹���
		nRtn=RTN_LOSS_RS_CNC;
		printf("stCNCParams.cID:%d ,Lost CNC!!!!\n",stCNCParams.cID);
		//pNetPortal->m_CClientNetA.ClientNetUninit();
		//stCNCParams.pCClientNet->ClientNetUninit();
		stCNCParams.pCThis->m_cCncStatus[stCNCParams.cID][1]=RTN_LOSS_RS_CNC;
		printf("Lost CNC found!!!!\n",stCNCParams.cID);
		return &nRtn;
	}
}
/*
void* NetPortal::ThreadListenA(void* lpParam)
{
	int nRtn;
	NetPortal *pNetPortal=(NetPortal *)lpParam;
	if(pNetPortal->m_CClientNetA.Listen2Host()!=-1)
	{
		nRtn=RTN_OK;
		return &nRtn;
	}
	else
	{
		//������ʧȥ��ϵ,����ֹͣ�����˹���
		nRtn=RTN_LOSS_RS_CNC;
		pNetPortal->m_CClientNetA.ClientNetUninit();
		return &nRtn;
	}
}

void* NetPortal::ThreadListenB(void* lpParam)
{
	int nRtn;
	NetPortal *pNetPortal=(NetPortal *)lpParam;
	if(pNetPortal->m_CClientNetB.Listen2Host()!=-1)
	{
		nRtn=RTN_OK;
		return &nRtn;
	}
	else
	{
		//������ʧȥ��ϵ,����ֹͣ�����˹���
		nRtn=RTN_LOSS_RS_CNC;
		pNetPortal->m_CClientNetB.ClientNetUninit();
		return &nRtn;
	}
}

void* NetPortal::ThreadNetMapBuilder(void* lpParam)
{
	int nRtn;
	NetPortal *pNetPortal=(NetPortal *)lpParam;
	if(pNetPortal->m_CClientNetMapBuilder.Listen2Host()!=-1)
	{
		nRtn=RTN_OK;
		return &nRtn;
	}
	else
	{
		//������ʧȥ��ϵ,����ֹͣ�����˹���
		nRtn=RTN_LOSS_RS_CNC;
		pNetPortal->m_CClientNetMapBuilder.ClientNetUninit();
		return &nRtn;
	}
}
*/

void* NetPortal::ThreadRestartCnc(void* lpParam)
{
	int nRtn=RTN_OK;
	NP_CNCParams stCNCParams;
	memcpy(&stCNCParams,lpParam,sizeof(NP_CNCParams));

	printf("recnc %d \n",stCNCParams.cID);
	stCNCParams.pCThis->m_cCncStatus[stCNCParams.cID][1]=RTN_RS_CNC_RECNC;


	while(stCNCParams.pCThis->InitNetByClass
			(stCNCParams.pCClientNet,
			stCNCParams.pCThis->m_cNetConfig[stCNCParams.cID])!=RTN_OK
		)
	{
		usleep(10000);
	}


	stCNCParams.pCThis->m_hThreadListen[stCNCParams.cID]=0;
	if(pthread_create(&stCNCParams.pCThis->m_hThreadListen[stCNCParams.cID],NULL,ThreadListen,&stCNCParams)!=0)
	{
		printf("Create ThreadListenB Thread Failed!!!!\n");
	}

	stCNCParams.pCThis->m_cCncStatus[stCNCParams.cID][1]=RTN_OK;
	usleep(100);

	return &nRtn;
}


void* NetPortal::ThreadNetMonitor(void* lpParam)
{
	int nRtn=RTN_OK;
	NetPortal *pNetPortal=(NetPortal *)lpParam;
	//printf("ThreadNetMonitor000!!!\n");
	while(!pNetPortal->m_bSysClose)
	{
		//printf("ThreadNetMonitor!!!\n");
		if(pNetPortal->m_cCncStatus[0][0]==1
			&&pNetPortal->m_cCncStatus[1][0]==1
			&&pNetPortal->m_cCncStatus[2][0]==1)
		{
			//printf("ThreadNetMonitor22222222!!!\n");
			if(pNetPortal->m_cCncStatus[0][1]==RTN_LOSS_RS_CNC
				||pNetPortal->m_cCncStatus[1][1]==RTN_LOSS_RS_CNC
				||pNetPortal->m_cCncStatus[2][1]==RTN_LOSS_RS_CNC)
			{
				//printf("ThreadNetMonitor3333333333333!!!\n");
				pNetPortal->m_cCncStatus[0][1]=RTN_RS_CNC_RECNC;
				pNetPortal->m_cCncStatus[1][1]=RTN_RS_CNC_RECNC;
				pNetPortal->m_cCncStatus[2][1]=RTN_RS_CNC_RECNC;

				printf("NetPortalStop All !!!!\n");

				pNetPortal->NetPortalStop(NULL);

			/*	while(pNetPortal->m_cCncStatus[0][1]!=RTN_LOSS_RS_CNC
						&&pNetPortal->m_cCncStatus[1][1]!=RTN_LOSS_RS_CNC
						&&pNetPortal->m_cCncStatus[2][1]!=RTN_LOSS_RS_CNC)
				{
					printf("port1:%d****port2:%d****port3:%d  \n",
							pNetPortal->m_cCncStatus[0][1],
							pNetPortal->m_cCncStatus[1][1],
							pNetPortal->m_cCncStatus[2][1]);
					usleep(10000);
				}*/
				printf("all ports discnc !!!!\n");

				//sleep(6);

				while(pNetPortal->InitNetByClass(&pNetPortal->m_CClientNetA,pNetPortal->m_cNetConfig[0])!=RTN_OK)
				{
					usleep(10000);
				}
				sleep(1);
				while(pNetPortal->InitNetByClass(&pNetPortal->m_CClientNetB,pNetPortal->m_cNetConfig[1])!=RTN_OK)
				{
					usleep(10000);
				}
				sleep(1);
				while(pNetPortal->InitNetByClass(&pNetPortal->m_CClientNetMapBuilder,
						pNetPortal->m_cNetConfig[2])!=RTN_OK)
				{
					usleep(10000);
				}
				sleep(1);
				printf("all ports cnc host !!!!\n");

				NP_CNCParams stReCNCParams;
				stReCNCParams.cID=0;
				stReCNCParams.pCClientNet=&pNetPortal->m_CClientNetA;
				stReCNCParams.pCThis=pNetPortal;
				pNetPortal->m_hThreadListen[0]=0;
				if(pthread_create(&pNetPortal->m_hThreadListen[0],NULL,ThreadListen,&stReCNCParams)!=0)
				{
					printf("Create ThreadListenB Thread Failed!!!!\n");
				}
				sleep(1);
				stReCNCParams.cID=1;
				stReCNCParams.pCClientNet=&pNetPortal->m_CClientNetB;
				stReCNCParams.pCThis=pNetPortal;
				pNetPortal->m_hThreadListen[1]=0;
				if(pthread_create(&pNetPortal->m_hThreadListen[1],NULL,ThreadListen,&stReCNCParams)!=0)
				{
					printf("Create ThreadListenB Thread Failed!!!!\n");
				}
				sleep(1);
				stReCNCParams.cID=2;
				stReCNCParams.pCClientNet=&pNetPortal->m_CClientNetMapBuilder;
				stReCNCParams.pCThis=pNetPortal;
				pNetPortal->m_hThreadListen[2]=0;
				if(pthread_create(&pNetPortal->m_hThreadListen[2],NULL,ThreadListen,&stReCNCParams)!=0)
				{
					printf("Create ThreadListenB Thread Failed!!!!\n");
				}
				sleep(1);
				printf("all ports listen host !!!!\n");
				
				pNetPortal->m_cCncStatus[0][1]=RTN_OK;
				pNetPortal->m_cCncStatus[1][1]=RTN_OK;
				pNetPortal->m_cCncStatus[2][1]=RTN_OK;
			}
		}
	/*	if(pNetPortal->m_cCncStatus[0][0])
		{
			if(pNetPortal->m_cCncStatus[0][1]==RTN_LOSS_RS_CNC)
			{
				pNetPortal->m_cCncStatus[0][1]=RTN_RS_CNC_RECNC;
				NP_CNCParams stCNCParams;
				stCNCParams.cID=0;
				stCNCParams.pCClientNet=&pNetPortal->m_CClientNetA;
				stCNCParams.pCThis=pNetPortal;
				ReCNCAll((void*)&stCNCParams);
				//printf("Idx :0  Lost CNC,try to reCNC!!!!!!!!!\n");
				//if(pthread_create(&m_hThreadReCNC[0],NULL,ReCNCAll,&stCNCParams)!=0)
				//{
				//	printf("Create ThreadListenB Thread Failed!!!!\n");
				//}
				usleep(100000);
			}
		}

		if(pNetPortal->m_cCncStatus[1][0])
		{
			if(pNetPortal->m_cCncStatus[1][1]==RTN_LOSS_RS_CNC)
			{
				pNetPortal->m_cCncStatus[1][1]=RTN_RS_CNC_RECNC;
				NP_CNCParams stCNCParams;
								stCNCParams.cID=1;
								stCNCParams.pCClientNet=&pNetPortal->m_CClientNetB;
								stCNCParams.pCThis=pNetPortal;
								printf("Idx :1  Lost CNC,try to reCNC!!!!!!!!!\n");
								//if(pthread_create(&m_hThreadReCNC[0],NULL,ReCNCAll,&stCNCParams)!=0)
								//{
								//	printf("Create ThreadListenB Thread Failed!!!!\n");
								//}
				ReCNCAll((void*)&stCNCParams);
								usleep(100000);
			}
		}

		if(pNetPortal->m_cCncStatus[2][0])
		{
			if(pNetPortal->m_cCncStatus[2][1]==RTN_LOSS_RS_CNC)
			{
				NP_CNCParams stCNCParams;
				stCNCParams.cID=2;
				stCNCParams.pCClientNet=&m_CClientNetMapBuilder;
				stCNCParams.pCThis=pNetPortal;
				printf("Idx :2  Lost CNC,try to reCNC!!!!!!!!!\n");
				//if(pthread_create(&m_hThreadReCNC[0],NULL,ReCNCAll,&stCNCParams)!=0)
				//{
				//	printf("Create ThreadListenB Thread Failed!!!!\n");
				//}
				ReCNCAll((void*)&stCNCParams);
				usleep(100000);
			}
		}*/
		/*if(pNetPortal->m_cCncStatus[0][0])
		{
			if(pNetPortal->m_cCncStatus[0][1]==RTN_LOSS_RS_CNC)
			{
				pNetPortal->m_cCncStatus[0][1]=RTN_RS_CNC_RECNC;
				NP_CNCParams stCNCParams;
				stCNCParams.cID=0;
				stCNCParams.pCClientNet=&pNetPortal->m_CClientNetA;
				stCNCParams.pCThis=pNetPortal;

				printf("Idx :0  Lost CNC,try to reCNC!!!!!!!!!\n");
				if(pthread_create(&m_hThreadReCNC[0],NULL,ThreadRestartCnc,&stCNCParams)!=0)
				{
					printf("Create ThreadListenB Thread Failed!!!!\n");
				}
				usleep(100000);
			}
		}

		if(pNetPortal->m_cCncStatus[1][0])
		{
			if(pNetPortal->m_cCncStatus[1][1]==RTN_LOSS_RS_CNC)
			{
				pNetPortal->m_cCncStatus[1][1]=RTN_RS_CNC_RECNC;
				NP_CNCParams stCNCParams;
								stCNCParams.cID=1;
								stCNCParams.pCClientNet=&pNetPortal->m_CClientNetB;
								stCNCParams.pCThis=pNetPortal;
								printf("Idx :1  Lost CNC,try to reCNC!!!!!!!!!\n");
								if(pthread_create(&m_hThreadReCNC[0],NULL,ThreadRestartCnc,&stCNCParams)!=0)
								{
									printf("Create ThreadListenB Thread Failed!!!!\n");
								}
								usleep(100000);
			}
		}

		if(pNetPortal->m_cCncStatus[2][0])
		{
			if(pNetPortal->m_cCncStatus[2][1]==RTN_LOSS_RS_CNC)
			{
				pNetPortal->m_cCncStatus[2][1]=RTN_RS_CNC_RECNC;
				NP_CNCParams stCNCParams;
								stCNCParams.cID=2;
								stCNCParams.pCClientNet=&m_CClientNetMapBuilder;
								stCNCParams.pCThis=pNetPortal;
								printf("Idx :2  Lost CNC,try to reCNC!!!!!!!!!\n");
								if(pthread_create(&m_hThreadReCNC[0],NULL,ThreadRestartCnc,&stCNCParams)!=0)
								{
									printf("Create ThreadListenB Thread Failed!!!!\n");
								}
								usleep(100000);
			}
		}*/
		sleep(1);
	}
	return &nRtn;
}

void* NetPortal::ThreadLaserCom(void* lpParam)
{
	int nRtn;
	int LaserIdx=*((int*)lpParam);
	
	if (m_CLaserCom[LaserIdx].LaserComRun()!=-1)
	{
		nRtn=RTN_OK;
		return &nRtn;
	}
	else
	{
		nRtn=RTN_LOSS_RS_CNC;
		return &nRtn;
	}
}

int NetPortal::NetPortalRun()
{
	/*CreateThread(NULL,0,
		(LPTHREAD_START_ROUTINE)ThreadListenA,
		this,
		0,0);
	usleep(20);

	CreateThread(NULL,0,
		(LPTHREAD_START_ROUTINE)ThreadListenB,
		this,
		0,0);
	usleep(20);*/

	//m_CClientNetA.m_bStopListen=true;
	//m_CClientNetB.m_bStopListen=true;
	//m_CClientNetMapBuilder.m_bStopListen=true;



	NP_CNCParams stCNCParams;
	stCNCParams.cID=0;
	stCNCParams.pCClientNet=&m_CClientNetA;
	stCNCParams.pCThis=this;
	if(pthread_create(&m_hThreadListen[0],NULL,ThreadListen,&stCNCParams)!=0)
	{
		printf("Create ThreadListenA Thread Failed!!!!\n");
	}
	sleep(1);

	stCNCParams.cID=1;
	stCNCParams.pCClientNet=&m_CClientNetB;
	stCNCParams.pCThis=this;
	if(pthread_create(&m_hThreadListen[1],NULL,ThreadListen,&stCNCParams)!=0)
	{
		printf("Create ThreadListenB Thread Failed!!!!\n");
	}
	sleep(1);

	stCNCParams.cID=2;
	stCNCParams.pCClientNet=&m_CClientNetMapBuilder;
	stCNCParams.pCThis=this;
	if(pthread_create(&m_hThreadListen[2],NULL,ThreadListen,&stCNCParams)!=0)
	{
		printf("Create ThreadListenB Thread Failed!!!!\n");
	}
	sleep(1);

	if(pthread_create(&m_hThreadHeartBit,NULL,ThreadHeartBit,this)!=0)
	{
		printf("Create ThreadListenB Thread Failed!!!!\n");
	}
	usleep(100000);

//	printf("create m_hThreadNetMonitor!!!!\n");

	if(pthread_create(&m_hThreadNetMonitor,NULL,ThreadNetMonitor,this)!=0)
	{
		printf("Create ThreadListenB Thread Failed!!!!\n");
	}
	sleep(1);



	

/*	for (i=0;i<LASER_NUM;i++)
	{
		m_CLaserCom[i].LaserComRun();
	}*/
	return 0;
}

int NetPortal::ParseNetDataM(char *pcData,int nDataLen)
{
	int nCmd;

		memcpy(&nCmd,pcData,4);
	//	printf("M  CMD:%d \n",nCmd);
		if(nCmd==HEART_BIT)
		{
		//	printf("M:  HEART_BIT\n");
		}
		return 0;
}

int NetPortal::ParseNetDataA(char *pcData,int nDataLen)
{
	int nCmd;
	//printf("CMD:%d \n",nCmd);
	memcpy(&nCmd,pcData,4);
//	printf("A  CMD:%d \n",nCmd);
	if(nCmd==HEART_BIT)
	{
	//	printf("A:  HEART_BIT\n");
	}
	return 0;
}

int NetPortal::ParseNetDataB(char *pcData,int nDataLen)
{
	int nCmd;
	memcpy(&nCmd,pcData,4);

//	printf("B  CMD:%d \n",nCmd);

	if (nCmd==NEW_TASK_PATH)
	{
		printf("why###################################\n");
		m_cbTaskIn(1,pcData+4,nDataLen-4);
	}
	else if(nCmd==SLOW_BREAK)
	{
	//	printf("hahahah Why!!\n");
		m_cbTaskIn(10,NULL,0);
	}
	else if(nCmd==GRID_MAP_IN)
	{
		m_cbTaskIn(nCmd,pcData+4,nDataLen-4);
	}
	else if(nCmd==RE_TASK_PATH)
	{
			m_cbTaskIn(RE_TASK_PATH,pcData+4,nDataLen-4);
	}
	else if(nCmd==HEART_BIT)
	{
		//printf("B:  HEART_BIT\n");
	}
	return 0;
}

int NetPortal::ParseNetDataL(char *pcData,int nDataLen)
{
	return 0;
}
void* NetPortal::ThreadNetMapBuilder(void* lpParam)
{

}


void* NetPortal::ReCNCAll(void* lpParam)
{
	int nRtn=RTN_OK;
	NP_CNCParams stCNCParams;
	memcpy(&stCNCParams,lpParam,sizeof(NP_CNCParams));

	printf("recnc %d \n",stCNCParams.cID);
	stCNCParams.pCThis->m_cCncStatus[0][1]=RTN_RS_CNC_RECNC;
	stCNCParams.pCThis->m_cCncStatus[1][1]=RTN_RS_CNC_RECNC;
	stCNCParams.pCThis->m_cCncStatus[2][1]=RTN_RS_CNC_RECNC;

	stCNCParams.pCThis->NetPortalStop(NULL);

	while(stCNCParams.pCThis->m_cCncStatus[0][1]!=RTN_LOSS_RS_CNC
		&&stCNCParams.pCThis->m_cCncStatus[1][1]!=RTN_LOSS_RS_CNC
		&&stCNCParams.pCThis->m_cCncStatus[2][1]!=RTN_LOSS_RS_CNC)
	{
		usleep(10000);
	}

	if(stCNCParams.pCThis->InitNetByClass
			(stCNCParams.pCClientNet,
			stCNCParams.pCThis->m_cNetConfig[0])!=RTN_OK
		)
	{
		usleep(10000);
	}

	if(stCNCParams.pCThis->InitNetByClass
			(stCNCParams.pCClientNet,
			stCNCParams.pCThis->m_cNetConfig[1])!=RTN_OK
		)
	{
		usleep(10000);
	}

	if(stCNCParams.pCThis->InitNetByClass
			(stCNCParams.pCClientNet,
			stCNCParams.pCThis->m_cNetConfig[2])!=RTN_OK
		)
	{
		usleep(10000);
	}

	NP_CNCParams stReCNCParams;
	stReCNCParams.cID=0;
	stReCNCParams.pCClientNet=&stCNCParams.pCThis->m_CClientNetA;
	stReCNCParams.pCThis=stCNCParams.pCThis;

	stCNCParams.pCThis->m_hThreadListen[0]=0;
	if(pthread_create(&stCNCParams.pCThis->m_hThreadListen[0],NULL,ThreadListen,&stReCNCParams)!=0)
	{
		printf("Create ThreadListenB Thread Failed!!!!\n");
	}
	sleep(1);
	stReCNCParams.cID=1;
	stReCNCParams.pCClientNet=&stCNCParams.pCThis->m_CClientNetB;
	stReCNCParams.pCThis=stCNCParams.pCThis;
	stCNCParams.pCThis->m_hThreadListen[1]=0;
	if(pthread_create(&stCNCParams.pCThis->m_hThreadListen[1],NULL,ThreadListen,&stReCNCParams)!=0)
	{
		printf("Create ThreadListenB Thread Failed!!!!\n");
	}
	sleep(1);
	stReCNCParams.cID=2;
	stReCNCParams.pCClientNet=&stCNCParams.pCThis->m_CClientNetMapBuilder;
	stReCNCParams.pCThis=stCNCParams.pCThis;
	stCNCParams.pCThis->m_hThreadListen[2]=0;
	if(pthread_create(&stCNCParams.pCThis->m_hThreadListen[2],NULL,ThreadListen,&stReCNCParams)!=0)
	{
		printf("Create ThreadListenB Thread Failed!!!!\n");
	}
	sleep(1);
	/*while(stCNCParams.pCThis->InitNetByClass
			(stCNCParams.pCClientNet,
			stCNCParams.pCThis->m_cNetConfig[stCNCParams.cID])!=RTN_OK
		)
	{
		usleep(10000);
	}


	stCNCParams.pCThis->m_hThreadListen[stCNCParams.cID]=0;
	if(pthread_create(&stCNCParams.pCThis->m_hThreadListen[stCNCParams.cID],NULL,ThreadListen,&stCNCParams)!=0)
	{
		printf("Create ThreadListenB Thread Failed!!!!\n");
	}*/

	stCNCParams.pCThis->m_cCncStatus[0][1]=RTN_OK;
	stCNCParams.pCThis->m_cCncStatus[1][1]=RTN_OK;
	stCNCParams.pCThis->m_cCncStatus[2][1]=RTN_OK;
	usleep(100);
}

void* NetPortal::ThreadHeartBit(void* lpParam)
{
	char cCMD[4];
	int nCMD=HEART_BIT;
	memcpy(cCMD,&nCMD,4);
	int nRtn;
	while(1)
	{
		if(m_cCncStatus[0][1]==RTN_OK
				&&m_cCncStatus[1][1]==RTN_OK
				&&m_cCncStatus[2][1]==RTN_OK)
		{
			nRtn=UploadMapBuilder(cCMD,4);
			//printf("UploadMapBuilder:%d  \n",nRtn);
			nRtn=UploadA(cCMD,4);
			//printf("UploadA:%d  \n",nRtn);
			nRtn=UploadB(cCMD,4);
			//printf("UploadB:%d  \n",nRtn);
		}

		usleep(300000);
	}
	return 0;
}
