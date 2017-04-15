#include "C_C.h"
#include "slam_v1.h"
#include <semaphore.h>
float C_C::m_fCurPos[3];
C_C *C_C::m_pCC_CThis;
float C_C::m_fOdometryData[4];

int C_C::m_nSysErrList;
float C_C::m_fVoltBatteryCtrl;
float C_C::m_fVoltBatteryPower;
float C_C::m_fGYRO_Global;
float C_C::m_fSICK_IO_Read;
float C_C::m_fTaskParams[14];
float C_C::m_fOffset[3];
int C_C::m_nTrustData;
int C_C::m_nRefreshPCOnRS;
float  C_C::m_fScanData[541];

vector<float>  C_C::m_vctPC;

CSlamV1* C_C::m_pCSLAM;


pthread_mutex_t g_ErrListLock;
pthread_mutex_t g_FusionDataAndPCLock;
sem_t g_SndPCSgl;

int C_C::cbCtrlCmdParse(float x,float y,float th)
{
	static int cnt = 0;
	char cTmp[12];
	x*=100;
	y*=100;
	memcpy(cTmp,&x,4);
	memcpy(cTmp+4,&y,4);
	memcpy(cTmp+8,&th,4);
	memcpy(m_fCurPos,cTmp,12);


	//NetPortal::UploadA(cTmp,12);
	return 0;
}




C_C::C_C(void)
{
	m_pCC_CThis=this;
}

C_C::~C_C(void)
{
}

int C_C::C_CInit()
{
	m_nSysErrList=0;
	pthread_mutex_init(&g_ErrListLock,0);
	pthread_mutex_init(&g_FusionDataAndPCLock,0);
	char cChassisPort[100];
	memset(cChassisPort,0,100);
	memcpy(cChassisPort,"/dev/",5);
	GetSerialPortNum("FTDI USB Serial Device converter now attached to ",cChassisPort+5);
	printf("cChassisPort:%s  \n",cChassisPort);
	//if(strlen(cChassisPort)==5)m_nSysErrList=SYS_LOST_LOW_CTRL_SERIAL;

	char cBNPort[100];
	memset(cBNPort,0,100);
	memcpy(cBNPort,"/dev/",5);
	GetSerialPortNum("pl2303 converter now attached to ",cBNPort+5);


	//if(strlen(cBNPort)==5)m_nSysErrList=SYS_LOST_BN_SERIAL;
	printf("cBNPort:%s  \n",cBNPort);
	//init BN
#ifdef OPEN_BN
	BNParams stBNParams;


	//char *pcBNDev="/dev/ttyUSB0";
	memset(stBNParams.cDev,0,100);
	//memcpy(stBNParams.cDev,pcBNDev,strlen(pcBNDev));
	memcpy(stBNParams.cDev,cBNPort,strlen(cBNPort));
	stBNParams.uiBaudrate=57600;
	stBNParams.ucRobotID=16;//18;
	m_CBN.Init(stBNParams);
#endif


	printf("C_C init start!!!!\n");
	char cTmp[1000];
	unsigned char *pucTmp=NULL;
	memset(cTmp,0,1000);
	LoadConfig("Conf.xml");

	signal(SIGPIPE,SIG_IGN);
	m_CLogFile.LogFileInit(m_pcLogFilePath);
	if(pthread_create(&m_hThreadLogFile,NULL,ThreadLogFile,this)!=0)
	{
		printf("Create ThreadLogFile Thread Failed!!!!\n");
	}
	usleep(100);


	m_CTask.TaskInit(m_fTaskParams);


	printf("C_C init ThreadLogFile!!!!\n");


	m_CNetPortal.m_cbLaserData[0]=NULL;
	m_CNetPortal.m_cbLaserData[1]=NULL;

	m_CNetPortal.m_cbLogFile=LogFile::PushContent2LogStack;
	//m_CNetPortal.m_cbTaskIn=m_CTask.TaskIn;

	m_CNetPortal.m_cbTaskIn=m_CTask.NewTaskIn;
	m_CNetPortal.NetPortalInit(m_cNetPortalParams);
	printf("C_C init LaserData!!!!\n");

	m_CSubCtrlCom.SubCtrlComInit(cChassisPort);
	m_CSubCtrlCom.m_cbBNlctAndOdometry=NULL;
	m_CSubCtrlCom.m_cbLogFile=LogFile::PushContent2LogStack;

	m_CSubCtrlCom.m_cbLaserDtct[0]=NULL;
	m_CSubCtrlCom.m_cbLaserDtct[1]=NULL;
	printf("C_C init SubCtrlComInit!!!!\n");

	m_CIOA.IOAInit();
	m_CIOA.m_stSimpleCtrlCmd.cbBackward=SubCtrlCom::Backward;
	m_CIOA.m_stSimpleCtrlCmd.cbForward=SubCtrlCom::Forward;
	m_CIOA.m_stSimpleCtrlCmd.cbBreak=SubCtrlCom::Break;
	m_CIOA.m_stSimpleCtrlCmd.cbTurn=SubCtrlCom::Turn;
	m_CIOA.m_cbLogFile=LogFile::PushContent2LogStack;

	printf("C_C init IOAInit!!!!\n");


	SLAMParams stSLAMParams;

	memset(stSLAMParams.cHost_IP_A,0,16);
	memset(stSLAMParams.cSICK_IP_A,0,16);
	memset(stSLAMParams.cHost_IP_B,0,16);
	memset(stSLAMParams.cSICK_IP_B,0,16);

	pucTmp=(unsigned char *)&m_cNetPortalParams[32];
	sprintf(stSLAMParams.cSICK_IP_A, "%d.%d.%d.%d", pucTmp[0],pucTmp[1],pucTmp[2],pucTmp[3]);
	pucTmp+=4;
	memcpy(&stSLAMParams.nHost_Port_A,pucTmp,4);
	pucTmp+=4;
	sprintf(stSLAMParams.cHost_IP_A, "%d.%d.%d.%d", pucTmp[0],pucTmp[1],pucTmp[2],pucTmp[3]);
	pucTmp+=4;
	memcpy(&stSLAMParams.nHost_Port_A,pucTmp,4);


	pucTmp+=4;
	sprintf(stSLAMParams.cSICK_IP_B, "%d.%d.%d.%d", pucTmp[0],pucTmp[1],pucTmp[2],pucTmp[3]);
	pucTmp+=4;
	memcpy(&stSLAMParams.nHost_Port_B,pucTmp,4);
	pucTmp+=4;
	sprintf(stSLAMParams.cHost_IP_B, "%d.%d.%d.%d", pucTmp[0],pucTmp[1],pucTmp[2],pucTmp[3]);
	pucTmp+=4;
	memcpy(&stSLAMParams.nHost_Port_B,pucTmp,4);


	memset(&m_stSLAM_CallBack,0,sizeof(SLAM_CallBack));
	m_stSLAM_CallBack.cbOdometry=ReadOdometry;
	m_stSLAM_CallBack.cbBNLocation=BN::BNLocation;
	m_stSLAM_CallBack.cbMainSICKForSLAM=m_DuoSick.getScan_A_SLAM;
	m_stSLAM_CallBack.cbMainSICKForOD=m_DuoSick.getScan_A_OD;
	m_stSLAM_CallBack.cbMinorSICKForSLAM=m_DuoSick.getScan_B_SLAM;
	m_stSLAM_CallBack.cbMinorSICKForOD=m_DuoSick.getScan_B_OD;

	m_stSLAM_CallBack.cbOnlyBNResult=cbOnlyBNRslt;
	m_stSLAM_CallBack.cbOnlyOdoResult=cbOnlyOdoRslt;
	m_stSLAM_CallBack.cbOnlySLAMResult=cbOnlySLAMRslt;
	m_stSLAM_CallBack.cbDataFusionAndPC=cbDataFusionAndPC;
	// m_stSLAM_CallBack.cbSICKA=IOA:: GetLaserAData;
	// m_stSLAM_CallBack.cbSICKB=IOA:: GetLaserBData;


	//m_stSLAM_CallBack.cbDataFusionResult=IOA::DataFusionResult;
	m_stSLAM_CallBack.cbLocalMap=NULL;

	//m_stSLAM_CallBack.cbErrList=ErrList;
	//m_CSLAMTest.Init(m_stSLAM_CallBack,stSLAMParams);


	//	m_CSLAM.m_cbNetUpload=NetPortal::UploadA;
	//	m_CSLAM.m_cbLogFile=LogFile::PushContent2LogStack;
	m_pCSLAM=new CSlamV1;
	m_pCSLAM->init(&m_stSLAM_CallBack,NULL);

	//1 read Txt
	//2 Sick
	//3 fusion
	//4 Odo
	//6 BN

	//	m_pCSLAM->m_work_model=1;

	m_pCSLAM->m_work_model=(int)(m_fSLAMParams[7]+0.5);

	//m_pCSLAM->m_work_model=2;
	printf("m_pCSLAM->m_work_model:%d  \n",m_pCSLAM->m_work_model);
	m_pCSLAM->setSystem();

	//init GPS
	printf("C_C GPS Init!!!!\n");
	//m_CGPS.start("/dev/ttyUSB0");

	printf("C_C BN Init!!!!\n");


	m_CAmbientGridMap.AGMInit();

	m_CTask.m_cbRobotPos=cbCtrlCmdParse;
	m_CTask.m_cbSend2RS_A=NetPortal::UploadA;
	m_CTask.m_cbSend2RS_B=NetPortal::UploadB;

	//	printf("A Addr:%d  ,B Addr:%d M Addr:%d \n",NetPortal::UploadA,NetPortal::UploadB,NetPortal::UploadMapBuilder);
	m_CTask.m_cbSendNKJCMD=SubCtrlCom::SendNKJCmd;
	m_CTask.m_cbSendNKJCMDRot=SubCtrlCom::SendNKJCmd_Rot;

	//	m_CSubCtrlCom.SubCtrlComInit();
	m_fOdometryData[0]=0;
	m_fOdometryData[1]=0;
	m_fOdometryData[2]=0;


	m_CAmbientGridMap.m_cbSend2RS_B=NetPortal::UploadB;

	m_CSubCtrlCom.SendNKJCmd_ClearOdo();
	sem_init(&g_SndPCSgl,0,0);

	if(m_nRefreshPCOnRS==0)
	{
		char cCMD[4];
		int nCMD=CLEAR_PC_DATA;
		memcpy(cCMD,&nCMD,4);
		NetPortal::UploadB(cCMD,4);
	}
	return 0;
}

int C_C::C_CRun(int argc, char* argv[])
{

	if(pthread_create(&m_hThreadMainSick,NULL,ThreadMainSick,this)!=0)
	{
		printf("Create ThreadMainSick Thread Failed!!!!\n");
	}
	usleep(100000);

	if(pthread_create(&m_hThreadMinorSick,NULL,ThreadMinorSick,this)!=0)
	{
		printf("Create ThreadMinorSick Thread Failed!!!!\n");
	}
	usleep(100000);

	if(pthread_create(&m_hThreadNetPortal,NULL,ThreadNetPortal,this)!=0)
	{
		printf("Create ThreadNetPortal Thread Failed!!!!\n");
	}
	usleep(100000);

	if(pthread_create(&m_hThreadSLAM,NULL,ThreadSLAM,this)!=0)
	{
		printf("Create ThreadSLAM Thread Failed!!!!\n");
	}
	usleep(100000);


	if(pthread_create(&m_hThreadIOA,NULL,ThreadIOA,this)!=0)
	{
		printf("Create ThreadSLAM Thread Failed!!!!\n");
	}
	usleep(100000);


	if(pthread_create(&m_hThreadSubCtrlCom,NULL,ThreadSubCtrlCom,this)!=0)
	{
		printf("Create ThreadSubCtrlCom Thread Failed!!!!\n");
	}
	usleep(100000);

	if(pthread_create(&m_hReadRobotStatus,NULL,ThreadReadRobotStatus,this)!=0)
	{
		printf("Create ThreadSubCtrlCom Thread Failed!!!!\n");
	}
	usleep(100000);

	if(pthread_create(&m_hThreadSystemMonitor,NULL,ThreadSystemMonitor,this)!=0)
	{
		printf("Create ThreadSubCtrlCom Thread Failed!!!!\n");
	}
	usleep(100000);

	/*if(pthread_create(&m_hThreadUploadPointCloud,NULL,ThreadUploadPointCloud,this)!=0)
	{
		printf("Create ThreadSLAM Thread Failed!!!!\n");
	}
	usleep(100000);*/

	if(pthread_create(&m_hThreadAskRobotStatus,NULL,ThreadAskRobotStatus,this)!=0)
	{
		printf("Create ThreadSLAM Thread Failed!!!!\n");
	}
	usleep(100000);

	if(pthread_create(&m_hThreadNetMonitor,NULL,ThreadNetMonitor,this)!=0)
	{
		printf("Create ThreadNetPortal Thread Failed!!!!\n");
	}
	usleep(100000);

	if(pthread_create(&m_hThreadGetScanData,NULL,ThreadGetScanData,this)!=0)
	{
		printf("Create ThreadNetPortal Thread Failed!!!!\n");
	}
	usleep(100000);

	if(pthread_create(&m_hThreadUploadDF_PC,NULL,ThreadUploadDF_PC,this)!=0)
	{
		printf("Create ThreadNetPortal Thread Failed!!!!\n");
	}
	usleep(100000);


#ifdef OPEN_BN
	if(pthread_create(&m_hThreadBN,NULL,ThreadBN,this)!=0)
	{
		printf("Create ThreadBN Thread Failed!!!!\n");
	}
#endif


	usleep(100);

	m_CTask.TaskRun();

	printf("System Start 2 Run!!!!\n");
	return 0;
}

SLAM_CallBack cbCB;

int main(int argc, char* argv[] )
{

	sleep(2);
	C_C test;


	//cbCB.cbDataFusionResult = cbSLAM;
	test.C_CInit();
	test.C_CRun(argc, argv);
	//test.m_pCSLAM->init(&cbCB,NULL);
	//test.m_pCSLAM->start();
	//test.m_pCSLAM->startEventLoop(argc,argv);
	printf("main-ctrl start to run!!!!\n");
	/*	while (test.m_nSysErr==0)
	{
		sleep(60);
	}*/

#ifdef OPEN_SELF_CHECK
	while(test.m_nSysErrList!=0)
	{
		sleep(1);
	}
#else
	while(1)
	{
		sleep(10000);
	}
#endif

	//printf("SYSTEM SHUT DOWN, REASON CODE:%n\n",test.m_nSysErrList);
	return 0;
}


int C_C::LoadConfig(char *pcConfigPath)
{
	//ParseXML CParseXML;
	char cIP[4],cHostIP[4];
	int i,j,nPort,nHostPort;
	m_CParseXML.ParseXMLRun(pcConfigPath);
	j=0;
	for (i=0;i<5;i++)
	{
		ParseIP((char *)m_CParseXML.m_vctData[4*i].c_str(),cIP);
		nPort=atoi(m_CParseXML.m_vctData[4*i+1].c_str());
		ParseIP((char *)m_CParseXML.m_vctData[4*i+2].c_str(),cHostIP);
		nHostPort=atoi(m_CParseXML.m_vctData[4*i+3].c_str());

		memcpy(m_cNetPortalParams+j,cIP,4);
		j+=4;
		memcpy(m_cNetPortalParams+j,&nPort,4);
		j+=4;
		memcpy(m_cNetPortalParams+j,cHostIP,4);
		j+=4;
		memcpy(m_cNetPortalParams+j,&nHostPort,4);
		j+=4;
	}

	m_pcLogFilePath=(char*)m_CParseXML.m_vctData[20].c_str();
	m_ucRobotID=atoi(m_CParseXML.m_vctData[21].c_str());

	j=0;
	_EXTERN_SLAM_PARAM = true;
	//for(i=22;i<m_CParseXML.m_vctData.size();i++)
	for(i=22;i<37;i++)
	{
		m_fSLAMParams[j++]=atof(m_CParseXML.m_vctData[i].c_str());
	}
	j=0;
	for(i=37;i<51;i++)
	{
		m_fTaskParams[j++]=atof(m_CParseXML.m_vctData[i].c_str());
		printf("m_fIOAParams:%f  \n",m_fTaskParams[j-1]);
	}


	float fTmp;

	//Trust Location Src
	//0  Fusion Data
	//1  SLAM Rslt
	//2  Odo
	//3  BN
	fTmp=atof(m_CParseXML.m_vctData[51].c_str());
	m_nTrustData=(int)(fTmp+0.5);

	printf("m_nTrustData:%d  \n",m_nTrustData);

	fTmp=atof(m_CParseXML.m_vctData[52].c_str());
	m_nRefreshPCOnRS=(int)(fTmp+0.5);

	printf("m_nRefreshPCOnRS:%d  \n",m_nRefreshPCOnRS);

	m_fOffset[0]=m_fSLAMParams[4];
	m_fOffset[1]=m_fSLAMParams[5];
	m_fOffset[2]=m_fSLAMParams[6];

	return 0;
}


int C_C::ParseIP(char *pcData,char *pcIP)
{
	char cTmp[10];
	int i,j=0,k=0;
	for (i=0;i<(int)strlen(pcData);i++)
	{
		if (pcData[i]=='.')
		{
			pcIP[k++]=atoi(cTmp);
			j=0;
			memset(cTmp,0,10);

		}
		else
		{
			cTmp[j++]=pcData[i];
		}
	}
	pcIP[k]=atoi(cTmp);

	return 0;
}



void* C_C::ThreadSLAM(void* lpParam)
{
	pthread_detach(pthread_self());
	C_C * pCC=(C_C *)lpParam;
	//pCC->m_CSLAMTest.Run();

	// method to call slam interface


	int argc=0;
	//char* argv[]=0;
	char** pTmp=NULL;
	pCC->m_pCSLAM->start();
	pCC->m_pCSLAM->startEventLoop(argc,pTmp);
	return 0;
}



void* C_C::ThreadNetPortal(void* lpParam)
{
	pthread_detach(pthread_self());
	C_C * pCC=(C_C *)lpParam;
	pCC->m_CNetPortal.m_bSysClose=false;
	pCC->m_CNetPortal.NetPortalRun();

	return 0;
}

void* C_C::ThreadBN(void* lpParam)
{
	pthread_detach(pthread_self());
	C_C * pCC=(C_C *)lpParam;
	pCC->m_CBN.Run();
	return 0;
}

void* C_C::ThreadMainSick(void* lpParam)
{
	C_C * pCC=(C_C *)lpParam;
	string ip = "192.168.1.2";
	int port = 2112;
	if(pCC->m_DuoSick.setSick_A(ip, port))
	{
		printf("ThreadMainSick!!!!\n");
		pCC->m_DuoSick.runSick_A();
	}
	return 0;
}

void* C_C::ThreadMinorSick(void* lpParam)
{
	C_C * pCC=(C_C *)lpParam;

	//readSickForOD(lpParam);
	/*
	string ip = "192.168.1.3";
	int port = 2112;

	if(pCC->m_DuoSick.setSick_B(ip, port))
	{
		pCC->m_DuoSick.runSick_B();
	}
	 */
	return 0;
}


void* C_C::ThreadIOA(void* lpParam)
{
	pthread_detach(pthread_self());
	C_C * pCC=(C_C *)lpParam;

	//	while(1)
	{
		//printf("ThreadIOA!!!!!!!!\n");
		//pCC->m_CIOA.CanRunAreaCheck();
		//usleep(20000);
	}
	return 0;
}


void *C_C::ThreadUploadPointCloud(void* lpParam)
{
	pthread_detach(pthread_self());
	C_C * pCC=(C_C *)lpParam;
	int nRtn;
	int nCount=0;
	int nSleepCount=0;
	while(1)
	{
		sem_wait(&g_SndPCSgl);
		if(m_vctPC.size()>0)
		{
			static int cnt = 0;
			//	int nDatalen=m_vctPC.size()*4+12,i;

			int nDatalen=ONE_LINE_LEN,i;
			char *pcData=new char[m_vctPC.size()*4+12];
			char *pcTmp=pcData+12;

			memcpy(pcData,&m_fCurPos[0],4);
			memcpy(pcData+4,&m_fCurPos[1],4);
			memcpy(pcData+8,&m_fCurPos[2],4);

			float fTmp;
			for(i=0;i<m_vctPC.size();i++)
			{
				fTmp=m_vctPC[i];
				memcpy(pcTmp,&fTmp,4);
				pcTmp+=4;
			}

			nRtn=NetPortal::UploadMapBuilder(pcData,nDatalen);
			if(nRtn==-1)
			{
				//printf("Store Pc data!!!!\n");
				if(nSleepCount>40)
				{
					m_pCC_CThis->m_CAmbientGridMap.Wrtie2RawDataBuff(pcData,nDatalen);
					nSleepCount=0;
				}
				nSleepCount++;

			}
			else
			{
				nSleepCount=0;
			}

			/*	printf("Send UploadMapBuilder\n");
			if(pCC->m_CNetPortal.m_cCncStatus[0][1]==RTN_RS_CNC_NOR
							&&pCC->m_CNetPortal.m_cCncStatus[1][1]==RTN_RS_CNC_NOR
							&&pCC->m_CNetPortal.m_cCncStatus[0][1]==RTN_RS_CNC_NOR)
			{
				nRtn=NetPortal::UploadMapBuilder(pcData,nDatalen);
				printf("UploadMapBuilder nRtn:%d \n",nRtn);
				if(nRtn==-1)
				{
					m_pCC_CThis->m_CAmbientGridMap.Wrtie2RawDataBuff(pcData,nDatalen);
				}
			}
			else
			{
				m_pCC_CThis->m_CAmbientGridMap.Wrtie2RawDataBuff(pcData,nDatalen);
			}*/




			delete [] pcData;
		}
	}



}

void *C_C::ThreadNetMonitor(void* lpParam)
{
	pthread_detach(pthread_self());
	C_C * pCC=(C_C *)lpParam;
	int nCount=100;
	bool bJump=false;

	while(1)
	{
		if(pCC->m_CNetPortal.m_cCncStatus[0][1]==RTN_OK
				&&pCC->m_CNetPortal.m_cCncStatus[1][1]==RTN_OK
				&&pCC->m_CNetPortal.m_cCncStatus[2][1]==RTN_OK)
		{
			//printf("good net!!!\n");
			usleep(10000);
			bJump=false;
		}
		else
		{
			while(!bJump)
			{
				if(pCC->m_CNetPortal.m_cCncStatus[0][1]==RTN_OK
						&&pCC->m_CNetPortal.m_cCncStatus[1][1]==RTN_OK
						&&pCC->m_CNetPortal.m_cCncStatus[2][1]==RTN_OK)
				{
					if(nCount>2)
					{
						printf("Start 2 send old pc!!!\n");
						pCC->m_CAmbientGridMap.SendStoreData2RS();
						bJump=true;
					}
					else
					{
						printf("net recover:%d !!!!!\n",nCount);
						nCount++;
						sleep(1);
					}
				}
				else
				{
					printf("Net error!!!!\n");
					nCount=0;
					sleep(1);
				}
			}
		}
	}
	return 0;
}


void* C_C::ThreadSubCtrlCom(void* lpParam)
{
	pthread_detach(pthread_self());
	C_C * pCC=(C_C *)lpParam;
	pCC->m_CSubCtrlCom.SubCtrlComRun();
	return 0;
}

void* C_C::ThreadLogFile(void* lpParam)
{
	pthread_detach(pthread_self());
	C_C * pCC=(C_C *)lpParam;
	pCC->m_CLogFile.LogFileRun();
	return 0;
}


void *C_C::ThreadGPS(void* lpParam)
{
	pthread_detach(pthread_self());
	double dVal1,dVal2;
	C_C * pCC=(C_C *)lpParam;
	while(1)
	{
		pCC->m_CGPS.listenUART();
		usleep(100000);
		pCC->m_CGPS.getGPSValue(&dVal1,&dVal2);
		printf("GPS Value:  %f   %f",dVal1,dVal2);
	}
}

int C_C::GetSerialPortNum(char* pcKey,char *pcPortNum)
{

	int nRtn=-1;
	remove("usb.list");
	string cmd("dmesg > usb.list");
	//cout<<"key: "<<string(pcKey)<<endl;

	//string cmdf = cmd + string(" grep ") + string(pcKey) + string(" | tail -f > ") + string("usb.txt");
	//cout<<"cmd: "<<cmdf<<endl;
	system(cmd.c_str());
	int i;
	fstream fin("usb.list");
	char line[4096];
	char* pv = pcPortNum;
	char * pV = 0;
	char *pcTmp1,*pcTmp2;
	int nLineLen=0;
	int nKeyLen=strlen(pcKey);
	while(fin.getline(line,4096)){
		pcTmp1=line;
		nLineLen=strlen(pcTmp1);

		for(i=0;i<nLineLen-nKeyLen;i++)
		{
			if(memcmp(pcKey,pcTmp1+i,nKeyLen)==0)
			{
				printf("i found it!!!!\n");
				printf("%s  \n",line);
				memcpy(pcPortNum,pcTmp1+i+nKeyLen,strlen(pcTmp1+i+nKeyLen));
				printf("%s  \n",pcTmp1+i);
				nRtn=0;

			}
		}
		memset(line,0,4096);

	}
	return nRtn;
}

int C_C::ParseMsg(char *pcData,float *pfValue,int &nID)
{
	//	printf("0000:%d\n",pcData);
	float fSign;

	char cData[10];
	int nCount;
	//if(pcData[1]!='A'||pcData[1]=='B')return -1;

	if(pcData[1]=='A')nID=0;
	else nID=1;

	if(pcData[2]=='-')fSign=-1.0;
	else fSign=1.0;
	nCount=0;
	memset(cData,0,10);
	//printf("1111111\n");
	while(nCount<7)
	{
		if(pcData[3+nCount]=='0')
			nCount++;
		else
		{
			//printf("%c  \n",*(pcData+3+nCount));
			memcpy(cData,pcData+3+nCount,7-nCount);
			break;
		}
	}
	pfValue[0]=atof(cData)*fSign/10;

	//	printf("Parse 1:%f ,%s \n",pfValue[0],cData);

	if(pcData[10]=='-')fSign=-1.0;
	else fSign=1.0;
	nCount=0;
	memset(cData,0,10);
	while(nCount<7)
	{
		if(pcData[11+nCount]=='0')
			nCount++;
		else
		{
			memcpy(cData,pcData+11+nCount,7-nCount);
			break;
		}
	}
	pfValue[1]=atof(cData)*fSign/10;
	//	printf("Parse 2:%f ,%s \n",pfValue[1],cData);

	if(pcData[18]=='-')fSign=-1.0;
	else fSign=1.0;
	nCount=0;
	memset(cData,0,10);
	while(nCount<5)
	{
		if(pcData[19+nCount]=='0')
			nCount++;
		else
		{
			memcpy(cData,pcData+19+nCount,5-nCount);
			break;
		}
	}
	pfValue[2]=atof(cData)*fSign/10;
	//	printf("Parse 3:%f ,%s \n",pfValue[2],cData);

	if(pcData[24]=='-')fSign=-1.0;
	else fSign=1.0;
	nCount=0;
	memset(cData,0,10);
	while(nCount<5)
	{
		if(pcData[25+nCount]=='0')
		{
			//printf("$$%c  \n",pcData[25+nCount]);
			nCount++;
		}
		else
		{

			memcpy(cData,pcData+25+nCount,5-nCount);
			break;
		}
	}
	pfValue[3]=atof(cData)*fSign;

	//	printf("Parse 4:%f,%s  \n",pfValue[3],cData);

	return 0;
}
void *C_C::ThreadAskRobotStatus(void* lpParam)
{
	C_C * pCC=(C_C *)lpParam;
	while(1)
	{
		//printf("GetRobotOdo\n");
		pCC->m_CSubCtrlCom.GetRobotOdo();

		//	pCC->m_CSubCtrlCom.GetRobotStatus();
		usleep(100000);
	}

}
void* C_C::ThreadReadRobotStatus(void* lpParam)
{
	C_C * pCC=(C_C *)lpParam;
	char cData[32],*pcTmp,cTmp[12];
	float fValue[4],th,x,y;
	int i,nID;
	sleep(3);
	printf("ThreadReadRobotStatus!!!!!!!!!\n");
	printf("m_fOffset:%f   ,%f   ,%f  \n",m_fOffset[0],m_fOffset[1],m_fOffset[2]);
	while(1)
	{
		if(pCC->m_CSubCtrlCom.ReadData(cData,32)!=-1)
		{

			if(pCC->ParseMsg(cData,fValue,nID)!=-1)
			{
				if(nID==0)
				{
					m_fOdometryData[0]=fValue[0];
					m_fOdometryData[1]=fValue[1];
					m_fOdometryData[2]=fValue[2];
					m_fOdometryData[3]=fValue[3];

					/*	if(m_pCSLAM->m_work_model==3)
					{
						th=(m_fOdometryData[2]-90,0)/180.0*3.1415926+m_fOffset[2];
						x=m_fOdometryData[0]+m_fOffset[0];
						y=m_fOdometryData[1]+m_fOffset[1];

						IOA::DataFusionResult(m_fOdometryData[0],
								m_fOdometryData[1],th);

						memcpy(cTmp,&x,4);
						memcpy(cTmp+4,&y,4);
						memcpy(cTmp+8,&th,4);
						memcpy(m_fCurPos,cTmp,12);
						NetPortal::UploadA(cTmp,12);
					}*/
					/*	printf("Odo:x :%f  ,y:%f  ,th:%f  ,:time:%f \n",
							m_fOdometryData[0],m_fOdometryData[1],
							m_fOdometryData[2],m_fOdometryData[3]);*/
				}
				else
				{
					m_fVoltBatteryCtrl=fValue[0];
					m_fVoltBatteryPower=fValue[1];
					m_fGYRO_Global=fValue[2];
					m_fSICK_IO_Read=fValue[3];

					/*printf("Status:m_fVoltBatteryCtrl :%f ,m_fVoltBatteryPower:%f  ,m_fGYRO_Global:%f  ,:m_fSICK_IO_Read:%f\n",
							m_fVoltBatteryCtrl,m_fVoltBatteryPower,
							m_fGYRO_Global,m_fSICK_IO_Read);*/
				}
			}
		}
	}
}

int C_C::ReadOdometry(float *pcData)
{
	pcData[0]=m_fOdometryData[0];
	pcData[1]=m_fOdometryData[1];
	pcData[2]=m_fOdometryData[2];
	pcData[3]=m_fOdometryData[3];
	return 0;
}


void *C_C::ThreadSystemMonitor(void* lpParam)
{
	C_C * pCC=(C_C *)lpParam;
	m_fVoltBatteryCtrl=100;
	m_fVoltBatteryPower=100;
	while(1)
	{
		pCC->m_CSubCtrlCom.GetRobotStatus();

		if(m_fVoltBatteryCtrl<CTRL_BATTERY_SAFE_VOLT)
		{
			m_nSysErrList=SYS_ERR_CTRL_BATTERY_LOW;
		}

		if(m_fVoltBatteryPower<POWER_BATTERY_SAFE_VOLT)
		{
			m_nSysErrList=SYS_ERR_CTRL_BATTERY_LOW;
		}
		sleep(1);
	}
}

int C_C::ErrList(int nErrCode)
{
	printf("ErrList in...\n");
	//pthread_mutex_lock(&g_ErrListLock);
	m_nSysErrList=nErrCode;

	printf("m_nSysErrList:%d  \n");
	//pthread_mutex_unlock(&g_ErrListLock);
	return 0;
}

int C_C::cbSLAM(float x,float y,float th){

	static int cnt = 0;
	char cTmp[16];

	int nDataType=FUSION_DATA;
	//int nDataType=SLAM_DATA;


	x*=1000;
	y*=1000;
	x+=m_fOffset[0];
	y+=m_fOffset[1];
	th+=m_fOffset[2];

	if(m_nTrustData==TRUST_DATA_FUSION)
	{
		IOA::DataFusionResult(x,y,th);
	}

	memcpy(cTmp,&nDataType,4);
	memcpy(cTmp+4,&x,4);
	memcpy(cTmp+8,&y,4);
	memcpy(cTmp+12,&th,4);

	memcpy(m_fCurPos,cTmp+4,12);
	if(cnt>5)
	{
		NetPortal::UploadA(cTmp,16);
		cnt=0;
	}
	else cnt++;
	//printf("cbOnlyDFRslt:x:%f,  y:%f  ,th:%f \n",x,y,th);

}

int C_C::cbOnlySLAMRslt(float x,float y,float th)
{
	static int cnt = 0;
	char cTmp[16];

	int nDataType=SLAM_DATA;



	x*=1000;
	y*=1000;

	x+=m_fOffset[0];
	y+=m_fOffset[1];
	th+=m_fOffset[2];
	if(m_nTrustData==TRUST_SLAM_RSLT)
	{
		IOA::DataFusionResult(x,y,th);
	}

	memcpy(cTmp,&nDataType,4);
	memcpy(cTmp+4,&x,4);
	memcpy(cTmp+8,&y,4);
	memcpy(cTmp+12,&th,4);

	if(cnt>6)
	{

		NetPortal::UploadA(cTmp,16);
		cnt=0;
	}
	else cnt++;
	//printf("cbOnlySLAmRslt:x:%f,  y:%f  ,th:%f \n",x,y,th);
	return 0;
}
int C_C::cbOnlyOdoRslt(float x,float y,float th)
{
	static int cnt = 0;
	char cTmp[16];

	int nDataType=ODO_DATA;


	x*=1000;
	y*=1000;

	x+=m_fOffset[0];
	y+=m_fOffset[1];
	th+=m_fOffset[2];
	if(m_nTrustData==TRUST_ODO_RSLT)
	{
		IOA::DataFusionResult(x,y,th);
	}

	memcpy(cTmp,&nDataType,4);
	memcpy(cTmp+4,&x,4);
	memcpy(cTmp+8,&y,4);
	memcpy(cTmp+12,&th,4);

	if(cnt>7)
	{

		NetPortal::UploadA(cTmp,16);
		cnt=0;
	}
	else cnt++;
	//printf("cbOnlyODORslt:x:%f,  y:%f  ,th:%f \n",x,y,th);
	return 0;
}
int C_C::cbOnlyBNRslt(float x,float y,float th)
{
	static int cnt = 0;
	char cTmp[12];
	int nDataType=BN_DATA;

	x*=1000;
	y*=1000;
	//printf("cbOnlyBNRslt:x:%f,  y:%f  ,th:%f \n",x,y,th);
	if(m_nTrustData==TRUST_BN_RSLT)
	{
		IOA::DataFusionResult(x,y,th);
	}


	memcpy(cTmp,&nDataType,4);
	memcpy(cTmp+4,&x,4);
	memcpy(cTmp+8,&y,4);
	memcpy(cTmp+12,&th,4);
	if(cnt>5)
	{

		NetPortal::UploadA(cTmp,16);
		cnt++;
	}
	else cnt=0;

	NetPortal::UploadA(cTmp,12);
	return 0;
}


int C_C::cbDataFusionAndPC(float* b, int n, double px, double py, double pth)
{
	pthread_mutex_lock(&g_FusionDataAndPCLock);
	memcpy(m_fScanData,b,541*4);
	m_fCurPos[0]=px;
	m_fCurPos[1]=py;
	m_fCurPos[2]=pth;

	//printf("CurPos:%f  ,%f  ,%f  \n",px,py,pth);
	pthread_mutex_unlock(&g_FusionDataAndPCLock);
	return 0;
}
void *C_C::ThreadUploadDF_PC(void* lpParam)
{
	float fScanData[541];
	float fX,fY,fTheta;
	char cTmp[16];
	static int nDFCount=0;

	int nDatalen=ONE_LINE_LEN,i,nRtn=0,nSleepCount=0;
	char *pcData=new char[541*4+12];
	char *pcTmp=pcData+12;
	while(1)
	{
		pthread_mutex_lock(&g_FusionDataAndPCLock);
		memcpy(fScanData,m_fScanData,541*4);
		fX=m_fCurPos[0];
		fY=m_fCurPos[1];
		fTheta=m_fCurPos[2];
		pthread_mutex_unlock(&g_FusionDataAndPCLock);

		int nDataType=FUSION_DATA;
		fX*=1000;
		fY*=1000;

		fX+=m_fOffset[0];
		fY+=m_fOffset[1];
		fTheta+=m_fOffset[2];
		if(m_nTrustData==TRUST_SLAM_RSLT)
		{
			IOA::DataFusionResult(fX,fY,fTheta);
		}
		memcpy(cTmp,&nDataType,4);
		memcpy(cTmp+4,&fX,4);
		memcpy(cTmp+8,&fY,4);
		memcpy(cTmp+12,&fTheta,4);

		memcpy(pcData,&fX,4);
		memcpy(pcData+4,&fY,4);
		memcpy(pcData+8,&fTheta,4);

		memcpy(pcTmp,fScanData,541*4);

		if(nDFCount>5)
		{
			NetPortal::UploadA(cTmp,16);
			nDFCount=0;
			nRtn=NetPortal::UploadMapBuilder(pcData,nDatalen);
			if(nRtn==-1)
			{
				//printf("Store Pc data!!!!\n");
				if(nSleepCount>8)
				{
					m_pCC_CThis->m_CAmbientGridMap.Wrtie2RawDataBuff(pcData,nDatalen);
					nSleepCount=0;
				}
				nSleepCount++;

			}
			else
			{
				nSleepCount=0;
			}
		}
		else nDFCount++;

	//	printf("ThreadUploadDF_PC:%d \n",nRtn);


		usleep(20000);
	}
}


void *C_C::ThreadGetScanData(void* lpParam)
{
	float fScanData[541];
	TTimeStamp ulTime;
	vector<float> vctScanData;
	int i;
	while(1)
	{

		vctScanData.clear();
		if(m_pCC_CThis->m_DuoSick.getScan_A_OD(fScanData,ulTime)==1)
		{
			//printf("ThreadGetScanData ");
			for(i=0;i<541;i++)
			{
				vctScanData.push_back(fScanData[i]/100);
			}
			IOA::GetLaserAData(vctScanData);
		}
		usleep(20000);
	}
	return 0;
}

/*int C_C::readSickForOD(void* lpParam)
{
	C_C * pCC=(C_C *)lpParam;


	float sick_data[541];
	uint64_t t;
	while(1)
	{
		if(pCC->m_DuoSick.getScan_A_OD(sick_data,t))
		{
			cout<<"OD: "<<t<<endl;
		}
		usleep(20000);
	}

	return 0;
}*/
