#include "slam_v1.h"
#include "point.h"
#include "clientFrontend.h"
#include "clientBackend.h"
#include "clientFusion.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <fstream>
#include "point.h"
#include "localv1.h"

//using namespace zhpsm;

namespace{
const float bad_range = -100000;
	bool isBadRange(float r){
		return (r==bad_range);
	}

	void filterMapinfo(vector<float>& fx,vector<float>& fy){
		vector<float>::iterator it;
		it = remove_if(fx.begin(),fx.end(),(bool(*)(float))isBadRange);
		if(it!=fx.end()){
			fx.erase(it,fx.end());
			it = remove_if(fy.begin(),fy.end(),(bool(*)(float))isBadRange);
			fy.erase(it,fy.end());
		}
	}
}

CSlamV1::CSlamV1():
m_pThreadFrontend(new CClientFrontend),
m_pThreadBackend(new CClientBackend),
m_stopped(false),
m_received(false),
m_received_fusion(false),
m_received_scan(false),
m_pCBSet(0)
{}
CSlamV1::~CSlamV1(){
}

void CSlamV1::default_init()
{
	
	// m_file_name = "/media/ShareRegion/Datasets/LMS151_20130108/20130108_t2.txt";
	// m_file_name = "/media/ShareRegion/Datasets/LMS151_20130108/t1.txt";

	// m_file_name = "/media/ShareRegion/Datasets/LMS151_20130108/t2.txt";
	m_file_name = "/home/administrator/Work/lms151/20130108_t2.txt";

	//m_file_name = "/mnt/hgfs/SharedFold/dataset/lenovo/LMS151/20130108_t2.txt";
	m_work_model = 1; // 1 read carmon; 2 sick online; 3 fusion;
	m_sick2_ip = "192.168.1.3";
	m_sick1_ip = "192.168.1.2";
	m_sick1_port = 2112;
	m_sick2_port = 2112;


}
void CSlamV1::initFromParams( SLAMParams* p){
	cout<<"not yet realised!"<<endl;
}

void CSlamV1::init(SLAM_CallBack* pCBSet, SLAMParams* pParams){
	if(pCBSet != NULL)
		m_pCBSet = pCBSet;
	if(pParams!= NULL){
		initFromParams(pParams);
	}else{
		default_init();
	}

	bnPose = new float[3];
	for(int i=0;i<3;i++)
		bnPose[i]=0.;
	odoPose = new float[4];
	for(int i=0;i<4;i++)
			odoPose[i]=0.;
	odo_first = true;
	bn_first = true;
	scanMain = new float[541];
	scanMinor = new float[541];

	t_scanMain_cur = 9999;
	t_scanMinor_cur = 9999;
	t_scanMain_last = 9999;
	t_scanMinor_last = 9999;

	// setSystem();
}


// these settings are important
void CSlamV1::setSystem()
{
	vector<string > ips;
	vector<unsigned int> ports;
	ips.push_back(m_sick1_ip); ports.push_back(m_sick1_port);
	ips.push_back(m_sick2_ip); ports.push_back(m_sick2_port);

	// Frontend process
	m_pThreadFrontend = new CClientFrontend;
	m_pThreadFrontend->setFile(m_file_name);
	m_pThreadFrontend->setSICKIP(ips, ports);
	m_pThreadFrontend->setModel(m_work_model);
	m_pThreadFrontend->moveToThread(&m_threadFrontend);
	connect(&m_threadFrontend,SIGNAL(started()),m_pThreadFrontend,SLOT(runFrontEnd()));
	
	// Backend process
	m_pThreadBackend = new CClientBackend;
	m_pThreadBackend->moveToThread(&m_threadBackend);
	m_pThreadBackend->setModel(m_work_model);
	connect(&m_threadBackend,SIGNAL(started()),m_pThreadBackend,SLOT(startBackend()));

	// quit up->down
	connect(this,SIGNAL(stopFrontend()),m_pThreadFrontend,SLOT(stop()),Qt::DirectConnection);
	connect(this, SIGNAL(stopBackend()),m_pThreadBackend,SLOT(stopBackend()),Qt::DirectConnection);
	connect(m_pThreadFrontend,SIGNAL(finished()),&m_threadFrontend,SLOT(quit()),Qt::DirectConnection);
	connect(m_pThreadBackend,SIGNAL(finished()),&m_threadBackend,SLOT(quit()),Qt::DirectConnection);
	connect(m_pThreadFrontend,SIGNAL(sickError()),this,SLOT(quit()),Qt::DirectConnection);

	//For localization

	string imgfile = "/home/jetfire/CoProjects/trunk/data/B2Map.jpg";

	int start_x = 1393;
	int start_y = 517;
	m_pLocalization = new LocalV1;
	m_pLocalization->init(imgfile.c_str(),start_x,start_y);
	m_pLocalization->moveToThread(&m_threadLocalization);
	connect(&m_threadLocalization,SIGNAL(started()),m_pLocalization,SLOT(runLocalization()));
	connect(m_pLocalization,SIGNAL(finished()),&m_threadLocalization,SLOT(quit()),Qt::DirectConnection);
	connect(this,SIGNAL(stopLocalization()),m_pLocalization,SLOT(stop()),Qt::DirectConnection);


	if(m_work_model >= 3)
	{
		//fusion
		m_pThreadFrontend->setFusion();
		connect(this,SIGNAL(sendBN(float,float,float)),m_pThreadFrontend->m_pFusion,SLOT(recvBN(float,float,float)),Qt::DirectConnection);
		connect(this,SIGNAL(sendODO(float,float,float,float)),m_pThreadFrontend->m_pFusion,SLOT(recvODO(float,float,float,float)),Qt::DirectConnection);

		// frontend -> backend
		connect(m_pThreadFrontend->m_pFusion,SIGNAL(sendScanFrameCov(void*,int,double*)),m_pThreadBackend,SLOT(receScanFrameCov(void*,int,double*)),Qt::DirectConnection);
		// backend -> frontend
		connect(m_pThreadBackend,SIGNAL(sendUpdatePose(int,void*,double*)),m_pThreadFrontend->m_pFusion,SLOT(recvRelGlobalOptimized(int,void*,double*)),Qt::DirectConnection);
		connect(m_pThreadFrontend->m_pFusion,SIGNAL(send2CBScan(float*, int, double,double,double)),this,SLOT(receCurrentScan(float*, int, double,double,double)),Qt::DirectConnection);
		connect(m_pThreadFrontend->m_pFusion,SIGNAL(finished()),&m_threadFrontend,SLOT(quit()),Qt::DirectConnection);
		//detect sick connection errors
		connect((const QObject*)(m_pThreadFrontend->m_pFusion->mainSICK),SIGNAL(sickCNKError()),this,SLOT(receMainSickCNKError()),Qt::DirectConnection);
		connect((const QObject*)(m_pThreadFrontend->m_pFusion->minorSICK),SIGNAL(sickCNKError()),this,SLOT(receMinorSickCNKError()),Qt::DirectConnection);
		//send scan only before being processed
		connect((const QObject*)(m_pThreadFrontend->m_pFusion->mainSICK),SIGNAL(send2CBScanOnly(float*, int)),this,SLOT(receCurrentScanOnly(float*, int)),Qt::DirectConnection);

		//this scan -> frontend sick
		connect(this,SIGNAL(sendScanMain(float*, uint64_t)),(const QObject*)(m_pThreadFrontend->m_pFusion->mainSICK),SLOT(receScan(float*, uint64_t)),Qt::DirectConnection);
		connect(this,SIGNAL(sendScanMinor(float*, uint64_t)),(const QObject*)(m_pThreadFrontend->m_pFusion->minorSICK),SLOT(receScan(float*, uint64_t)),Qt::DirectConnection);

		//send Main sick result
		connect((const QObject*)(m_pThreadFrontend->m_pFusion->mainSICK),SIGNAL(sendFrameInfo(double,double,double,float*,int)),this,SLOT(receMainSickSLAM(double,double,double,float*,int)),Qt::DirectConnection);

		//send odo
		connect(m_pThreadFrontend->m_pFusion,SIGNAL(sendODO(float, float, float, float)),this,SLOT(receODO(float, float, float, float)),Qt::DirectConnection);
		//send BN
		connect(m_pThreadFrontend->m_pFusion,SIGNAL(sendBN(float, float, float)),this,SLOT(receBN(float, float, float)),Qt::DirectConnection);

		//send LD
		connect(m_pThreadFrontend->m_pFusion,SIGNAL(sendLD(float, float, float)),this,SLOT(receLD(float, float, float)),Qt::DirectConnection);

		// send to localization
		connect(m_pThreadFrontend->m_pFusion,SIGNAL(sendFirstFrame(void*)),m_pLocalization,SLOT(receFirstFrame(void*)),Qt::DirectConnection);
		connect(m_pThreadFrontend->m_pFusion,SIGNAL(sendScanFrame(void*,int)),m_pLocalization,SLOT(recePoseScan(void*,int)),Qt::DirectConnection);
		connect(m_pLocalization,SIGNAL(sendUpdatePose(int, void*)),m_pThreadFrontend->m_pFusion,SLOT(recvLocalizedPose(int, void*)),Qt::DirectConnection);

	}
	else
	{
		// frontend -> backend
		// connect(m_pThreadFrontend,SIGNAL(sendScanFrame(void*,int)),m_pThreadBackend,SLOT(receScanFrame(void*,int)),Qt::DirectConnection);
		connect(m_pThreadFrontend,SIGNAL(sendScanFrameCov(void*,int,double*)),m_pThreadBackend,SLOT(receScanFrameCov(void*,int,double*)),Qt::DirectConnection);
		// backend -> frontend
		connect(m_pThreadBackend,SIGNAL(sendUpdatePose(int,void*,double*)),m_pThreadFrontend,SLOT(receUpdatePose(int,void*,double*)),Qt::DirectConnection);

		// connect(m_pThreadFrontend,SIGNAL(sendFrameInfo(double,double,double,float*,int)),this,SLOT(receLaserFrame(double,double,double,float*,int)));
		// frontend -> CALLBACK
		connect(m_pThreadFrontend,SIGNAL(send2CBScan(float*, int, double,double,double)),this,SLOT(receCurrentScan(float*, int ,double,double,double)),Qt::DirectConnection);
		// connect(m_pThreadFrontend, SIGNAL(send2CBPose(double,double,double)),this,SLOT(receCurrentPose(double,double,double)),Qt::DirectConnection);
		connect(m_pThreadFrontend,SIGNAL(sickCNKError()),this,SLOT(receMainSickCNKError()),Qt::DirectConnection);
		connect(m_pThreadFrontend,SIGNAL(send2CBScanOnly(float*, int)),this,SLOT(receCurrentScanOnly(float*, int)),Qt::DirectConnection);
		//this scan -> frontend
		connect(this,SIGNAL(sendScanMain(float*, uint64_t)),m_pThreadFrontend,SLOT(receScan(float*, uint64_t)),Qt::DirectConnection);

		// send to localization
		connect(m_pThreadFrontend,SIGNAL(sendFirstFrame(void*)),m_pLocalization,SLOT(receFirstFrame(void*)),Qt::DirectConnection);
		connect(m_pThreadFrontend,SIGNAL(sendScanFrame(void*,int)),m_pLocalization,SLOT(recePoseScan(void*,int)),Qt::DirectConnection);
		connect(m_pLocalization,SIGNAL(sendUpdatePose(int, void*)),m_pThreadFrontend,SLOT(receUpdatePose2(int, void*)),Qt::DirectConnection);

		//this<-odo/bn->frontend
		connect(this,SIGNAL(sendBN(float,float,float)),m_pThreadFrontend,SLOT(recvBn(float,float,float)),Qt::DirectConnection);
		connect(this,SIGNAL(sendODO(float,float,float,float)),m_pThreadFrontend,SLOT(recvOdo(float,float,float,float)),Qt::DirectConnection);

		//send odo
		connect(m_pThreadFrontend,SIGNAL(sendOdo(float, float, float, float)),this,SLOT(receODO(float, float, float, float)),Qt::DirectConnection);
		//send BN
		connect(m_pThreadFrontend,SIGNAL(sendBn(float, float, float)),this,SLOT(receBN(float, float, float)),Qt::DirectConnection);



	}
}

void CSlamV1::startEventLoop(int& argc, char** argv){
	qt_app = new QCoreApplication(argc,argv);
	qt_app->exec();
}

void CSlamV1::run(){

	TTimeStamp ts = getCurrentTime();

	ofstream outf("slamv1_scan.log");
	ofstream outf2("slamv1_odo.log");
	// startThreads
	m_threadFrontend.start();
	m_threadBackend.start();
	m_threadLocalization.start();
	while(!m_stopped && m_pCBSet!=NULL){

		//cout<<"LOOP>>>>"<<endl;

		//send odo
		if(m_pCBSet->cbOdometry!=NULL)
		{
			m_pCBSet->cbOdometry(odoPose);
			//printf("sendODO: %f, %f, %f, %f\n", odoPose[0], odoPose[1], odoPose[2],odoPose[3]);

			if(odoPose[0] || odoPose[1] || odoPose[2] || odoPose[3])
			{
				odoTime_cur = odoPose[3];
				if(odo_first)
				{
					outf2<<getCurrentTime()<<", "<<odoPose[0]<<", "<<odoPose[1]<<", "<<odoPose[2]<<", "<<odoPose[3]<<endl;
					odoTime_last = odoTime_cur;

					//printf("sendODO: %f, %f, %f, %f\n", odoPose[0], odoPose[1], odoPose[2],odoPose[3]);
					this->sendODO(odoPose[0],odoPose[1],odoPose[2],odoPose[3]);//mm, degree
					odo_first = false;
				}
				else{

					if(odoTime_cur == odoTime_last)
					{ }
					else
					{


						outf2<<getCurrentTime()<<", "<<odoPose[0]<<", "<<odoPose[1]<<", "<<odoPose[2]<<", "<<odoPose[3]<<endl;
						//printf("sendODO: %f, %f, %f, %f\n", odoPose[0], odoPose[1], odoPose[2],odoPose[3]);
						this->sendODO(odoPose[0],odoPose[1],odoPose[2],odoPose[3]);//mm, degree
						odoTime_last = odoTime_cur;
					}
				}
			}
		}

		//send bn
		if(m_pCBSet->cbBNLocation != NULL)
		{
			m_pCBSet->cbBNLocation(bnPose);

			//printf("sendBN: %f, %f, %f \n", bnPose[0], bnPose[1], bnPose[2]);
			if(bnPose[0] || bnPose[1] || bnPose[2])
			{
				//printf("sendBN: %f, %f, %f \n", bnPose[0], bnPose[1], bnPose[2]);
				this->sendBN(bnPose[0], bnPose[1], bnPose[2]); //cm, rad
			}

			if(bn_first)
			bn_first = false;
		}

		//send laser A (Main)
		if(m_pCBSet->cbMainSICKForSLAM != NULL)
		{
			if(m_pCBSet->cbMainSICKForSLAM(scanMain, t_scanMain_cur))
			{
				//cout<<"MainSICK TIME: "<<timeDifference(t_scanMain_last, t_scanMain_cur)<<" "<<t_scanMain_last<<" "<<t_scanMain_cur<<endl;

				if(t_scanMain_cur == t_scanMain_last)
				{/*old scan*/}
				else
				{

					//cout<<"MainSICK TIME: "<<t_scanMain_last<<" "<<t_scanMain_cur<<endl;
					//outf<<timeDifference(t_scanMain_last, t_scanMain_cur)<<", 541, 0, ";
					outf<<t_scanMain_cur<<", 541, 0, ";
					for(int i=0;i<541;i++)
					{
						outf<<scanMain[i]<<", ";
					}
					outf<<endl;

				    this->sendScanMain(scanMain, t_scanMain_cur); //cm, rad
				    t_scanMain_last = t_scanMain_cur;
				}
			}
		}

		//wait for few ms for processing
		usleep(10000);


	}
	cout<<"threadSLAM quit!"<<endl;
}

void CSlamV1::receCurrentPose(double x, double y, double th)
{
	QMutexLocker lock(&m_mutex);
	m_px = x; m_py = y; m_pth = th;
	//m_received_fusion = true;
	//m_wait_cond.wakeOne();
}
//SS Comment out
/*
void CSlamV1::receCurrentScan(float* b, int n, double px, double py, double pth)
{

	//FUSION RESULT

	QMutexLocker lock(&m_mutex);
	//printf("receCurrentScan receive: %f, %f, %f \n", px, py, pth);
	m_px = px; m_py = py; m_pth = pth;
	if(n>0)
	{
		if(m_b.size()<n) {
			m_b.resize(n);
		}
		copy(b,b+n,m_b.begin());
	}
	// copy(y,y+n,m_sy.begin());
	m_received_fusion = true;
	//m_wait_cond.wakeOne();
}*/


void CSlamV1::receCurrentScan(float* b, int n, double px, double py, double pth)
{

	//FUSION RESULT

	QMutexLocker lock(&m_mutex);
	//printf("receCurrentScan receive: %f, %f, %f \n", px, py, pth);
	m_px = px; m_py = py; m_pth = pth;
	if(n>0)
	{
		if(m_b.size()<n) {
			m_b.resize(n);
		}
		copy(b,b+n,m_b.begin());
	}
	m_pCBSet->cbDataFusionAndPC(b,n,px,py,pth);
	// copy(y,y+n,m_sy.begin());
	//m_received_fusion = true;
	//m_wait_cond.wakeOne();
}

void CSlamV1::receCurrentScanOnly(float* b, int n)
{
	QMutexLocker lock(&m_mutex);
	if(n>0)
	{
		if(m_b.size()<n) {
			m_b.resize(n);
		}
		copy(b,b+n,m_b.begin());
	}
	//m_received_scan = true;
	//m_wait_cond.wakeOne();
	//printf("receive SICK SCAN! \n");

}


void CSlamV1::receMainSickSLAM(double x,double y,double theta,float* scan,int scan_num)
{
	// MAIN SICK ONLY
	//cout<<"receMainSickSLAM"<<endl;
	m_pCBSet->cbOnlySLAMResult(x,y,theta);
}

void CSlamV1::receODO(float x, float y, float theta, float t)
{
	//ODO ONLY
	cout<<"CSlamV1::receODO: "<<x<<" "<<y<<" "<<theta<<endl;
	m_pCBSet->cbOnlyOdoResult(x,y,theta);

}

void CSlamV1::receBN(float x, float y, float theta)
{
	//BN only
	m_pCBSet->cbOnlyBNResult(x,y,theta);
}


void CSlamV1::receLD(float x, float y, float theta)
{
	//LD ONLY

	m_pCBSet->cbLocalization(x,y,theta);

}


void CSlamV1::receMainSickCNKError()
{

	printf("CSlamV1::receMainSickCNKError(): Failed to connect SICK! \n");
	printf("m_pCBSet->cbErrList :%d  \n",m_pCBSet->cbErrList);
	m_pCBSet->cbErrList(SYS_LOST_CNC_SICK_A);

}

void CSlamV1::receMinorSickCNKError()
{

	printf("CSlamV1::receMinorSickCNKError(): Failed to connect SICK! \n");
	m_pCBSet->cbErrList(SYS_LOST_CNC_SICK_B);

}

void CSlamV1::stop(){
	if(!m_threadFrontend.isRunning())
	{
		stopFrontend();
		stopBackend();
		// stopAllThread();
	}
	while(m_threadBackend.isRunning()){
		QThread::yieldCurrentThread();
	}
	cout<<"all subThreads are stopped!"<<endl;
	m_stopped = true;
}


