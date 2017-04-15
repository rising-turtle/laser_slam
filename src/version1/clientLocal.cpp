#include "clientLocal.h"
#include "clientFrontend.h"
#include "clientBackend.h"
#include "clientFusion.h"
#include "localv1.h"
#include <iostream>
#include <string>
#include <QByteArray>
#include <QtNetwork>
#include <QDataStream>
#include <QMutexLocker>
#include <QThread>
#include <QString>
#include <vector>

#include "point.h"
// #include <QIODevice>

using namespace std;
using namespace zhpsm;

CClientLocal::CClientLocal(QObject* parent):
m_pFrontend(0),
m_pBackend(0),
work_model(0)
{
	// connect(this, SIGNAL(connected()),this,SLOT(startTransfer()));
	connect(this, SIGNAL(connected()),this,SLOT(startFrontend()));
	//connect(this, SIGNAL(bytesWritten(qint64)), this, SLOT(sendLaserFrame()));
	connect(this, SIGNAL(laserFrameReady()), this, SLOT(sendLaserFrame()));

	// rece Update Pose from server
	connect(this,SIGNAL(readyRead()), this, SLOT(readServer()));
}
CClientLocal::~CClientLocal(){}

void CClientLocal::quit(){
	cout<<"quit clientLocal!"<<endl;
	// close();
	finished();
}
void CClientLocal::quitAll(){
	cout<<"clientLocal.cpp: quitAll()!"<<endl;
	stopFrontend();
	// cout<<"after quit Frontend();"<<endl;
	stopBackend();
	// cout<<"after quit Backend();"<<endl;
	stopLocalization();
	// cout<<"after quit Localization()"<<endl;
	while(threadFrontend.isRunning() || threadBackend.isRunning() \
		|| threadLocalization.isRunning())
	{ 
		QThread::yieldCurrentThread();
		usleep(200000);
		cout<<"clientLocal.cpp: wait for other threads to stop!"<<endl;
	}
	finished();
}

void CClientLocal::setFile(const char* file){
	m_file = string(file);
}

void CClientLocal::setSICKIP(vector<string> ip, vector<unsigned int> port)
{
	sick_ip = ip;
	sick_port = port;
}

void CClientLocal::setModel(int model){
	work_model = model;
}

void CClientLocal::tryToConnect(){
	cout<<"try to connect to server!"<<endl;
	connectToHost(QHostAddress::LocalHost,6188);
}
void CClientLocal::startFrontend(){
	cout<<"succeed to connect to server!"<<endl;
	connectACK();

	// Frontend process
	m_pFrontend = new CClientFrontend;
	m_pFrontend->setFile(m_file);
	cout<<"clientFrontend"<<sick_ip.size()<<endl;
	m_pFrontend->setSICKIP(sick_ip, sick_port);
	m_pFrontend->setModel(work_model);
	m_pFrontend->moveToThread(&threadFrontend);
	connect(&threadFrontend,SIGNAL(started()),m_pFrontend,SLOT(runFrontEnd()));
	// quit down->up
	connect(m_pFrontend,SIGNAL(finished()),&threadFrontend,SLOT(quit()),Qt::DirectConnection);
	// connect(&threadFrontend,SIGNAL(finished()),this,SLOT(quit()),Qt::DirectConnection);
	// quit up->down
	connect(this,SIGNAL(stopFrontend()),m_pFrontend,SLOT(stop()),Qt::DirectConnection);

	// Backend process
	m_pBackend = new CClientBackend;
	m_pBackend->moveToThread(&threadBackend);
	m_pBackend->setModel(work_model);
	connect(&threadBackend,SIGNAL(started()),m_pBackend,SLOT(startBackend()));
	// quit
	connect(m_pBackend,SIGNAL(finished()),&threadBackend,SLOT(quit()),Qt::DirectConnection);
	connect(this, SIGNAL(stopBackend()),m_pBackend,SLOT(stopBackend()),Qt::DirectConnection);
	connect(&threadBackend,SIGNAL(finished()),this,SLOT(quit()));
	// connect(&threadFrontend,SIGNAL(finished()),SLOT(quit()));

	// localization process
	string imgfile = "/home/lyxp/work/mkproj/SVN/PFG/trunk/src/localization/maps/B2Map.jpg";
	// string imgfile = "/home/administrator/Desktop/B2Map.jpg";

	int start_x = 1393;
	int start_y = 517;
	m_pLocalization = new LocalV1;
	m_pLocalization->init(imgfile.c_str(),start_x,start_y);
	m_pLocalization->moveToThread(&threadLocalization);
	connect(&threadLocalization,SIGNAL(started()),m_pLocalization,SLOT(runLocalization()));
	connect(m_pLocalization,SIGNAL(finished()),&threadLocalization,SLOT(quit()),Qt::DirectConnection);
	connect(this,SIGNAL(stopLocalization()),m_pLocalization,SLOT(stop()),Qt::DirectConnection);
	// send to localization
	// connect(m_pFrontend,SIGNAL(sendFirstFrame(void*)),m_pLocalization,SLOT(receFirstFrame(void*)),Qt::DirectConnection);
	// connect(m_pFrontend,SIGNAL(sendScanFrame(void*,int)),m_pLocalization,SLOT(recePoseScan(void*,int)),Qt::DirectConnection);
	// connect(m_pLocalization,SIGNAL(sendUpdatePose(int, void*)),m_pFrontend,SLOT(receUpdatePose2(int, void*)),Qt::DirectConnection);
	// connect(m_pLocalization,SIGNAL(sendUpdatePose(int, void*, bool)),m_pFrontend,SLOT(receUpdatePose2(int, void*, bool)),Qt::DirectConnection);

	if(work_model != 3)
	{
		// send to localization
		connect(m_pFrontend,SIGNAL(sendFirstFrame(void*)),m_pLocalization,SLOT(receFirstFrame(void*)),Qt::DirectConnection);
		connect(m_pFrontend,SIGNAL(sendScanFrame(void*,int)),m_pLocalization,SLOT(recePoseScan(void*,int)),Qt::DirectConnection);
		// connect(m_pLocalization,SIGNAL(sendUpdatePose(int, void*)),m_pFrontend,SLOT(receUpdatePose2(int, void*)),Qt::DirectConnection);
 		connect(m_pLocalization,SIGNAL(sendUpdatePose(int, void*, bool)),m_pFrontend,SLOT(receUpdatePose2(int, void*, bool)),Qt::DirectConnection);


		// frontend -> backend
		// connect(m_pFrontend,SIGNAL(sendScanFrame(void*,int)),m_pBackend,SLOT(receScanFrame(void*,int)),Qt::DirectConnection);
		connect(m_pFrontend,SIGNAL(sendScanFrameCov(void*,int,double*)),m_pBackend,SLOT(receScanFrameCov(void*,int,double*)),Qt::DirectConnection);

		// backend -> frontend
		connect(m_pBackend,SIGNAL(sendUpdatePose(int,void*,double*)),m_pFrontend,SLOT(receUpdatePose(int,void*,double*)),Qt::DirectConnection);

		// send laser frame
		// connect(m_pFrontend,SIGNAL(sendFrameInfo(double,double,double,float*,int,int)),this,SLOT(receLaserFrame(double,double,double,float*,int,int)));
		// send update pose
		// connect(this,SIGNAL(sendUpdatePose(int,void*)),m_pFrontend,SLOT(receUpdatePose(int,void*)),Qt::DirectConnection);

		connect(m_pFrontend,SIGNAL(sendFrameInfo(double,double,double,float*,int)),this,SLOT(receLaserFrame(double,double,double,float*,int)));

	}
	else
	{


		//fusion
		m_pFrontend->setFusion();

		// frontend -> backend
		connect(m_pFrontend->m_pFusion,SIGNAL(sendScanFrameCov(void*,int,double*)),m_pBackend,SLOT(receScanFrameCov(void*,int,double*)),Qt::DirectConnection);

		// backend -> frontend
		connect(m_pBackend,SIGNAL(sendUpdatePose(int,void*,double*)),m_pFrontend->m_pFusion,SLOT(recvRelGlobalOptimized(int,void*,double*)),Qt::DirectConnection);

		//send to server for show
		connect(m_pFrontend->m_pFusion,SIGNAL(sendFrameInfo(double,double,double,float*,int)),this,SLOT(receLaserFrame(double,double,double,float*,int)));
		connect(m_pFrontend->m_pFusion,SIGNAL(finished()),&threadFrontend,SLOT(quit()),Qt::DirectConnection);

		// send to localization
		connect(m_pFrontend->m_pFusion,SIGNAL(sendFirstFrame(void*)),m_pLocalization,SLOT(receFirstFrame(void*)),Qt::DirectConnection);
		connect(m_pFrontend->m_pFusion,SIGNAL(sendScanFrame(void*,int)),m_pLocalization,SLOT(recePoseScan(void*,int)),Qt::DirectConnection);
		connect(m_pLocalization,SIGNAL(sendUpdatePose(int, void*, bool)),m_pFrontend->m_pFusion,SLOT(recvLocalizedPose(int, void*, bool)),Qt::DirectConnection);
		// connect(m_pLocalization,SIGNAL(sendUpdatePose(int, void*)),m_pFrontend->m_pFusion,SLOT(recvLocalizedPose(int, void*)),Qt::DirectConnection);


	}
	// startThread
	threadFrontend.start();
	//threadBackend.start();
	threadLocalization.start();
}

void CClientLocal::receLaserFrame(double x,double y,double th, float* bearing,int n)
{
	float *p = bearing;
	vector<float> scan(n,0);
 	// cout<<"CClientLocal::receLaserFrame x: "<<x<<" y: "<<y<<" th: "<<th<<endl;
	for(int i=0;i<n;i++,p++)
		scan[i] = *p;
	{
		QMutexLocker locker(&m_mutex);
		px.push_back(x);
		py.push_back(y);
		pth.push_back(th);
		// psyn.push_back(syn_num);
		m_bearing.push_back(scan);
	}
	laserFrameReady();
}
void CClientLocal::sendLaserFrame(){
	vector<float> x;
	vector<float> y;
	vector<float> th;
	// vector<int> syn;
	static int cnt=0;
	vector<vector<float> > scans;
	{
	QMutexLocker locker(&m_mutex);
	if(px.size()<=0) {
		return ;
	}
	x.swap(px);
	y.swap(py);
	th.swap(pth);
	scans.swap(m_bearing);
	}

	int n_points;
	static int control_threshold = 5*2000;
	static int bytesSize = sizeof(double)*3 + sizeof(int);
	for(int i=0;i<x.size();){
		// read from server is always first
		if(bytesAvailable() >= bytesSize){
			readServer(); 
		}
		// control stream 
		if(bytesToWrite() > control_threshold)
		{
			cout<<"wait for stream!"<<endl;
			QThread::yieldCurrentThread();	
			continue;
		}

		// send these data
		QByteArray block;
		QDataStream out(&block, QIODevice::WriteOnly);
		out<< quint16(0)<<x[i]<<y[i]<<th[i];
		n_points = scans[i].size();
		out<< quint16(n_points);
		for(int j=0;j<n_points;j++)
			out<<scans[i][j];
		out.device()->seek(0);
		out<<quint16(block.size()-sizeof(quint16));
		// cout<<"send "<<++cnt<<" frames: "<<x[i]<<" "<<y[i]<<" "<<th[i]<<endl;
		write(block);
		i++;
		// usleep(50000);
	}
}

void CClientLocal::readServer()
{
	static int bytesSize = sizeof(double)*3 + sizeof(int);
	QDataStream in(this);
	int id;
	double x,y,th;
	if(bytesAvailable() < bytesSize)
		return;
	while(bytesAvailable() >=  bytesSize)
	{
		in>>id>>x>>y>>th;
		cout<<"rece id: "<<id<<" pose: "<<x<<" "<<y<<" "<<th<<endl;
	}
	zhpsm::OrientedPoint2D pose(x,y,th);
	sendUpdatePose(id,(void*)(&pose));
}
