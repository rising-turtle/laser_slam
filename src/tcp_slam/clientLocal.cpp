#include "clientLocal.h"
#include "clientFrontend.h"
#include "clientFusion.h"
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
work_model(0)
{
	// connect(this, SIGNAL(connected()),this,SLOT(startTransfer()));
	connect(this, SIGNAL(connected()),this,SLOT(startFrontend()));
	//connect(this, SIGNAL(bytesWritten(qint64)), this, SLOT(sendLaserFrame()));
	connect(this, SIGNAL(laserFrameReady()), this, SLOT(sendLaserFrame()));
	connect(this, SIGNAL(laserFrameCovReady()), this, SLOT(sendLaserFrameCov()));

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
	stopFrontend();
}

void CClientLocal::setFile(const char* file){
	m_file = string(file);
}
void CClientLocal::setServerIP(QString ip, unsigned int port)
{
	server_ip = ip;
	server_port = port;
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
	//connectToHost(QHostAddress::LocalHost,6188);
	//connectToHost("192.168.1.100",6188);
	connectToHost(server_ip, server_port);
}
void CClientLocal::startFrontend(){
	cout<<"succeed to connect to server!"<<endl;
	connectACK();

	m_pFrontend = new CClientFrontend;
	m_pFrontend->setFile(m_file);
	cout<<"clientFrontend"<<sick_ip.size()<<endl;
	m_pFrontend->setSICKIP(sick_ip, sick_port);
	m_pFrontend->setModel(work_model);
	m_pFrontend->moveToThread(&threadFrontend);
	connect(&threadFrontend,SIGNAL(started()),m_pFrontend,SLOT(runFrontEnd()));
	// quit down->up
	connect(m_pFrontend,SIGNAL(finished()),&threadFrontend,SLOT(quit()),Qt::DirectConnection);
	connect(&threadFrontend,SIGNAL(finished()),this,SLOT(quit()),Qt::DirectConnection);
	// quit up->down
	connect(this,SIGNAL(stopFrontend()),m_pFrontend,SLOT(stop()),Qt::DirectConnection);
	// send laser frame
	// connect(m_pFrontend,SIGNAL(sendFrameInfo(double,double,double,float*,int,int)),this,SLOT(receLaserFrame(double,double,double,float*,int,int)));


	if(work_model == 3)
	{
		//fusion
		m_pFrontend->setFusion();
		connect(m_pFrontend->m_pFusion,SIGNAL(sendFrameInfo(double,double,double,float*,int,int)),this,SLOT(receLaserFrame(double,double,double,float*,int,int)),Qt::DirectConnection);
		connect(m_pFrontend->m_pFusion,SIGNAL(sendFrameInfoCov(double,double,double,float*,int,int,double*)),this,SLOT(receLaserFrameCov(double,double,double,float*,int,int,double*)),Qt::DirectConnection);
		connect(m_pFrontend->m_pFusion,SIGNAL(finished()),&threadFrontend,SLOT(quit()),Qt::DirectConnection);
		connect(this,SIGNAL(sendUpdatePose(int,void*,double*)),m_pFrontend->m_pFusion,SLOT(recvRelGlobalOptimized(int,void*,double*)),Qt::DirectConnection);
	}
	else
	{
		// send update pose
		connect(this,SIGNAL(sendUpdatePose(int,void*,double*)),m_pFrontend,SLOT(receUpdatePose(int,void*,double*)),Qt::DirectConnection);
		connect(m_pFrontend,SIGNAL(sendFrameInfo(double,double,double,float*,int,int)),this,SLOT(receLaserFrame(double,double,double,float*,int,int)),Qt::DirectConnection);
		connect(m_pFrontend,SIGNAL(sendFrameInfoCov(double,double,double,float*,int,int,double*)),this,SLOT(receLaserFrameCov(double,double,double,float*,int,int,double*)),Qt::DirectConnection);
	}
	// startThread
	threadFrontend.start();
}

void CClientLocal::receLaserFrame(double x,double y,double th, float* bearing,int n, int syn_num)
{
	float *p = bearing;
	vector<float> scan(n,0);
	// cout<<"receLaserinfo x: "<<x<<" y: "<<y<<" th: "<<th<<endl;
	for(int i=0;i<n;i++,p++)
		scan[i] = *p;
	{
		QMutexLocker locker(&m_mutex);
		px.push_back(x);
		py.push_back(y);
		pth.push_back(th);
		psyn.push_back(syn_num);
		m_bearing.push_back(scan);
	}
	laserFrameReady();
}

void CClientLocal::receLaserFrameCov(double x,double y,double th, float* bearing, int n, int syn_num, double* m)
{
	static int cnt=0;
	float *p = bearing;
	vector<float> scan(n,0);
	double *pm = m;
	for(int i=0;i<n;i++,p++)
		scan[i] = *p;
	vector<double> cov(6);
	copy(pm,pm+6,cov.begin());
	
	{
		QMutexLocker locker(&m_mutex);
		px.push_back(x);
		py.push_back(y);
		pth.push_back(th);
		psyn.push_back(syn_num);
		pcov.push_back(cov);
		m_bearing.push_back(scan);
	}
	laserFrameCovReady();

}

void CClientLocal::sendLaserFrameCov(){
	vector<float> x;
	vector<float> y;
	vector<float> th;
	vector<int> syn;
	vector< vector<double> > cov;
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
	syn.swap(psyn);
	scans.swap(m_bearing);
	cov.swap(pcov);
	}

	int n_points;
	static int control_threshold = 100*2000;
	static int bytesSize = sizeof(double)*9 + sizeof(int);
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
		out<< quint16(0)<<x[i]<<y[i]<<th[i]<<syn[i];
		for(int k=0;k<6;k++)
			out<<cov[i][k];
		n_points = scans[i].size();
		out<< quint16(n_points);
		for(int j=0;j<n_points;j++)
			out<<scans[i][j];
		out.device()->seek(0);
		out<<quint16(block.size()-sizeof(quint16));

		// cout<<"send "<<++cnt<<" frames cov: "<<cov[i][0]<<" "<<cov[i][3]<<" "<<cov[i][5]<<" syn: "<<syn[i]<<endl;
		write(block);
		i++;
	}
}

void CClientLocal::sendLaserFrame(){
	vector<float> x;
	vector<float> y;
	vector<float> th;
	vector<int> syn;
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
	syn.swap(psyn);
	scans.swap(m_bearing);
	}

	int n_points;
	static int control_threshold = 5*2000;
	static int bytesSize = sizeof(double)*9 + sizeof(int);
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
		out<< quint16(0)<<x[i]<<y[i]<<th[i]<<syn[i];
		n_points = scans[i].size();
		out<< quint16(n_points);
		for(int j=0;j<n_points;j++)
			out<<scans[i][j];
		out.device()->seek(0);
		out<<quint16(block.size()-sizeof(quint16));

		cout<<"send "<<++cnt<<" frames: "<<x[i]<<" "<<y[i]<<" "<<th[i]<<" syn: "<<syn[i]<<endl;
		write(block);
		i++;
	}
}

void CClientLocal::readServer()
{
	static int bytesSize = sizeof(double)*9 + sizeof(int);
	QDataStream in(this);
	int id;
	double x,y,th;
	double cov[6];
	if(bytesAvailable() < bytesSize)
		return;
	while(bytesAvailable() >=  bytesSize)
	{
		in>>id>>x>>y>>th;
		for(int i=0;i<6;i++)
			in>>cov[i];
		cout<<"rece id: "<<id<<" pose: "<<x<<" "<<y<<" "<<th<<" ";
		cout<<"cov: "<<cov[0]<<" "<<cov[3]<<" "<<cov[5]<<endl;
	}
	OrientedPoint2D pose(x,y,th);
	memcpy(glCov,cov,6*sizeof(double));
	sendUpdatePose(id,(void*)(&pose),glCov);
}
