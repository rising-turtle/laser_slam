#include "slam.h"
#include "threadLocal1.h"
#include "threadLocal2.h"
#include "threadGlobal1.h"
#include <iostream>
#include <vector>
#include <algorithm>

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

CSlam::CSlam():
m_pThreadLocal1(new ThreadLocal1),
m_pThreadLocal2(new ThreadLocal2),
m_pThreadGlobal1(new ThreadGlobal1),
m_stopped(false),
m_received(false)
{}
CSlam::~CSlam(){
	delete m_pThreadLocal1;
	delete m_pThreadLocal2;
	delete m_pThreadGlobal1;
}

// these settings are important
void CSlam::setSystem()
{
	m_pThreadLocal2->moveToThread(&m_threadlocal2);
	m_pThreadGlobal1->moveToThread(&m_threadglobal1);

	QObject::connect(&m_threadlocal2,SIGNAL(started()),m_pThreadLocal2,SLOT(prepareMapNode()));
	QObject::connect(&m_threadglobal1,SIGNAL(started()),m_pThreadGlobal1,SLOT(addMapNode()));
	
	// quit process 
	QObject::connect(m_pThreadLocal1,SIGNAL(finished()),&m_threadlocal1,SLOT(quit()),Qt::DirectConnection);
	QObject::connect(m_pThreadLocal2,SIGNAL(finished()),&m_threadlocal2,SLOT(quit()),Qt::DirectConnection);
	QObject::connect(m_pThreadGlobal1,SIGNAL(finished()),&m_threadglobal1,SLOT(quit()),Qt::DirectConnection);

	// send btn_quit -> thread1 -> thread2 (send last MapNode) -> thread3 (record trajectory)
	QObject::connect(this,SIGNAL(stopAllThread()),m_pThreadLocal1,SLOT(stopThreadLocal1()),Qt::DirectConnection);
	QObject::connect(&m_threadlocal1,SIGNAL(finished()),m_pThreadLocal2,SLOT(stopThreadLocal2()),Qt::DirectConnection);
	QObject::connect(&m_threadlocal2,SIGNAL(finished()),m_pThreadGlobal1,SLOT(stopThreadGlobal()),Qt::DirectConnection);
	QObject::connect(&m_threadglobal1,SIGNAL(finished()),this,SLOT(stop()));
	
	// thread1's poseNode -> thread2
	QObject::connect(m_pThreadLocal1,SIGNAL(sendPoseNode(void*)),m_pThreadLocal2,SLOT(recePoseNode(void*)),Qt::DirectConnection);
	QObject::connect(m_pThreadLocal1,SIGNAL(sendCurrentPose(double,double,double)),this,SLOT(receCurrentPose(double,double,double)),Qt::DirectConnection);
	// thread2's mapNode -> thread3 
	QObject::connect(m_pThreadLocal2,SIGNAL(sendMapNode(void*)),m_pThreadGlobal1,SLOT(receMapNode(void*)),Qt::DirectConnection);
	// thread3's update -> thread1
	QObject::connect(m_pThreadGlobal1,SIGNAL(updateLocalPose(int,void*)),m_pThreadLocal1,SLOT(synFromGlobal(int,void*)),Qt::DirectConnection);

}

void CSlam::InitLog(string file_name, CallBack_SLAM cbslam){
	this->m_cbSLAM = cbslam;
	setSystem();
	m_pThreadLocal1->setLogPath(file_name.c_str());
	m_pThreadLocal1->moveToThread(&m_threadlocal1);
	QObject::connect(&m_threadlocal1,SIGNAL(started()),m_pThreadLocal1,SLOT(runLog()));
}

void CSlam::startEventLoop(int& argc, char** argv){
	qt_app = new QCoreApplication(argc,argv);
	qt_app->exec();
}

void CSlam::InitSick(string ip, int port, CallBack_SLAM cbslam){
	cout<<"Not enabled yet!"<<endl;
	return ;
}
void CSlam::run(){
	m_threadglobal1.start();
	m_threadlocal2.start();
	m_threadlocal1.start();
	while(!m_stopped){
		// wait for returned pose
		m_mutex.lock();
		while(m_received == false)
			m_wait_cond.wait(&m_mutex);
		if(m_cbSLAM!=NULL)
			m_cbSLAM(m_px,m_py,m_pth);
		m_received = false;
		m_mutex.unlock();
	}
	cout<<"threadSLAM quit!"<<endl;
}

void CSlam::receCurrentPose(double x, double y, double th)
{
	QMutexLocker lock(&m_mutex);
	m_px = x; m_py = y; m_pth = th;
	m_received = true;
	m_wait_cond.wakeOne();
}

void CSlam::stop(){
	if(!m_threadlocal1.isRunning())
		stopAllThread();
	while(m_threadglobal1.isRunning()){
		QThread::yieldCurrentThread();
	}
	cout<<"all subThreads are stopped!"<<endl;
	m_stopped = true;
}


