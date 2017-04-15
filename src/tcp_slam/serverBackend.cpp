#include "serverBackend.h"
#include "threadG2OTcp.h"	// #include "threadG2OTcp.h"
#include "threadMapNode.h"	// #include "threadLocal2.h"
#include "FlirterNode.h"
#include "ZHPolar_Match.h"
#include <iostream>
#include <QMutexLocker>


CServerBackend::CServerBackend(int model, bool cov, QObject* parent):
work_model(model),
use_cov(cov),
m_stop_backend(false),
m_pThreadLocal2(0),
m_pThreadGlobal1(0)
{
}
CServerBackend::~CServerBackend(){
	for(int i=0;i<m_traj.size();i++)
	{
		delete m_traj[i];
		m_traj[i] = 0;
	}
}

void CServerBackend::setModel(int model){
	work_model = model;
}

void CServerBackend::setConnections(){
	
	m_pThreadLocal2->moveToThread(&threadMapNode);
	m_pThreadGlobal1->moveToThread(&threadG2O);

	// start connections
	connect(&threadMapNode,SIGNAL(started()),m_pThreadLocal2,SLOT(prepareMapNode()));
	connect(&threadG2O,SIGNAL(started()),m_pThreadGlobal1,SLOT(addMapNode()));

	// quit connections
	connect(m_pThreadLocal2,SIGNAL(finished()),&threadMapNode,SLOT(quit()),Qt::DirectConnection);
	connect(m_pThreadGlobal1,SIGNAL(finished()),&threadG2O,SLOT(quit()),Qt::DirectConnection);

	connect(this, SIGNAL(stopThreads()),m_pThreadLocal2,SLOT(stopThreadMapNode()),Qt::DirectConnection);
	connect(&threadMapNode,SIGNAL(finished()),m_pThreadGlobal1,SLOT(stopThreadGlobal()),Qt::DirectConnection);

	// server_socket(raw scan) -> server_backend(pose node) -> thread_mapNode(map node) -> thread_G2O
	connect(this,SIGNAL(sendPoseNode(void*)),m_pThreadLocal2,SLOT(recePoseNode(void*)),Qt::DirectConnection);
	connect(m_pThreadLocal2,SIGNAL(sendMapNode(void*)),m_pThreadGlobal1,SLOT(receMapNode(void*)),Qt::DirectConnection);
	// thread_G2O(loop closure) -> server_backend(pose)
	connect(m_pThreadGlobal1,SIGNAL(updateLocalPose(int,void*,double*)),this,SLOT(receLoopInfo(int,void*,double*)),Qt::DirectConnection);
	// thread_G2O(syn pose) -> server_backend(syn pose)
	connect(m_pThreadGlobal1,SIGNAL(updateSynInfo(int,void*,void*)),m_pThreadLocal2, SLOT(receSynInfo(int,void*,void*)),Qt::DirectConnection);

// #ifdef USE_PMAP
	// display PMAP threadLocal2-> serverBackend
	connect(m_pThreadLocal2,SIGNAL(sendPMAP(float*,float*,int,void*)),this,SLOT(recePMAP(float*,float*,int,void*)),Qt::DirectConnection);
// #endif
}

void CServerBackend::startBackend(){
	cout<<"threadBackend start: "<<(int)QThread::currentThreadId()<<endl;
	switch(work_model){
	case 0: // RawSeed
		cout<<"Backend: RawSeed LMS211!"<<endl;
		m_pPSM = new CPolarMatch("LMS211");
		break;
	case 1: // Carmon
		cout<<"Backend: Carmon LMS151!"<<endl;
		// m_pPSM = new CPolarMatch("LMS511");
		// m_pPSM = new CPolarMatch("LMS211");
		m_pPSM = new CPolarMatch("LMS151");
		break;
	case 2:	// SICK
		cout<<"Backend: SICK LMS151!"<<endl;
		m_pPSM = new CPolarMatch("LMS151");
		break;
	default:
		cout<<"unknown work model!"<<endl;
		// quitAll();
		finished();
		break;
	}
	m_pThreadLocal2 =  new ThreadMapNode;	// new ThreadLocal2;
	m_pThreadGlobal1 = new ThreadG2OTcp;	// new ThreadGlobal1;
	setConnections();
	threadMapNode.start();
	threadG2O.start();
	constructPoseNode();
}

bool CServerBackend::bigChange(OrientedPoint2D& pose){
#define BIGCHANGE_DIS 0.04 
#define BIGCHANGE_ANGLE 10*M_PI/180.
	static double sin_angle = sin(BIGCHANGE_ANGLE);
	if(pose.x*pose.x + pose.y*pose.y >= BIGCHANGE_DIS) // 0.2m 
		return true;
	if(sin(pose.theta) >= sin_angle) // 10'
		return true;
	return false;
}

void CServerBackend::receLoopInfo(int id, void* pose, double* cov){
	// cout<<"loop Info has not been realized!"<<endl;
	OrientedPoint2D * p = static_cast<OrientedPoint2D*>(pose);
	if(p == NULL) return ;
	if(id < 0 || id >= m_traj.size()) {
		cout<<"error happen in CServerBackend::receLoopInfo()!"<<endl;
		return;
	}
	sendUpdatePose(id, p->x, p->y, p->theta, cov);

	/*
	OrientedPoint2D * prev = m_traj[id];
	OrientedPoint2D poseChange = prev->ominus(*p);
	// if the change is too small, we do not send this update
	// big change for this pose!
	if(bigChange(poseChange)){
		// send this update info to client
		// cout<<"send update info to client!"<<endl;
		sendUpdatePose(id, p->x, p->y, p->theta);
	}
	*/
	return ;
}

void CServerBackend::recePMAP(float* x, float* y, int n, void* pose)
{
	// cout<<"serverBackend: rece PMAP!"<<endl;
	OrientedPoint2D* p = static_cast<OrientedPoint2D*>(pose);
	// cout<<"send PMAP to Display!"<<endl;
	sendPMAP2Dis(x,y,n,p->x,p->y,p->theta);
	paintReady();
}

void CServerBackend::receScanFrameCov(float x, float y, float th, float * scan, int n, int syn_num, double* m)
{
	float *p = scan;
	vector<float> ls(n);
	memcpy(&ls[0],p,n*sizeof(float));
	double *pm = m;
	vector<double> cov(6);
	memcpy(&cov[0],pm,6*sizeof(double));
	{
	QMutexLocker locker(&m_mutex);
	px.push_back(x);
	py.push_back(y);
	pth.push_back(th);
	psyn.push_back(syn_num);
	pcov.push_back(cov);
	m_bearing.push_back(ls);
	}

}

void CServerBackend::receScanFrame(float x, float y, float th, float* scan, int n, int syn_num)
{
	float *p = scan;
	vector<float> ls(n);
	memcpy(&ls[0],p,n*sizeof(float));
	{
	QMutexLocker locker(&m_mutex);
	px.push_back(x);
	py.push_back(y);
	pth.push_back(th);
	psyn.push_back(syn_num);
	m_bearing.push_back(ls);
	}
}

void CServerBackend::constructPMScan(float x, float y, float th, vector<float>& scan, PMScan& ls)
{
	if(scan.size()!=m_pPSM->m_pParam->pm_l_points){
		cout<<"error in Backend:constructPMScan"<<endl;
		return;
	}
	ls.rx = x*100.;
	ls.ry = y*100.;
	ls.th = th;
	for(int i=0;i<scan.size();i++){
		ls.r[i] = scan[i];
		ls.x[i] = ls.r[i]*m_pPSM->pm_co[i];
		ls.y[i] = ls.r[i]*m_pPSM->pm_si[i];
		ls.bad[i] = 0;
		if(ls.r[i] < m_pPSM->m_pParam->pm_min_range){
			ls.r[i] = m_pPSM->m_pParam->pm_max_range+1;
		}
	}
	return ;
}

void CServerBackend::sendtoDisplay(PMScan& ls)
{
	const float bad_range = -100000;
	static vector<float> fx, fy;
	static double rx,ry,rth;
	if(fx.size() < ls.np) {
		fx.resize(ls.np);
		fy.resize(ls.np);
	}
	
	rx = ls.rx/100.;
	ry = ls.ry/100.;
	rth = ls.th;
	float x,y;
	float fcos = cosf(rth);
	float fsin = sinf(rth);
	for(int i=0;i<ls.np;i++)
	{
		if(ls.r[i] >= m_pPSM->m_pParam->pm_max_range)
		{
			fx[i] = fy[i] = bad_range;
		}else{
			x = ls.x[i]/100.;
			y = ls.y[i]/100.;
			fx[i] = fcos*x - fsin*y + rx;
			fy[i] = fsin*x + fcos*y + ry;
		}
	}
	sendMapInfo(&fx[0],&fy[0],ls.np,&rx,&ry,&rth);
	paintReady();
}

void CServerBackend::constructPoseNode()
{
	vector<float> x;
	vector<float> y;
	vector<float> th;
	vector<int> syn;
	vector<vector< float> > scans;
	vector<vector<double> > cov;
	PMScan ls(m_pPSM->m_pParam->pm_l_points);
	cout<<"start to constructPoseNode!"<<endl;
	static int cnt=0;
	while(1){
		{
			QMutexLocker locker(&m_mutex);
			if(px.size()<=0){
				if(m_stop_backend)
					break;
				QThread::yieldCurrentThread();
				continue;
			}
			x.swap(px);
			y.swap(py);
			th.swap(pth);
			syn.swap(psyn);
			cov.swap(pcov);
			scans.swap(m_bearing);
		}
		for(int i=0;i<x.size();i++){
			// 1 construct PSM
			constructPMScan(x[i],y[i],th[i],scans[i],ls);
			// to display this frame
			if(DISPLAY)
				sendtoDisplay(ls);	
			// 2 construct FlirtNode here
			CFliterNode* pcurNode = new CFliterNode(new PMScan(ls),m_pPSM);
			pcurNode->m_psyn= syn[i];
			pcurNode->m_id = m_traj.size();
			if(use_cov)
			{
				pcurNode->setCov(cov[i]);
			}
			// cout<<"serverBacnend.cpp node: "<<pcurNode->m_id<<" has syn: "<<pcurNode->m_psyn<<endl;
			// cout<<"send PoseNode "<<++cnt<<endl;
			// 3 send this poseNode
			sendPoseNode((void*)pcurNode);
			// 4 record trajectory
			m_traj.push_back(new OrientedPoint2D(x[i],y[i],th[i]));
		}
		x.clear();
		y.clear();
		th.clear();
		syn.clear();
		scans.clear();
		cov.clear();
	}
	stopThreads();
	while(threadG2O.isRunning()){
		// cout<<"wait for threadGlobal to quit!"<<endl;
		QThread::yieldCurrentThread();
	}
	cout<<"quit threadBackend!"<<endl;
	finished();
}

void CServerBackend::stopBackend(){
	m_stop_backend = true;
}
