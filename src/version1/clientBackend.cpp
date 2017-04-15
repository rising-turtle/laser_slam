#include "clientBackend.h"
#include "threadG2OTcp.h"	
#include "threadMapNode.h"	
// #include "threadGlobal1.h"
// #include "threadLocal2.h"
#include "FlirterNode.h"
#include "ZHPolar_Match.h"
#include <iostream>
#include <QMutexLocker>

CClientBackend::CClientBackend(int model, QObject* parent):
work_model(model),
m_stop_backend(false),
m_pThreadLocal2(0),
m_pThreadGlobal1(0)
{
}
CClientBackend::~CClientBackend(){
	for(int i=0;i<m_traj.size();i++)
	{
		delete m_traj[i];
		m_traj[i] = 0;
	}
}

void CClientBackend::setModel(int model){
	work_model = model;
}

void CClientBackend::setConnections(){
	
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
}

void CClientBackend::startBackend(){
	cout<<"threadBackend start: "<<(int)QThread::currentThreadId()<<endl;
	switch(work_model){
	case 0: // RawSeed
		cout<<"Backend: RawSeed LMS211!"<<endl;
		m_pPSM = new CPolarMatch("LMS211");
		break;
	case 1: // Carmon
		cout<<"Backend: Carmon LMS151!"<<endl;
		m_pPSM = new CPolarMatch("LMS151");
		break;
	case 2:	// SICK
		cout<<"Backend: SICK LMS151!"<<endl;
		m_pPSM = new CPolarMatch("LMS151");
		break;
	case 3:	// FUSION
		cout<<"Backend: SICK LMS151!"<<endl;
		m_pPSM = new CPolarMatch("LMS151");
		break;
	default:
		cout<<"unknown work model!"<<endl;
		// quitAll();
		finished();
		break;
	}
	 m_pThreadLocal2 =  new ThreadMapNode;	
	 m_pThreadGlobal1 = new ThreadG2OTcp;	
	// m_pThreadLocal2 =  new ThreadLocal2;
	// m_pThreadGlobal1 =  new ThreadGlobal1;
	setConnections();
	threadMapNode.start();
	threadG2O.start();
	constructPoseNode();
}

bool CClientBackend::bigChange(OrientedPoint2D& pose){
#define BIGCHANGE_DIS 0.04 
#define BIGCHANGE_ANGLE 10*M_PI/180.
	static double sin_angle = sin(BIGCHANGE_ANGLE);
	if(pose.x*pose.x + pose.y*pose.y >= BIGCHANGE_DIS) // 0.2m 
		return true;
	if(sin(pose.theta) >= sin_angle) // 10'
		return true;
	return false;
}

void CClientBackend::receLoopInfo(int id, void* pose, double* cov){
	sendUpdatePose(id,pose,cov);
	return ;
}

void CClientBackend::receScanFrameCov(void* p, int syn, double* m)
{

	PMScan *pScan = static_cast<PMScan*>(p);
	//cout<<"CClientBackend::receScanFrameCov"<<pScan->rx<<" "<<pScan->ry<<" "<<pScan->th<<endl;
	vector<double> cov(6);
	double* pm = m;
	memcpy(&cov[0],pm,sizeof(double)*6);
	{
	QMutexLocker locker(&m_mutex);
	m_pScans.push_back(new PMScan(*pScan));
	m_pSyn.push_back(syn);
	m_pcov.push_back(cov);
	}

}
void CClientBackend::receScanFrame(void* p, int syn)
{
	PMScan *pScan = static_cast<PMScan*>(p);
	/*vector<float> ls(n);
	memcpy(&ls[0],p,n*sizeof(float));*/
	{
	QMutexLocker locker(&m_mutex);
	m_pScans.push_back(new PMScan(*pScan));
	m_pSyn.push_back(syn);
	/*px.push_back(x);
	py.push_back(y);
	pth.push_back(th);
	m_bearing.push_back(ls);*/
	}
}

void CClientBackend::constructPMScan(float x, float y, float th, vector<float>& scan, PMScan& ls)
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

void CClientBackend::sendtoDisplay(PMScan& ls)
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

void CClientBackend::constructPoseNode()
{
	/*vector<float> x;
	vector<float> y;
	vector<float> th;
	vector<vector< float> > scans;
	PMScan ls(m_pPSM->m_pParam->pm_l_points);*/
	PMScan* ls;
	vector<PMScan*> scans;
	vector<int> syns;
	vector< vector<double> > scov;
	cout<<"start to constructPoseNode!"<<endl;
	static int cnt=0;
	while(1){
		{
			QMutexLocker locker(&m_mutex);
			if(m_pScans.size()<=0){
				if(m_stop_backend)
					break;
				QThread::yieldCurrentThread();
				continue;
			}
			scans.swap(m_pScans);
			syns.swap(m_pSyn);
			scov.swap(m_pcov);
			/*x.swap(px);
			y.swap(py);
			th.swap(pth);
			scans.swap(m_bearing);*/
		}
		for(int i=0;i<scans.size();i++){
			// 1 construct PSM
			// constructPMScan(x[i],y[i],th[i],scans[i],ls);
			ls = scans[i];
			// to display this frame
		#ifdef DISPLAY
			// sendtoDisplay(ls);	
		#endif
			// 2 construct FlirtNode here
			CFliterNode* pcurNode = new CFliterNode(ls,m_pPSM);
			pcurNode->m_psyn= syns[i];
			pcurNode->m_id = m_traj.size();
			
			// 3 set cov for this node
			pcurNode->setCov(scov[i]);

			// cout<<"serverBacnend.cpp node: "<<pcurNode->m_id<<" has syn: "<<pcurNode->m_psyn<<endl;
			// cout<<"send PoseNode "<<++cnt<<endl;
			// 3 send this poseNode
			sendPoseNode((void*)pcurNode);
			// 4 record trajectory
			m_traj.push_back(new OrientedPoint2D(pcurNode->m_pose));
		}
		/*x.clear();
		y.clear();
		th.clear();
		scans.clear();*/
		scans.clear();
		syns.clear();
		scov.clear();
	}
	stopThreads();
	while(threadG2O.isRunning()){
		// cout<<"wait for threadGlobal to quit!"<<endl;
		QThread::yieldCurrentThread();
	}
	cout<<"quit threadBackend!"<<endl;
	finished();
}

void CClientBackend::stopBackend(){
	m_stop_backend = true;
}
