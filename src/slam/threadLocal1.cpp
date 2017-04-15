#include "threadLocal1.h"
#include <iostream>
#include <fstream>

#include "FlirterNode.h"
#include "ZHPolar_Match.h"
#include "MapNode.h"

// sick_reader
#include "CSICK.h"
#include "CObs2DScan.h"

#include <unistd.h>
#include <QMutexLocker>
#include <QThread>
using namespace std;

ThreadLocal1::ThreadLocal1():
	// m_pPSM(new CPolarMatch("LMS151")),
	m_pPSM(0), // all new resource must be initialized in main_loop
	m_stop_thread(false),
	m_file_name(""),
	m_SName(""),
	sick_ip(""),
	sick_port(0)
{}
ThreadLocal1::~ThreadLocal1(){
	if(m_pPSM!=0) delete m_pPSM;
}
void ThreadLocal1::stopThreadLocal1(){
	// cout<<"thread: "<<(int)QThread::currentThreadId()<<" stop threadLocal1"<<endl;
	m_stop_thread = true;
}

void ThreadLocal1::setLogPath(const char* path){
	m_file_name = string(path);
}

// This is for RawSeed
void ThreadLocal1::runRawSeed()
{
	cout<<"ThreadLocal1: runRawseed() "<<(int)QThread::currentThreadId()<<endl;

	m_pPSM = new CPolarMatch("LMS151");

	/*
	   m_pPSM->m_pParam->pm_dfi = PM_D2R;
	   m_pPSM->m_pParam->pm_fov = 180;
	   m_pPSM->m_pParam->pm_fi_min = -M_PI_2;
	   m_pPSM->m_pParam->pm_fi_max = M_PI_2;
	m_pPSM->m_pParam->pm_max_range = 8000;
	m_pPSM->m_pParam->pm_l_points = 181;
	m_pPSM->pm_init();
	*/


	PMScan ls(m_pPSM->m_pParam->pm_l_points);
	ls.rx = 0.;
	ls.ry = 0.;
	ls.t = 0.;
	ls.th = 0.;

	ifstream inf(m_file_name.c_str());
	if(!inf.is_open()){
		cout<<"in runRawseed() failed to open file: "<<m_file_name<<endl;
		return ;
	}

	char line[8192];
	double timestamp; //must be double
	string laser;
	int laserType, remissionMode, num_points;
	double start, fov, resolution, maxRange, accuracy;

	float r;
	int offset;
	string delim(" ");

	//ofstream ofs;
	//ofs.open("tmp.txt");
	while(!inf.eof() && inf.getline(line,8192) && !m_stop_thread)
	{
		timestamp = atof(strtok(line,delim.c_str()));
		/*laserType = (int)(atof(strtok(NULL,delim.c_str())));
		start = atof(strtok(NULL,delim.c_str()));
		fov = atof(strtok(NULL,delim.c_str()));
		resolution = atof(strtok(NULL,delim.c_str()));
		maxRange = atof(strtok(NULL,delim.c_str()));
		accuracy = atof(strtok(NULL,delim.c_str()));
		remissionMode = atoi(strtok(NULL,delim.c_str()));*/
		num_points = atoi(strtok(NULL,delim.c_str()));
		offset = atoi(strtok(NULL,delim.c_str()));
		//cout<<line<<endl;
		//cout<<timestamp<<" "<<num_points<<endl;

		if(m_pPSM->m_pParam->pm_l_points!=num_points){
			cout<<"not enough points in file!"<<endl;
			//return ;
			continue;
		}

		int cnt=0;
		for(int i=0;i<num_points;i++){
			++cnt;
			ls.r[i] = atof(strtok(NULL,delim.c_str()))*100.0; // from [m] 2 [cm]
			ls.x[i] = ls.r[i] * m_pPSM->pm_co[i];
			ls.y[i] = ls.r[i] * m_pPSM->pm_si[i];
			ls.bad[i] = 0;
			if(ls.r[i]<PM_MIN_RANGE){
				ls.r[i]=m_pPSM->m_pParam->pm_max_range+1; //set it to a value larger than the max
			}
			//cout<<ls.r[i]<<" "<<ls.x[i]<<" "<<ls.y[i]<<endl;
		}

		// five zeros : rx,ry,rth
		/*for(int i=0;i<5;i++){
			strtok(NULL,delim.c_str());
		}*/

		// get timestamp
		// timestamp = atof(strtok(NULL,delim.c_str()));

		CFliterNode* pcurNode = new CFliterNode(new PMScan(ls),m_pPSM);
		pcurNode->m_timestamp = (int64_t)(timestamp*10000000.0); // adding a timestamp (int64_t)
		// cout<<"pcurNode: "<<pcurNode->m_timestamp<<endl;
		pcurNode->m_id = m_graph.size();
		runFrontEnd(pcurNode);
	}


	// send the last node
	sendPoseNode((void*)(m_graph[m_graph.size()-1]));
	// cout<<"quit thread1 loop!"<<endl;
	// stop threadLocal1
	// cout<<"quit threadLocal1!"<<endl;
	finished();
	return ;

}

void ThreadLocal1::runLog()
{
	cout<<"runLog() threadLocal1: "<<(int)QThread::currentThreadId()<<endl;
	m_pPSM = new CPolarMatch("LMS151");
	// m_pPSM = new CPolarMatch("LMS211"); // For RawSeed data
	if(!m_pPSM->readCarmon(m_file_name,m_pPSM->m_pParam->pm_laser_name))
	{
		cerr<<"in runLog() failed to read file: "<<m_file_name<<endl;
		return ;
	}
	// run pfg Local, here we do not use graph optimizition 
	int cnt=0;
	PMScan* ls;
	vector<float> obs_x;
	vector<float> obs_y;
	while(cnt<m_pPSM->m_SickScans.size() && !m_stop_thread)
	{
		ls = m_pPSM->m_SickScans[cnt];
		cnt++;
		CFliterNode* pcurNode = new CFliterNode(ls,m_pPSM);
		pcurNode->m_id = m_graph.size();
		runFrontEnd(pcurNode);
	}
	// send the last node
	sendPoseNode((void*)(m_graph[m_graph.size()-1]));
	// cout<<"quit thread1 loop!"<<endl;
	// stop threadLocal1
	// cout<<"quit threadLocal1!"<<endl;
	finished();
	return ;
}

bool ThreadLocal1::IsValidMotion(OrientedPoint2D& pose){
#define MOTION_THRE 0.0001 
#define ANGLE_THRE 0.0001  
	if(fabs(pose.x)+ fabs(pose.y) >= MOTION_THRE)
		return true;
	if(fabs(pose.theta) >= ANGLE_THRE)
		return true;
	return false;
}
bool ThreadLocal1::smallMove(OrientedPoint2D& pose)
{
#define SMALL_MOTION 0.05*0.05
#define SMALL_ANGLE 5*PM_D2R
	if(pose.x*pose.x + pose.y*pose.y > SMALL_MOTION)
		return false;
	if(pose.theta > SMALL_ANGLE)
		return false;
	return true;
}
bool ThreadLocal1::runFrontEnd(CFliterNode* pcurNode)
{
	vector<float> obs_x;
	vector<float> obs_y;
	static OrientedPoint2D sent_pose; 
	{
		QMutexLocker locker(&m_mutex_update);
		if(m_graph.size()<=0)
		{
			m_graph.insert(make_pair(0,pcurNode));
			// send current pose
			sendCurrentPose(pcurNode->m_pose.x,pcurNode->m_pose.y,pcurNode->m_pose.theta);
			translate2GlobalFrame(pcurNode->m_pScan->r,obs_x,obs_y,pcurNode->m_pose.x,pcurNode->m_pose.y,pcurNode->m_pose.theta);
			sendObservation(&obs_x[0],&obs_y[0],(int)obs_x.size(),&(pcurNode->m_pose.x),&(pcurNode->m_pose.y),&(pcurNode->m_pose.theta));
			//sendUncertainty(pcurNode->m_dis_mean_x,pcurNode->m_dis_mean_y);
			paintReady();
			return true;

		}else{

			// CFliterNode* prefNode = m_graph[m_graph.size()-1];
			CFliterNode* prefNode = m_graph.rbegin()->second;
			try{
				// 0 Failed 1 PSM succeed 2 ICP succeed
				// int status = pcurNode->matchNodeFrontend(prefNode);
				// 0 Failed 1 CSM succeed
				// static int cnt = 0;
				// cout<<" the "<<++cnt<<" match!"<<endl;
				int status = pcurNode->matchNodeFrontend2(prefNode);

				/*if(!status)
			{
				cout<<"@@@@@@@@@@@PSM failed!"<<endl;
				delete pcurNode;
				return false;
			}*/
				if(status == 1 && !IsValidMotion(pcurNode->m_relpose))
				{
					cout<<"~~~~~~~~Not move!"<<endl;
					delete pcurNode;
					return false;
				}

				pcurNode->m_pose = prefNode->m_pose.oplus(pcurNode->m_relpose);
				// cout<<"CurPose Pose: "<<pcurNode->m_pose.x<<" "<<pcurNode->m_pose.y<<" "<<pcurNode->m_pose.theta<<endl;
				// cout<<"CurPose relPose: "<<pcurNode->m_relpose.x<<" "<<pcurNode->m_relpose.y<<" "<<pcurNode->m_relpose.theta<<endl;
				m_graph.insert(make_pair(m_graph.size(),pcurNode));
				// send the last node into node buf
				if(m_graph.size()>2){
					// QMutexLocker locker2(&m_mutex_prepare);
					// m_back_nodes.push_back(m_graph[m_graph.size()-2]);
					CFliterNode* cp = m_graph[m_graph.size()-2];
					OrientedPoint2D rel_pose = sent_pose.ominus(cp->m_pose);
					if(!smallMove(rel_pose)){
						sendPoseNode((void*)(m_graph[m_graph.size()-2]));
						sent_pose = cp->m_pose;
					}
				}
				sendCurrentPose(pcurNode->m_pose.x,pcurNode->m_pose.y,pcurNode->m_pose.theta);
				// TODO: send pcurNode to display this frame
				translate2GlobalFrame(pcurNode->m_pScan->r,obs_x,obs_y,pcurNode->m_pose.x,pcurNode->m_pose.y,pcurNode->m_pose.theta);
				sendObservation(&obs_x[0],&obs_y[0],(int)obs_x.size(),&(pcurNode->m_pose.x),&(pcurNode->m_pose.y),&(pcurNode->m_pose.theta));
				//sendUncertainty(pcurNode->m_dis_mean_x,pcurNode->m_dis_mean_y);
				paintReady();
			}catch(...){
				std::cout<<"catch wrong in RunPFGLocal runlog()!"<<std::endl;
				return false;
			}
		}
	}
	return true;
}

PMScan* ThreadLocal1::constructPSM(vector<float>& bearing, CPolarMatch* psm){
	if(psm->m_pParam->pm_l_points != bearing.size())
	{
		cout<<"Number of Bearings not right!"<<endl;
		return NULL;
	}
	PMScan* ret = new PMScan(psm->m_pParam->pm_l_points);
	ret->rx = 0.;
	ret->ry = 0.;
	ret->th = 0.;
	ret->t = 0.;
	for(int i=0;i<psm->m_pParam->pm_l_points;i++){

		ret->r[i] = bearing[i]*100.0; // PSM using cm 
		ret->x[i] = ret->r[i] * psm->pm_co[i];
		ret->y[i] = ret->r[i] * psm->pm_si[i];
		ret->bad[i] = 0;
		if(ret->r[i] < PM_MIN_RANGE || ret->r[i] < 0){
			ret->r[i] = psm->m_pParam->pm_max_range+ 1;
		}
	}
	return ret;
}

void ThreadLocal1::runSick(){
	cout<<"in runSick()!"<<endl;
	// single SICK situation

	cout<<"sick addr, ip: "<<sick_ip<<" port: "<<sick_port<<endl;
	CSICK laser(sick_ip,sick_port);
	CObs2DScan outObs;
	bool isOutObs, hardwareError;
	if(!laser.turnOn()){
		cout<<"failed to turnOn SICK!"<<endl;
		return ;
	}
	cout<<"succeed to turnOn SICK!"<<endl;
	laser.doProcessSimple(isOutObs, outObs, hardwareError);
	PMScan* ls;
	while(1){
		laser.doProcessSimple(isOutObs, outObs, hardwareError);
		if(hardwareError) continue;
		// TODO:
		m_pPSM = new CPolarMatch("LMS151");
		ls = constructPSM(outObs.scan, m_pPSM);
		if(ls == NULL){
			cout<<"this frame maybe not right! "<<endl;
			continue;
		}
		CFliterNode* pcurNode = new CFliterNode(ls, m_pPSM, outObs.timestamp);
		pcurNode->m_id = m_graph.size();
		if(!runFrontEnd(pcurNode)){
			cout<<"failed to add this frame!"<<endl;
		}else{
			cout<<"++++++ succed to add node: +++++++"<<pcurNode->m_id<<endl;
		}
	}

	cout<<"!!!!!! SLAM LOCAL EXIT!"<<endl;
	return ;
}
void ThreadLocal1::synFromGlobal(int id, void* pose)
{
	cout<<"in synFromGlobal! graph.size(): "<<m_graph.size()<<", id: "<<id<<endl;
	OrientedPoint2D* new_pose = static_cast<OrientedPoint2D*> (pose);
	if(new_pose == NULL){
		cout<<"error in synFromGlobal!"<<endl;
		return ;
	}
	{
		QMutexLocker locker(&m_mutex_update);
		map<int, CFliterNode*>::iterator it = m_graph.find(id);
		if(it == m_graph.end()){
			cerr<<"error in synFromGlobal!"<<endl;
			return ;
		}
		// only update the last node is enough
		// it->second->m_pose = *new_pose;
		CFliterNode* last = m_graph.rbegin()->second;
		last->m_pose = last->m_pose.oplus(it->second->m_pose.ominus(*new_pose));
		// update all the newly node
		/*while(++it_next!=m_graph.end()){
			it_next->second->m_pose = m_last_pose.oplus(it_next->second->m_relpose);
			m_last_pose = it_next->second->m_pose;
		}*/

		// delete these not valid pose
		/*if(it!=m_graph.begin()){
			m_graph.erase(m_graph.begin(),it);
		}*/
		// cout<<"~~~~~~clean observation!!!"<<endl;
		// to clean observations before
		cleanObservation((double)last->m_pose.x,(double)last->m_pose.y);
	}
}

void ThreadLocal1::translate2GlobalFrame(float* bearing, vector<float>& fx,vector<float>& fy, double rx, double ry, double th)
{
	const float bad_range = -100000; 
	int BEAR_NUM = m_pPSM->m_pParam->pm_l_points;
	// cout<<"BEAR_NUM: "<<BEAR_NUM<<endl;
	if(fx.size()!=BEAR_NUM){
		fx.resize(BEAR_NUM);
		fy.resize(BEAR_NUM);
	}

	float x,y,tx,ty,nIdx,nIdy;
	float fcos = cosf(th);
	float fsin = sinf(th);
	for(int i=0;i<BEAR_NUM;i++){
		if(bearing[i]>m_pPSM->m_pParam->pm_max_range){ // this is bad range 
			fx[i]=fy[i]=bad_range;
			continue;
		}
		x=(bearing[i]/100.0)*m_pPSM->pm_co[i];
		y=(bearing[i]/100.0)*m_pPSM->pm_si[i];

		tx=fcos*x-fsin*y+rx;
		ty=fsin*x+fcos*y+ry;

		/*tx = fsin*x+y*fcos+rx;
		ty = -fcos*x+y*fsin+ry;*/

		fx[i] = tx;
		fy[i] = ty;
	}
	// cout<<"finish translate!!"<<endl;
}
