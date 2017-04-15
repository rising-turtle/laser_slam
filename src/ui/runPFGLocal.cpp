#include "runPFGLocal.h"
#include <iostream>
#include <fstream>

#include "FlirterNode.h"
#include "ZHPolar_Match.h"
#include "MapNode.h"

// sick_reader
#include "CSICK.h"
#include "CObs2DScan.h"

#include <pthread.h>
#include <unistd.h>
using namespace std;

pthread_mutex_t CRunPFGLocal::s_mutex_prepare = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t CRunPFGLocal::s_mutex_update = PTHREAD_MUTEX_INITIALIZER;
pthread_t CRunPFGLocal::s_thread_prepare;
int CRunPFGLocal::s_thre_Tl = 200;
int CRunPFGLocal::s_thre_Tg = 10;

void* CRunPFGLocal::s_prepare_map_node(void* param)
{
	CRunPFGLocal *plocal = static_cast<CRunPFGLocal*>(param);
	if(plocal == NULL){
		cerr<<"run cast in s_prepare_map_node!"<<endl;
		exit(0);
	}
	int n_of_feature = 0;
	int n_of_nodes = 0;
	static int cnt=0;
	CMapNode* pMapNode=NULL;
	vector<CFliterNode*> nodes_buf;
	while(1){
		pthread_mutex_lock(&s_mutex_prepare);
		// cout<<"nodes size: "<<plocal->m_back_nodes.size()<<endl;
		if(plocal->m_back_nodes.size()<=0){
			if(plocal->m_b_flag_frontend_quit){
				if(pMapNode->getNumofFeatures()>0){
					plocal->sendMapNode(pMapNode);
					// cout<<"send MapNode: "<<++cnt<<endl;
				}
				pthread_mutex_unlock(&s_mutex_prepare);
				// cout<<"break++++++"<<endl;
				break;
			}
			pthread_mutex_unlock(&s_mutex_prepare);
			sleep(10);
			continue;
		}
			nodes_buf.swap(plocal->m_back_nodes);
		pthread_mutex_unlock(&s_mutex_prepare);
		if(n_of_feature ==0 && pMapNode == NULL){
			pMapNode = new CMapNode;
		}
		for(int i=0;i<nodes_buf.size();i++){
			int n_added = pMapNode->addPoseNode(nodes_buf[i]);
			if(n_added == 0){
				delete nodes_buf[i];
				continue;
			}
			n_of_nodes++;
			n_of_feature +=n_added;
			// the mapNode should be sent, and create a new one
			if(n_of_feature >= s_thre_Tl && n_of_nodes >= s_thre_Tg){
				// TODO: sendMapNode()
				pMapNode->finishReduction();
				plocal->sendMapNode((void*)(pMapNode));
				cout<<"send MapNode: "<<++cnt<<endl;
				pMapNode = new CMapNode;
				n_of_nodes = 0;
				n_of_feature = 0;
				continue;
			}
		}
		nodes_buf.clear();
	}	
	// For debug
	plocal->sendMapNode((void*)NULL);
	cout<<"send end code++++++++++++++++!"<<endl;
}

// set parameters for Local Slam 

CRunPFGLocal::CRunPFGLocal():
m_pPSM(new CPolarMatch("LMS151")),
m_b_flag_frontend_quit(false)
{}

void CRunPFGLocal::runlog(const char* filename)
{
        cout<<"in run log, filename: "<<filename<<endl;
	if(!m_pPSM->readCarmon(string(filename),m_pPSM->m_pParam->pm_laser_name))
	{
		cerr<<"failed to read file: "<<filename<<endl;
		return ;
	}
	
	// For debug
	// ofstream ofile("/mnt/hgfs/SharedFold/log/debug.log");

	// run pfg Local, here we do not use graph optimizition 
	int cnt=0;
	PMScan* ls;
	vector<float> obs_x;
	vector<float> obs_y;
	ofstream ofile1("/mnt/hgfs/SharedFold/dataset/lms151/debug.log");
	while(cnt<m_pPSM->m_SickScans.size())
	{
		ls = m_pPSM->m_SickScans[cnt];
		cnt++;
		CFliterNode* pcurNode = new CFliterNode(ls,m_pPSM);
		pcurNode->m_id = m_graph.size();
		runFrontEnd(pcurNode);
	}
	// send the last node
	pthread_mutex_lock(&CRunPFGLocal::s_mutex_prepare);
	m_back_nodes.push_back(m_graph[m_graph.size()-1]);
	pthread_mutex_unlock(&CRunPFGLocal::s_mutex_prepare);

	// to quit this thread
	m_b_flag_frontend_quit = true;
	
	// For debug
	/*for(int i=0;i<m_back_nodes.size();i++)
		ofile<<m_back_nodes[i]->m_pose<<endl;
	cout<<"finished!"<<endl;
	*/
	return ;
}

bool CRunPFGLocal::IsValidMotion(OrientedPoint2D& pose){
#define MOTION_THRE 0.0001 
#define ANGLE_THRE 0.0001  
	if(fabs(pose.x)+ fabs(pose.y) >= MOTION_THRE)
		return true;
	if(fabs(pose.theta) >= ANGLE_THRE)
		return true;
	return false;
}

bool CRunPFGLocal::runFrontEnd(CFliterNode* pcurNode)
{
	vector<float> obs_x;
	vector<float> obs_y;
	pthread_mutex_lock(&CRunPFGLocal::s_mutex_update);
	if(m_graph.size()<=0)
	{
		m_graph.insert(make_pair(0,pcurNode));
		translate2GlobalFrame(pcurNode->m_pScan->r,obs_x,obs_y,pcurNode->m_pose.x,pcurNode->m_pose.y,pcurNode->m_pose.theta);
		sendObservation(&obs_x[0],&obs_y[0],(int)obs_x.size(),&(pcurNode->m_pose.x),&(pcurNode->m_pose.y),&(pcurNode->m_pose.theta));

		pthread_mutex_unlock(&CRunPFGLocal::s_mutex_update);
		return true;

	}else{

		// CFliterNode* prefNode = m_graph[m_graph.size()-1];
		CFliterNode* prefNode = m_graph.rbegin()->second;
		try{
			if(!pcurNode->matchNodeFrontend(prefNode)){
				cout<<"@@@@@@@@@@@PSM failed!"<<endl;
				delete pcurNode;
				pthread_mutex_unlock(&CRunPFGLocal::s_mutex_update);
				return false;
			}
			if(!IsValidMotion(pcurNode->m_relpose))
			{	
				cout<<"~~~~~~~~Not move!"<<endl;
				delete pcurNode;
				pthread_mutex_unlock(&CRunPFGLocal::s_mutex_update);
				return false;
			}

			// cout<<"add PoseNode: "<<cnt<<endl;
			pcurNode->m_pose = prefNode->m_pose.oplus(pcurNode->m_relpose);
			m_graph.insert(make_pair(m_graph.size(),pcurNode));
			// send the last node into node buf
			if(m_graph.size()>=2){
				pthread_mutex_lock(&CRunPFGLocal::s_mutex_prepare);
				m_back_nodes.push_back(m_graph[m_graph.size()-2]);
				pthread_mutex_unlock(&CRunPFGLocal::s_mutex_prepare);
			}
			// TODO: send pcurNode to display this frame
			translate2GlobalFrame(pcurNode->m_pScan->r,obs_x,obs_y,pcurNode->m_pose.x,pcurNode->m_pose.y,pcurNode->m_pose.theta);
			sendObservation(&obs_x[0],&obs_y[0],(int)obs_x.size(),&(pcurNode->m_pose.x),&(pcurNode->m_pose.y),&(pcurNode->m_pose.theta));
		}catch(...){
			std::cout<<"catch wrong in RunPFGLocal runlog()!"<<std::endl;
			pthread_mutex_unlock(&CRunPFGLocal::s_mutex_update);
			return false;
		}
	}
	pthread_mutex_unlock(&CRunPFGLocal::s_mutex_update);
	return true;
}

PMScan* CRunPFGLocal::constructPSM(vector<float>& bearing, CPolarMatch* psm){
	if(psm->m_pParam->pm_l_points != bearing.size())
	{
		cout<<"Number of Bearings not right!"<<endl;
		return NULL;
	}
	static bool once=true;
#define MIN_151 -0.78525 // -45
#define FOV_151 4.7115  // 270
	if(once){
		psm->m_pParam->pm_fi_min = MIN_151;
		psm->m_pParam->pm_fi_max = MIN_151 + FOV_151;
		psm->pm_init();
		once = false;
	}
#ifndef PM_MIN_RANGE  
#define PM_MIN_RANGE 10.0f // 10 cm 
#endif
	PMScan* ret = new PMScan(psm->m_pParam->pm_l_points);
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

void CRunPFGLocal::runSick(vector<string> sick_ip, vector<unsigned int> sick_port)
{
	cout<<"in runSick()!"<<endl;
	// single SICK situation
	if(sick_ip.size()==1){
		cout<<"sick addr, ip: "<<sick_ip[0]<<" port: "<<sick_port[0]<<endl;
		CSICK laser(sick_ip[0],sick_port[0]);	
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
			ls = constructPSM(outObs.scan, m_pPSM);
			if(ls == NULL){
				cout<<"this frame maybe not right! "<<endl;
				continue;
			}
			CFliterNode* pcurNode = new CFliterNode(ls, m_pPSM, outObs.timestamp);
			pcurNode->m_id = m_graph.size();
			if(!runFrontEnd(pcurNode)){
				// cout<<"failed to add this frame!"<<endl;
			}else{
				// cout<<"++++++ succed to add node: +++++++"<<pcurNode->m_id<<endl;
			}
		}
	}
	else
	{	
		// TODO: When two Sicks are seleceted!
		cout<<"two lasers, not realized!"<<endl;
		cout<<"first ip: "<<sick_ip[0]<<" port: "<<sick_port[0]<<endl;
		cout<<"second ip: "<<sick_ip[1]<<" port: "<<sick_port[1]<<endl;
	}

	cout<<"!!!!!! SLAM LOCAL EXIT!"<<endl;
	return ;
}

void CRunPFGLocal::startLocal(bool blog, const char* str1,unsigned int port1, const char* str2,unsigned int port2)
{
	// create thread to send those poses
	if(pthread_create(&CRunPFGLocal::s_thread_prepare,NULL,CRunPFGLocal::s_prepare_map_node,(void*)(this))!=0){
		cout<<"failed to create thread LocalSLAM!"<<endl;
		return ;
	}
	cout<<"local slam active!"<<endl;
	// run log file
	if(blog){
		runlog(str1);
		return ;
	}
	vector<string> sick_ip;
	vector<unsigned int> sick_port;

	// TODO: valid check for sick_ip and sick_port
	string ip1(str1);
	if(ip1 != string("0.0.0.0") && port1>1024){
		sick_ip.push_back(ip1);
		sick_port.push_back(port1);
	}
	string ip2(str2);
	if(ip2 != string("0.0.0.0") && port2>1024){
		sick_ip.push_back(ip2);
		sick_port.push_back(port2);
	}
	// for debug
	cout<<"ip1: "<<ip1<<" port1: "<<port1<<endl;
	cout<<"ip2: "<<ip2<<" port2: "<<port2<<endl;
	runSick(sick_ip,sick_port);
	return ;
}

void CRunPFGLocal::synFromGlobal(int id, void* pose)
{
	cout<<"in synFromGlobal! graph.size(): "<<m_graph.size()<<", id: "<<id<<endl;
	OrientedPoint2D* new_pose = static_cast<OrientedPoint2D*> (pose);
	if(new_pose == NULL){
		cout<<"error in synFromGlobal!"<<endl;
		return ;
	}
	
	pthread_mutex_lock(&CRunPFGLocal::s_mutex_update);
		map<int, CFliterNode*>::iterator it = m_graph.find(id);
		if(it == m_graph.end()){
			cerr<<"error in synFromGlobal!"<<endl;
			pthread_mutex_unlock(&CRunPFGLocal::s_mutex_update);
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
	pthread_mutex_unlock(&CRunPFGLocal::s_mutex_update);
	
}

void CRunPFGLocal::translate2GlobalFrame(float* bearing, vector<float>& fx,vector<float>& fy, double rx, double ry, double th)
{
// [-45, 225] 270  
#define BEAR_NUM 541
#define PI 3.141592654
#define MAX_RANGE 50
const float bad_range = -100000; 
	static bool initAngle = false;
	static float cosAngle[BEAR_NUM];
	static float sinAngle[BEAR_NUM];
	static float Angle[BEAR_NUM];
	if(!initAngle){
		initAngle = true;
		static float minAngle =  -(PI)/4.0;  // 0.0;
		static float dfi = (float)((PI)/360.0); // dfi 0.5' 
		for(int i=0;i<BEAR_NUM;i++){
			Angle[i]=minAngle + dfi*i;
			cosAngle[i] = cosf(Angle[i]);
			sinAngle[i] = sinf(Angle[i]);
		}
		cout<<"minAngle: "<<minAngle<<"; maxAngle "<<Angle[BEAR_NUM-1]<<endl;
	}
	/*if(bearing.size()!=BEAR_NUM){
		cout<<"something is wrong!"<<endl;
	}*/

	if(fx.size()!=BEAR_NUM){
		fx.resize(BEAR_NUM);
		fy.resize(BEAR_NUM);
	}

	float x,y,tx,ty,nIdx,nIdy;
	float fcos = cosf(th);
	float fsin = sinf(th);
	for(int i=0;i<BEAR_NUM;i++){
		if(bearing[i]>MAX_RANGE*100){ // this is bad range 
			fx[i]=fy[i]=bad_range;
			continue;
		}
		x=(bearing[i]/100.0)*cosAngle[i];
		y=(bearing[i]/100.0)*sinAngle[i];
		tx=fcos*x-fsin*y+rx;
		ty=fsin*x+fcos*y+ry;

		fx[i] = tx;
		fy[i] = ty;
	}

}
