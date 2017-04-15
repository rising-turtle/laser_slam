#include "threadGlobal1.h"
#include <QMutexLocker>
#include <QThread>
#include "MapNode.h"
#include "MapGraph.h"

ThreadGlobal1::ThreadGlobal1():
// m_pMGraph(new CMapGraph("LMS151")),
m_pMGraph(0),
m_stop_thread(false),
m_saved_path("debug_traj.log")
{}

ThreadGlobal1::~ThreadGlobal1(){
	if(m_pMGraph!=0) delete m_pMGraph;
}

void ThreadGlobal1::receMapNode(void* param)
{
	CMapNode* pNode = static_cast<CMapNode*>(param);
	if(pNode == 0){
		cout<<"error in receMapNode()!"<<endl;
		return ;
	}
	{
		QMutexLocker locker(&m_mutex);
		m_back_nodes.push_back(pNode);
	}
}
/*
void ThreadGlobal1::synNodePose(CMapNode* node){
	OrientedPoint2D* pOrilast=0;
	OrientedPoint2D* pOricurr=0;
	OrientedPoint2D* pNewlast=0;
	for(int i=m_root_ids.size()-1; i>=0 ;i--){
		if(m_root_ids[i]== node->m_root_id)
		{
			if(i<=1) {return ;}
			pOrilast = ori_traj[m_root_ids[i-1]];
			pOricurr = ori_traj[m_root_ids[i]];
			pNewlast = new_traj[m_root_ids[i-1]];
			break;
		}
	}
	if(pOrilast==0 || pOricurr == 0 || pNewlast == 0)
		return ;
	*(node->m_rootPose) = pNewlast->oplus(pOrilast->ominus(*pOricurr));
	return ;
}
 */
bool ThreadGlobal1::bigChange(OrientedPoint2D& pose){
#define BIGCHANGE_DIS 0.04 
#define BIGCHANGE_ANGLE 10*M_PI/180.
	static double sin_angle = sin(BIGCHANGE_ANGLE);
	if(pose.x*pose.x + pose.y*pose.y >= BIGCHANGE_DIS) // 0.2m 
		return true;
	if(sin(pose.theta) >= sin_angle) // 10'
		return true;
	return false;
}

void ThreadGlobal1::addMapNode()
{
	cout<<"addMapNode() in Thread3: "<<(int)QThread::currentThreadId()<<endl;
	m_pMGraph = new CMapGraph("LMS151");
	vector<CMapNode*> buf_nodes;
	int cnt=0;
	while(1){
		{
			QMutexLocker locker(&m_mutex);
			if(m_back_nodes.size()<=0)
			{
				if(m_stop_thread) break;
				// TODO: switch to other threads!
				// cout<<"in addMapNode(), should switch!"<<endl;
				QThread::yieldCurrentThread(); 
				continue;
			}
			else
			{
				buf_nodes.swap(m_back_nodes);
			}
		}
		bool failed = false;
		vector<float> obs_x;
		vector<float> obs_y;
		for(int i=0;i<buf_nodes.size();i++)
		{
			// synNodePose(buf_nodes[i]);	
			OrientedPoint2D ori_pose = *(buf_nodes[i]->m_rootPose);
			cout<<"ori_pose: "<<ori_pose<<endl;

			// if(!m_pMGraph->addMapNodeGT(buf_nodes[i]))
			if(!m_pMGraph->addMapNode(buf_nodes[i]))
			{
				cout<<"MapNode failed to be added!"<<endl;
				delete buf_nodes[i];
				if(i==buf_nodes.size()-1) failed = true;
				continue;
			}
			cout<<"add MapNode: "<<++cnt<<endl;
			// TODO: display this mapNode
			// CMapNode* p = m_pMGraph->getLastNode();
			// cout<<"send MapNode: "<<cnt-1<<endl;
			// sendSubMap((void*)(p));

			if(failed) continue; // this should rarely happen
			CMapNode* plast = m_pMGraph->getLastNode();
			if(plast != 0)
			{
				OrientedPoint2D rel_change = ori_pose.ominus(*(plast->m_rootPose));
				if(bigChange(rel_change)){
					cout<<"send Update Info!"<<endl;
					// updateLocalPose((int)plast->m_root_id,(void*)(plast->m_rootPose));	
					updateLocalPose((int)plast->m_root_id,(void*)(&rel_change));
				}
				else{
					cout<<"rel_change: "<<rel_change<<endl;
				}
			}
		}
		buf_nodes.clear();
	}
	// cout<<"quit thread3 loop!"<<endl;
	saveTrajectory();
	// cout<<"quit threadLocal2!"<<endl;
	finished();
}

void ThreadGlobal1::saveTrajectory()
{
	cout<<"saveTrajectory() in thread3: "<<(int)QThread::currentThreadId()<<endl;
	cout<<"save final trajectory at: "<<endl;
	cout<<m_saved_path<<endl;
	m_pMGraph->optimizeGraph(10);
	m_pMGraph->recordTraj(m_saved_path.c_str());	
	m_pMGraph->recordG2O();
}

void ThreadGlobal1::stopThreadGlobal()
{
	// cout<<"thread: "<<(int)QThread::currentThreadId()<<" stop threadGlobal1!"<<endl;
	m_stop_thread = true;
}
