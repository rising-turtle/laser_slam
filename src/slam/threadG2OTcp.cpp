#include "threadG2OTcp.h"
#include <QMutexLocker>
#include <QThread>
#include "MapNode.h"
#include "MapGraph.h"

ThreadG2OTcp::ThreadG2OTcp():
// m_pMGraph(new CMapGraph("LMS151")),
m_pMGraph(0),
m_psyn(0),
m_stop_thread(false),
m_saved_path("debug_traj.log")
{}

ThreadG2OTcp::~ThreadG2OTcp(){
	if(m_pMGraph!=0) delete m_pMGraph;
	{
	map<int, OrientedPoint2D*>::iterator it = ori_traj.begin(); 
	while(it!=ori_traj.end()){
		delete (it->second);
		it++;
	}
	map<int,OrientedPoint2D*> tmp;
	ori_traj.swap(tmp);
	}
	{
	map<int, OrientedPoint2D*>::iterator it = old_pose.begin(); 
	while(it!=old_pose.end()){
		delete (it->second);
		it++;
	}
	map<int,OrientedPoint2D*> tmp;
	old_pose.swap(tmp);
	}
	{
	map<int, OrientedPoint2D*>::iterator it = new_pose.begin(); 
	while(it!=new_pose.end()){
		delete (it->second);
		it++;
	}
	map<int,OrientedPoint2D*> tmp;
	new_pose.swap(tmp);
	}
}

void ThreadG2OTcp::receMapNode(void* param)
{
	CMapNode* pNode = static_cast<CMapNode*>(param);
	if(pNode == 0){
		cout<<"error in receMapNode()!"<<endl;
		return ;
	}
	{
		QMutexLocker locker(&m_mutex);
		m_back_nodes.push_back(pNode);
		/*m_root_ids.push_back(pNode->m_root_id);
		ori_traj.insert(make_pair(pNode->m_root_id, new OrientedPoint2D(*pNode->m_rootPose)));
		new_traj.insert(make_pair(pNode->m_root_id, new OrientedPoint2D(*pNode->m_rootPose)));*/
	}
}

void ThreadG2OTcp::synNodePose(CMapNode* node)
{
	if(m_psyn != node->m_psyn){
		int start = node->m_psyn+1;
		while(start <= m_psyn){
			map<int, OrientedPoint2D*>::iterator it_old = old_pose.find(start);
			map<int, OrientedPoint2D*>::iterator it_new = new_pose.find(start);
			if(it_old == old_pose.end() || it_new == new_pose.end()){
				cout<<"threadG2OTcp: error in synNodePose()!"<<endl;
				return ;
			}
			OrientedPoint2D trans = it_old->second->ominus(*(node->m_rootPose));
			*(node->m_rootPose) = it_new->second->oplus(trans);
			start++;
		}
	}
	// *(node->m_rootPoseBack) = *(node->m_rootPose);
	ori_traj.insert(make_pair(node->m_root_id,new OrientedPoint2D(*(node->m_rootPose))));
	node->m_psyn = m_psyn;
}

/*
void ThreadG2OTcp::synNodePose(CMapNode* node){
	// already updated pose
	if(m_psyn != node->m_psyn){
		cout<<"node->syn: "<<node->m_psyn<<" G2O syn: "<<m_psyn<<endl;
		int start = node->m_psyn+1;
		while(start <= m_psyn){
			map<int, OrientedPoint2D*>::iterator it = rel_traj.find(start);
			if(it == rel_traj.end()){
				cout<<"error in threadG2OTcp.cpp synNodePose()!"<<endl;
				return ;
			}
			*(node->m_rootPose) = *(node->m_rootPose) + (*(it->second)); 
			// cout<<*(node->m_rootPose)<<endl;
			start++;
		}
	}
	*(node->m_rootPoseBack) = *(node->m_rootPose);
	ori_traj.insert(make_pair(node->m_root_id,new OrientedPoint2D(*(node->m_rootPose))));
}
*/

bool ThreadG2OTcp::bigChange(OrientedPoint2D& pose){
#define BIGCHANGE_DIS 0.01 
#define BIGCHANGE_ANGLE 5*M_PI/180.
	static double sin_angle = sin(BIGCHANGE_ANGLE);
	if(pose.x*pose.x + pose.y*pose.y >= BIGCHANGE_DIS) // 0.2m 
		return true;
	if(sin(pose.theta) >= sin_angle) // 10'
		return true;
	return false;
}

void ThreadG2OTcp::fromRel2AbsPose(OrientedPoint2D& cor, OrientedPoint2D& rel, OrientedPoint2D& abs)
{
	float fcos = cos(cor.theta);
	float fsin = sin(cor.theta);
	abs.x = rel.x * fcos - rel.y * fsin;
	abs.y = rel.x * fsin + rel.y * fcos;
	abs.theta = rel.theta;
}
namespace{
	void getMatrix(double m[6], Eigen::Matrix3d& mr){
		m[0] = mr(0,0); m[1] = mr(0,1); m[2] = mr(0,2);
				m[3] = mr(1,1); m[4] = mr(1,2);
						m[5] = mr(2,2);
	}
}
void ThreadG2OTcp::addMapNode()
{
	cout<<"addMapNode() in Thread3: "<<(int)QThread::currentThreadId()<<endl;
	m_pMGraph = new CMapGraph("LMS151");
	vector<CMapNode*> buf_nodes;
	int cnt=0;
	double cov[6];
	bool bloop;
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
			bloop = false;	
			// syn pose 
			synNodePose(buf_nodes[i]);	
			// if(!m_pMGraph->addMapNodeGT(buf_nodes[i]))
			// if(!m_pMGraph->addMapNode(buf_nodes[i]))
			if(!m_pMGraph->addMapNodeCov(buf_nodes[i],cov,bloop))
			{
				cout<<"MapNode failed to be added!"<<endl;
				delete buf_nodes[i];
				if(i==buf_nodes.size()-1) failed = true;
				continue;
			}
			// cout<<"add MapNode: "<<++cnt<<endl;
			// TODO: display this mapNode
			// CMapNode* p = m_pMGraph->getLastNode();
			// cout<<"send MapNode: "<<cnt-1<<endl;
			// sendSubMap((void*)(p));
			if(failed) continue; // this should rarely happen
			CMapNode* plast = m_pMGraph->getLastNode();
			if(plast != 0)
			{
				OrientedPoint2D old_p = *(ori_traj[plast->m_root_id]);
				OrientedPoint2D new_p = *(plast->m_rootPose);
				OrientedPoint2D rel_change = ori_traj[plast->m_root_id]->ominus(*(plast->m_rootPose));
				OrientedPoint2D abs_change;
				double m[6];
				if(bigChange(rel_change) || bloop){
					// outf<<"trans is: "<<rel_change<<endl;
					m_psyn++;
					fromRel2AbsPose(*(ori_traj[plast->m_root_id]),rel_change,abs_change);
					// rel_traj.insert(make_pair(m_psyn,new OrientedPoint2D(abs_change)));
					old_pose.insert(make_pair(m_psyn, new OrientedPoint2D(old_p)));
					new_pose.insert(make_pair(m_psyn, new OrientedPoint2D(new_p)));
					updateSynInfo(m_psyn, (void*)(&old_p),(void*)(&new_p));
					// updateSynInfo(m_psyn,(void*)(&abs_change));
					// updateLocalPose((int)plast->m_root_id,(void*)(plast->m_rootPose));	
					if(!m_pMGraph->b_delayed_g2o){
						getMatrix(m,plast->m_covariance);
					}else{
						memcpy(m,cov,sizeof(double)*6);
					}
					updateLocalPose((int)plast->m_root_id,(void*)(&abs_change),m);
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

void ThreadG2OTcp::saveTrajectory()
{
	cout<<"saveTrajectory() in thread3: "<<(int)QThread::currentThreadId()<<endl;
	cout<<"save final trajectory at: "<<endl;
	cout<<m_saved_path<<endl;
	/*if(m_pMGraph->b_delayed_g2o){
		cout<<"start to construct DelayedG2O!"<<endl;
		if(!m_pMGraph->constructDelayedG2o()){
			cout<<"failed to construct delayed g2o!"<<endl;
			return ;
		}
	}*/
	m_pMGraph->recordG2O();
	// m_pMGraph->recordTraj("before_op.log");
	m_pMGraph->optimizeGraph(100);
	m_pMGraph->recordTraj(m_saved_path.c_str());	
	m_pMGraph->recordMap("map.log");
	// m_pMGraph->recordFuseMap("fusemap.log");
}

void ThreadG2OTcp::stopThreadGlobal()
{
	// cout<<"thread: "<<(int)QThread::currentThreadId()<<" stop threadGlobal1!"<<endl;
	m_stop_thread = true;
}
