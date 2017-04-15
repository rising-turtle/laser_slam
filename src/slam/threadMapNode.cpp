#include "threadMapNode.h"
#include <QMutexLocker>
#include <QThread>
#include "FlirterNode.h"
#include "MapNode.h"

int ThreadMapNode::s_thre_Tl = 120; // 150; //200; //300; //100;
int ThreadMapNode::s_thre_Tg = 10; // 20;  //8;
int ThreadMapNode::s_thre_Tn = 50; // 60 70 2; //100; //20;

ThreadMapNode::ThreadMapNode():
m_pMapNode(0),
m_bDisplay(0),
m_stop_thread(false)
{}

void ThreadMapNode::recePoseNode(void* param)
{
	CFliterNode* pNode = static_cast<CFliterNode*>(param);
	if(pNode == NULL){
		cout<<"Error in recePoseNode!"<<endl;
		return;
	}
	{
		QMutexLocker locker(&m_mutex);
		m_back_nodes.push_back(pNode);
	}
}


void ThreadMapNode::stopThreadMapNode(){
	// cout<<"thread: "<<(int)QThread::currentThreadId()<<" stop threadLocal2!"<<endl;
	m_stop_thread = true;
}

void ThreadMapNode::receSynInfo(int syn, void* old_p, void *new_p){
	OrientedPoint2D* oldp = static_cast<OrientedPoint2D*>(old_p);
	OrientedPoint2D* newp = static_cast<OrientedPoint2D*>(new_p);
	// m_synPose.insert(make_pair(syn,new OrientedPoint2D(*p)));
	m_synOldP.insert(make_pair(syn, new OrientedPoint2D(*oldp)));
	m_synNewP.insert(make_pair(syn, new OrientedPoint2D(*newp)));
}

void ThreadMapNode::synPoseTcp(CMapNode* node, int end){
	int start = node->m_psyn + 1;
	while(start <= end){
		map<int, OrientedPoint2D*>::iterator it_old = m_synOldP.find(start);
		map<int, OrientedPoint2D*>::iterator it_new = m_synNewP.find(start);
		if(it_old == m_synOldP.end() || it_new == m_synNewP.end()){
			cout<<"threadMapNode: error in synPoseTcp!"<<endl;
			return;
		}
		OrientedPoint2D trans = it_old->second->ominus(*(node->m_rootPose));
		*node->m_rootPose = it_new->second->oplus(trans);
		start++;
	}
	node->m_psyn = end;
	// *(node->m_rootPoseBack) = *(node->m_rootPose);
}

/*
void ThreadMapNode::synPoseTcp(CFliterNode* node, int end){
	int start = node->m_psyn;
	if(start != end){
		// cout<<"threadMapNode.cpp: mapNode syn: "<<end<<" poseNode id " <<node->m_id<<" syn: "<<node->m_psyn<<endl;
	}
	
	while(start!=end){
		map<int,OrientedPoint2D*>::iterator it = m_synPose.find(start);
		if(it == m_synPose.end()){
			cout<<"error in threadMapNode.cpp!"<<endl;
			return ;
		}
		OrientedPoint2D* relPose = it->second;
		node->m_pose = node->m_pose - *relPose;
		start--;
	}
}
*/

void ThreadMapNode::prepareMapNode()
{
	int n_of_feature = 0;
	int n_of_nodes = 0;
	int cnt = 0;
	
	float px,py,pth,a,b;
	vector<CFliterNode*> nodes_buf;
	cout<<"prepareMapNode() in Thread2: "<<(int)QThread::currentThreadId()<<endl;
	
	int id_of_curr_pose;
	ofstream outf("node_features.log");

	while(!m_stop_thread){
		{
			QMutexLocker locker(&m_mutex);
			if(m_back_nodes.size()<=0){
				//TODO: Switch to other threads!
				// cout<<"should switch to other threads!"<<endl;
				QThread::yieldCurrentThread();
				continue;
			}else{
				nodes_buf.swap(m_back_nodes);
			}
		}
		if(n_of_feature == 0 && m_pMapNode == 0){
			m_pMapNode = new CMapNode;
		}
		
		static bool once = true;

		for(int i=0;i<nodes_buf.size();i++){
			id_of_curr_pose = nodes_buf[i]->m_id;
			// this pose node is updated, while the mapNode is not
			if(nodes_buf[i]->m_psyn!=m_pMapNode->m_psyn && n_of_feature>0){
				// synPoseTcp(nodes_buf[i],m_pMapNode->m_psyn);
				synPoseTcp(m_pMapNode, nodes_buf[i]->m_psyn);
			}
			/*
			if(m_pMapNode->m_psyn == 1 && once){
				cout<<"first syn=1 node: "<< m_pMapNode->m_root_id<<" "<<*m_pMapNode->m_rootPose<<endl;
				once = false;
			}*/

			int n_added_featrue = m_pMapNode->addPoseNode(nodes_buf[i]);
			if(n_added_featrue == 0){
				delete nodes_buf[i];
				continue;
			}
			// cout<<"cur_node has: "<<nodes_buf[i]->m_featurePointsLocal.size()<<", and MapNode added: "<<n_added_featrue<<endl;
			n_of_nodes++;
			n_of_feature += n_added_featrue;
			if((n_of_feature >= s_thre_Tl && n_of_nodes>=s_thre_Tg) || (n_of_nodes >= s_thre_Tn)){
				// cout<<"threadMapNode: finishReduction()!"<<endl;
				m_pMapNode->finishReduction();
				
				outf<<m_pMapNode->m_root_id<<" "<<n_of_nodes<<" "<<n_of_feature<<endl;
			
				// draw the uncertainty of this mapNode
				m_pMapNode->getUncertainty(px,py,pth,a,b);
			#ifdef USE_PMAP
				// cout<<"threadMapNode: send PMAP: "<<m_pMapNode->m_obs_x.size()<<endl;
				if(m_bDisplay)
					sendPMAP((&m_pMapNode->m_obs_x[0]),&(m_pMapNode->m_obs_y[0]),(int)(m_pMapNode->m_obs_x.size()),(void*)(m_pMapNode->m_rootPose));
			#endif
				sendMapNode((void*)m_pMapNode);
				// cout<<"sendMapNodeUncertainty!"<<endl;
				// sendMapNodeUncertainty(px,py,pth,a,b);
				// cout<<"sendMapNode: "<<++cnt<<endl;
				m_pMapNode = new CMapNode;
				n_of_feature = 0;
				n_of_nodes = 0;
				continue;
			}
		}
		nodes_buf.clear();
	}
	// cout<<"quit thread2 loop!"<<endl;
	// send last node
	if(m_pMapNode->getNumofFeatures()>0)
	{
		m_pMapNode->finishReduction();
		sendMapNode((void*)m_pMapNode);
	}
	// cout<<"quit threadLocal2!"<<endl;
	// finish this thread
	finished();
}
