#include "threadLocal2.h"
#include <QMutexLocker>
#include <QThread>
#include "FlirterNode.h"
#include "MapNode.h"

int ThreadLocal2::s_thre_Tl = 50;
int ThreadLocal2::s_thre_Tg = 10;

ThreadLocal2::ThreadLocal2():
m_pMapNode(0),
m_stop_thread(false)
{}

void ThreadLocal2::recePoseNode(void* param)
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

void ThreadLocal2::stopThreadLocal2(){
	// cout<<"thread: "<<(int)QThread::currentThreadId()<<" stop threadLocal2!"<<endl;
	m_stop_thread = true;
}

void ThreadLocal2::prepareMapNode()
{
	int n_of_feature = 0;
	int n_of_nodes = 0;
	int cnt = 0;
	
	float px,py,pth,a,b;
	vector<CFliterNode*> nodes_buf;
	cout<<"prepareMapNode() in Thread2: "<<(int)QThread::currentThreadId()<<endl;
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
		for(int i=0;i<nodes_buf.size();i++){
			int n_added_featrue = m_pMapNode->addPoseNode(nodes_buf[i]);
			if(n_added_featrue == 0){
				delete nodes_buf[i];
				continue;
			}
			// cout<<"cur_node has: "<<nodes_buf[i]->m_featurePointsLocal.size()<<", and MapNode added: "<<n_added_featrue<<endl;
			n_of_nodes++;
			n_of_feature += n_added_featrue;
			if(n_of_feature >= s_thre_Tl && n_of_nodes>=s_thre_Tg){
				m_pMapNode->finishReduction();

				// draw the uncertainty of this mapNode
				m_pMapNode->getUncertainty(px,py,pth,a,b);
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
