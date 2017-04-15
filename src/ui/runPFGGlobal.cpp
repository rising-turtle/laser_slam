#include "runPFGGlobal.h"
#include "MapNode.h"
#include "MapGraph.h"

#include <iostream>
#include <fstream>

pthread_mutex_t CRunPFGGlobal::s_node_prepare = PTHREAD_MUTEX_INITIALIZER;
pthread_t CRunPFGGlobal::s_thread_prepare;
void CRunPFGGlobal::startGlobal(){
	if(pthread_create(&CRunPFGGlobal::s_thread_prepare,NULL,CRunPFGGlobal::s_add_mapNode,(void*)(this))!=0){
		cout<<"failed to create thread GLOBAL SLAM!"<<endl;
		return ;
	}
	cout<<"global slam active!"<<endl;
}
void* CRunPFGGlobal::s_add_mapNode(void* p)
{
	static int cnt = 0;
	CRunPFGGlobal* pglobal = static_cast<CRunPFGGlobal*>(p);
	if(pglobal == NULL){
		cerr<<"thread s_add_mapNode fail to initialize"<<endl;
		exit(0);
	}
	
	vector<CMapNode*> buf_node;
	bool quit_backend = false;
	while(1){
		// For debug
		if(quit_backend){
			pglobal->m_mapGraph->recordTraj("debug.log");
			cout<<"finished!"<<endl;
			break;
		}
		pthread_mutex_lock(&CRunPFGGlobal::s_node_prepare);
			if(pglobal->m_buf_node.size()<=0){
					if(pglobal->m_b_flag_backend_quit){
						cout<<"back_end now quit!"<<endl;
						quit_backend = true;
						}
				pthread_mutex_unlock(&CRunPFGGlobal::s_node_prepare);
				sleep(10);
				continue;
			}
			buf_node.swap(pglobal->m_buf_node);
		pthread_mutex_unlock(&CRunPFGGlobal::s_node_prepare);
		bool failed = false;
		for(int i=0;i<buf_node.size();i++){
			// add this node into mapGraph
			if(!pglobal->m_mapGraph->addMapNodeGT(buf_node[i])){
				cout<<"map_node fail to be added!"<<endl;
				delete buf_node[i];
				if(i==buf_node.size()-1)
					failed = true;
			}
			cout<<"add MapNode: "<<++cnt<<endl;
			// display this mapNode
			vector<float > obs_x;
			vector<float > obs_y;
			CMapNode* p = pglobal->m_mapGraph->getLastNode();
			cout<<"send MapNode: "<<cnt-1<<endl;
			pglobal->sendSubmap((void*)(p));
		}
		if(failed) continue; // this should rarely happen
		CMapNode* plast = pglobal->m_mapGraph->getLastNode();
		if(plast != NULL)
		{// update its the new pose
			//cout<<"####update node id: "<<plast->m_root_id<<endl;
			pglobal->updateMap((int)plast->m_root_id, (void*)(plast->m_rootPose));			     
		}
		
		buf_node.clear();
		// TODO : display p-Map
		
	}
}

CRunPFGGlobal::CRunPFGGlobal():
m_mapGraph(new CMapGraph("LMS151")),
m_b_flag_backend_quit(false)
{

}

CRunPFGGlobal::~CRunPFGGlobal()
{

}

void CRunPFGGlobal::receMapNode(void* node)
{
	//cout<<"in receMapNode"<<endl;
	static int cnt;
	CMapNode* pcur_Node = static_cast<CMapNode*>(node);
	if(pcur_Node == NULL)
	{
		cout<<"#######back_end will quit!"<<endl;
		m_b_flag_backend_quit = true;
		return ;
	}
	// For debug
	pthread_mutex_lock(&CRunPFGGlobal::s_node_prepare);
		m_buf_node.push_back(pcur_Node);
		//cout<<"rece MapNpde: "<<++cnt<<endl;
	pthread_mutex_unlock(&CRunPFGGlobal::s_node_prepare);
}




