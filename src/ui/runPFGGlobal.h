#ifndef RUN_PFGGLOBAL_H
#define RUN_PFGGLOBAL_H

#include <vector>
#include <string>
#include <map>
#include <QWidget>
#include <QObject>

#include <pthread.h>
#include <unistd.h>

class CMapNode;
class CMapGraph;

using namespace std;

class CRunPFGGlobal : public QWidget
{
	Q_OBJECT
public:
	CRunPFGGlobal();
	~CRunPFGGlobal();
	
public Q_SLOTS:
	void receMapNode(void*); // receive mapNode from Local SLAM
	void startGlobal(); // start Global process
Q_SIGNALS:
	void sendSubmap(void*); // send submap info
	void updateMap(int, void*); // send updated pose of current mapNode
public:
	CMapGraph* m_mapGraph; // add mapNode into mapGraph
	vector<CMapNode*> m_buf_node; // buffer for received map nodes
	bool m_b_flag_backend_quit;
public:
	static pthread_mutex_t s_node_prepare;
	static pthread_t s_thread_prepare;
	static void* s_add_mapNode(void*);
};

#endif
