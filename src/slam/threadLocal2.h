#ifndef THREAD_LOCAL2_H
#define THREAD_LOCAL2_H

#include <QObject>
#include <QMutex>
#include <vector>

using namespace std;
class CFliterNode;
class CMapNode;

class ThreadLocal2 : public QObject
{
	Q_OBJECT
public:
	ThreadLocal2();
Q_SIGNALS:
	void sendMapNode(void*);
	void finished();
	void sendMapNodeUncertainty(float,float,float,float,float);
public Q_SLOTS:
	void recePoseNode(void*);
	void prepareMapNode();
	void stopThreadLocal2();
public:
	vector<CFliterNode*> m_back_nodes;
	CMapNode* m_pMapNode;
	QMutex m_mutex;
	volatile bool m_stop_thread;
public:
	static int s_thre_Tl;
	static int s_thre_Tg;
};



#endif
