#ifndef THREAD_MAPNODE_H
#define THREAD_MAPNODE_H

#include <QObject>
#include <QMutex>
#include <vector>
#include <map>

using namespace std;
class CFliterNode;
class CMapNode;
class OrientedPoint2D;

class ThreadMapNode : public QObject
{
	Q_OBJECT
public:
	ThreadMapNode();
Q_SIGNALS:
	void sendMapNode(void*);
	void sendPMAP(float*,float*,int,void*);
	void finished();
	void sendMapNodeUncertainty(float,float,float,float,float);
public Q_SLOTS:
	void recePoseNode(void*);
	void receSynInfo(int, void*, void*);
	void prepareMapNode();
	void stopThreadMapNode();
public:
	void synPoseTcp(CMapNode* , int);

	map<int, OrientedPoint2D* > m_synOldP;
	map<int, OrientedPoint2D* > m_synNewP;

public:
	vector<CFliterNode*> m_back_nodes;
	CMapNode* m_pMapNode;
	QMutex m_mutex;
	volatile bool m_stop_thread;
	bool m_bDisplay;

public:
	static int s_thre_Tl;
	static int s_thre_Tg;
	static int s_thre_Tn;
};



#endif
