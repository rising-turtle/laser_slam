#ifndef THREAD_GLOBAL1_H
#define THREAD_GLOBAL1_H

#include <QObject>
#include <QMutex>
#include <vector>
#include <string>
#include <map>

class CMapNode;
class CMapGraph;
class OrientedPoint2D;

using namespace std;

class ThreadGlobal1: public QObject
{
	Q_OBJECT
public:
	ThreadGlobal1();
	~ThreadGlobal1();
	void saveTrajectory(); // record robots's Trajectory 
	bool bigChange(OrientedPoint2D&);
public Q_SLOTS:
	void receMapNode(void*); // receMapNode from Local SLAM
	void addMapNode();	// add MapNode to G2O
	void stopThreadGlobal(); // stop current thread
Q_SIGNALS:
	void updateLocalPose(int,void*);  // updateLocalPose
	void sendSubMap(void*); // display this submap
	void finished(); // finish corresponding thread
public:
	vector<CMapNode*> m_back_nodes;
	CMapGraph* m_pMGraph;
	volatile bool m_stop_thread;
	string m_saved_path;
	QMutex m_mutex;
/*
public:	
	vector<int> m_root_ids; // record root_id for each mapNode
	map<int, OrientedPoint2D*> ori_traj; // original trajectory
	map<int, OrientedPoint2D*> new_traj; // updated trajectory after G2O
	void synNodePose(CMapNode*);
*/
};
#endif
