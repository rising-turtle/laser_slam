#ifndef THREAD_G2OTCP_H
#define THREAD_G2OTCP_H

#include <QObject>
#include <QMutex>
#include <vector>
#include <string>
#include <map>

class CMapNode;
class CMapGraph;
class OrientedPoint2D;

using namespace std;

class ThreadG2OTcp: public QObject
{
	Q_OBJECT
public:
	ThreadG2OTcp();
	~ThreadG2OTcp();
	void saveTrajectory(); // record robots's Trajectory 
public Q_SLOTS:
	void receMapNode(void*); // receMapNode from Local SLAM
	void addMapNode();	// add MapNode to G2O
	void stopThreadGlobal(); // stop current thread
Q_SIGNALS:
	void updateLocalPose(int,void*,double*);  // updateLocalPose
	void updateSynInfo(int, void*, void*); // update Syn pose
	void sendSubMap(void*); // display this submap
	void finished(); // finish corresponding thread
public:
	vector<CMapNode*> m_back_nodes;
	CMapGraph* m_pMGraph;
	volatile bool m_stop_thread;
	string m_saved_path;
	QMutex m_mutex;
public:	
	vector<int> m_root_ids; // record root_id for each mapNode
	volatile int m_psyn; // number for tcp syn
	map<int, OrientedPoint2D*> ori_traj; // original trajectory
	map<int, OrientedPoint2D*> old_pose; // old pose before G2O
	map<int, OrientedPoint2D*> new_pose; // new pose after G2O 
	void synNodePose(CMapNode*);
	bool bigChange(OrientedPoint2D&);
	void fromRel2AbsPose(OrientedPoint2D&, OrientedPoint2D&, OrientedPoint2D&);
};
#endif
