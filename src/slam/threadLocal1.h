#ifndef THREAD_LOCAL1_H
#define THREAD_LOCAL1_H

#include <QMutex>
#include <QObject>
#include <string>
#include <vector>
#include <map>

using namespace std;

class CPolarMatch;
class CFliterNode;
class CSICK;
class CObs2DScan;
class OrientedPoint2D;
struct _PMScan;

class ThreadLocal1 : public QObject
{
	Q_OBJECT
public:
	ThreadLocal1();
	~ThreadLocal1();
	void setLogPath(const char*);
	struct _PMScan* constructPSM(vector<float>&, CPolarMatch*);
	bool IsValidMotion(OrientedPoint2D&); 
	bool runFrontEnd(CFliterNode* );
	void translate2GlobalFrame(float*,vector<float>&,vector<float>&,double,double,double);
	bool smallMove(OrientedPoint2D&);
Q_SIGNALS:
	void sendPoseNode(void*); // send PoseNode to next local
	void finished();
	// display current session
	void sendObservation(float*, float*, int, double*,double*,double*);
	void sendUncertainty(float ,float);
	void cleanObservation(double,double);
	void paintReady();

	// send the current robot pose
	void sendCurrentPose(double,double,double);
public Q_SLOTS:
	void runLog(); // run from file
	void runRawSeed(); // run RawSeed file
	void runSick(); // run from sick-lasers
	void synFromGlobal(int, void*);
	void stopThreadLocal1();	
public:
	string m_SName;
	string sick_ip;
	unsigned int sick_port;

	map<int,CFliterNode*> m_graph;
	CPolarMatch* m_pPSM;
	string m_file_name;
	QMutex m_mutex_update;
	QMutex m_mutex_prepare;
	volatile bool m_stop_thread;
};

#endif
