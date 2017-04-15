#ifndef THREAD_SICK_H
#define THREAD_SICK_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <unistd.h>

#include <QMutexLocker>
#include <QThread>
#include <QMutex>
#include <QObject>

#include "qobjectdefs.h"
#include "timestamp.h"

// sick_reader
#include "CSICK.h"
#include "CObs2DScan.h"


using namespace std;

class CPolarMatch;
class CCanonicalMatcher;
class CFliterNode;
class CSICK;
class CObs2DScan;
class OrientedPoint2D;
struct _PMScan;

class ThreadSICK : public QObject
{
	Q_OBJECT
public:
	ThreadSICK(string name="Main");
	~ThreadSICK();
	void setLogPath(const char*);
	struct _PMScan* constructPSM(vector<float>&, CPolarMatch*);
	bool IsValidMotion(OrientedPoint2D&); 
	bool runFrontEnd(CFliterNode* );
	void translate2RobotFrame(OrientedPoint2D* );
	void translate2GlobalFrame(float*,vector<float>&,vector<float>&,double,double,double);

Q_SIGNALS:
	void sendPoseNode(void*); // send PoseNode to next local
	void finished();
	// display current session
	void sendObservation(float*, float*, int, double*,double*,double*);
	void cleanObservation(double,double);
	void paintReady();
public Q_SLOTS:
	void runLog(); // run from file
	void runRawseed(); // run from file
	void runSick(); // run from sick-lasers
	void synFromGlobal(int, void*);
	void stopThreadSICK();
	void pauseThreadSICKReader(); // for synchronizing data sequence
	void resumeThreadSICKReader();
	void getTimeRecvSICK(TTimeStamp );
public:
	string sick_ip;
	unsigned int sick_port;

	OrientedPoint2D* oriInRobot;
	map<int,CFliterNode*> m_graph;
	CPolarMatch* m_pPSM;
	string m_file_name;
	QMutex m_mutex_update;
	QMutex m_mutex_prepare;
	QMutex m_mutex_time_recv;
	volatile bool m_stop_thread;
	volatile bool m_pause_thread_sick;
	volatile TTimeStamp m_time_recv_sick;
public:
	string m_SName; // descripts the name of this sick
};

#endif
