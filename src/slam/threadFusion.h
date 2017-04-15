/*
 * threadFusion.h
 *
 *  Created on: Dec 18, 2012
 *      Author: liu
 */

#ifndef THREADFUSION_H_
#define THREADFUSION_H_



#include <string>
#include <vector>
#include <map>
#include <iostream>
#include <fstream>
#include <unistd.h>


#include <QObject>
#include <QThread>
#include <QMutex>
#include "qobjectdefs.h"
#include "qmutex.h"


// #include "FlirterNode.h"
// #include "ZHPolar_Match.h"
// #include "MapNode.h"

// sick_reader
#include "CSICK.h"
#include "CObs2DScan.h"
//timestamp
#include "timestamp.h"

// odometry
#include "threadOdo.h"

// sensor fusion

#include "unsFlt.hpp"
#include <boost/numeric/ublas/io.hpp>
#include "config.hpp"


#ifndef M_PI
#	define M_PI 3.14159265358979323846264338327950288		// PI constant
#endif

#ifndef M_2PI
#	define M_2PI 6.283185307179586476925286766559	// The 2*PI constant
#endif

#define D2R M_PI/180.
#define R2D 180./M_PI


using namespace std;
using namespace Bayesian_filter;
using namespace Bayesian_filter_matrix;

class CPolarMatch;
class CCanonicalMatcher;
class CFliterNode;
class CSICK;
class CObs2DScan;
class OrientedPoint2D;
struct _PMScan;

class COdoNode;

class Robot_predict;
class SynGlobal_observe;
class SICK_observe;
class GPS_observe;
class ODO_observe;
class BN_observe;



class ThreadFusion : public QObject
{
	Q_OBJECT
public:
	ThreadFusion();
	~ThreadFusion();
	//void translate2GlobalFrame(float*,vector<float>&,vector<float>&,double,double,double);
	void translate2GlobalFrame(void* param,vector<float>&,vector<float>&);
	void translate2RobotFrame(const OrientedPoint2D* , OrientedPoint2D* ); // translate newPose to the robot frame

	Q_SIGNALS:
	void sendFusedNode(void*);
	void finished();

	//control sensors for synchronizing data
	void pauseMainSICKReader();
	void resumeMainSICKReader();
	void pauseMinorSICKReader();
	void resumeMinorSICKReader();
	void pauseOdoReader();
	void resumeOdoReader();

	void sendTimeMainSICK(TTimeStamp);
	void sendTimeMinorSICK(TTimeStamp);
	void sendTimeOdo(TTimeStamp);

	// display current session
	void sendObservation(float*, float*, int, double*,double*,double*);
	void cleanObservation(double,double);
	void paintReady();

public Q_SLOTS:

void prepareFusedNode_rawseed();
void prepareFusedNode_online();

void recvMainSICKNode(void* );
void recvMinorSICKNode(void* );
void recvOdoNode(void*);
//void recvGPSNode();

void stopThreadFusion();
void synFromGlobal(int, void*);

public:
map<int,CFliterNode*> m_graph;
void cvtOPD2X(const OrientedPoint2D opd, FM::Vec & x);
void cvtX2OPD(const FM::Vec x, OrientedPoint2D& opd);
void pi2pi(float& theta);
void pi2pi(double& theta);

public:

void updateSynGlobalNode();
void updateMainSICKNode();
void updateMinorSICKNode();
void updateOdoNode();

CFliterNode* fusedSICKNode;
CFliterNode* mainSICKNode;
CFliterNode* minorSICKNode;
COdoNode* odoNode;

TTimeStamp t_filter_current;
TTimeStamp t_mainSICKNode_last;
TTimeStamp t_minorSICKNode_last;
TTimeStamp t_odoNode_last;

Unscented_scheme* my_filter; //using unscented filters
OrientedPoint2D* opd_x_p;
OrientedPoint2D* opd_x_u;
OrientedPoint2D* oriInRobot_odo;
OrientedPoint2D* oriInRobot_minorSICK;
OrientedPoint2D* oriInRobot_mainSICK;

OrientedPoint2D* synGlobal_pose;
int synGlobal_pose_ID;

Robot_predict robot_predict;
SynGlobal_observe synGlobal_observe;
SICK_observe sick_observe_main;
SICK_observe sick_observe_minor;
GPS_observe gps_observe;
ODO_observe odo_observe;
BN_observe bn_observe;

QMutex mutex_synGlobal;
QMutex mutex_filter;
QMutex mutex_mainSICK;
QMutex mutex_minorSICK;
QMutex mutex_odo;

volatile bool m_stop_thread;
volatile bool m_pause_thread;

};



#endif /* THREADFUSION_H_ */
