#ifndef RUN_PFG_H
#define RUN_PFG_H

#include <algorithm>
#include <numeric>
#include <iostream>
#include <string>
#include <map>
#include <vector>

#include <QWidget>
#include <QObject>
#include "qobjectdefs.h"

using namespace std;

class CPolarMatch;
class CFliterNode;
class CSICK; 
class CObs2DScan;

class OrientedPoint2D;
struct _PMScan;

using namespace std;

// run Frontend of PFG 
class CRunPFGLocal : public QWidget
{
	Q_OBJECT
public:
	CRunPFGLocal();
	bool runFrontEnd(CFliterNode*);
	void runlog(const char* ); // run from file 
	void runSick(vector<string> sick_ip, vector<unsigned int> sick_port); // run from sick-lasers
	void runLogFusion(const char* ); // run from file
	bool IsValidMotion(OrientedPoint2D&);
public:
	struct _PMScan* constructPSM(vector<float>&, CPolarMatch*);
public Q_SLOTS:
	// run different model of SLAM 
	// blog: whether to read log or not
	// if read log, str1 is file name, else, sick_ip
	void startLocal(bool blog, const char* str1,unsigned int port1, const char* str2, unsigned int port2);
	void synFromGlobal(int , void*); // update local pose from global slam 
Q_SIGNALS:
	// to display the current session 
	void sendObservation(float *fx, float *fy, int, double* rx, double* ry, double* th);
	void cleanObservation(double rx, double ry);
	void sendMapNode(void*);
public: // TODO: Adaptive translate function
	void translate2GlobalFrame(float* bearing, vector<float>&fx, vector<float>& fy, double rx, double ry, double th);
	
public:
	CPolarMatch* m_pPSM;
	map<int, CFliterNode*> m_graph;
	vector<CFliterNode*> m_back_nodes;
	bool m_b_flag_frontend_quit; 
public: 
	static pthread_mutex_t s_mutex_prepare;
	static pthread_mutex_t s_mutex_update;
	static pthread_t s_thread_prepare;
	static void* s_prepare_map_node(void*);
	static int s_thre_Tg; // thre for size of submap
	static int s_thre_Tl; 
};

#endif
