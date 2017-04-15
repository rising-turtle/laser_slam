#ifndef SERVER_BACKEND_H
#define SERVER_BACKEND_H
#include <QObject>
#include <QMutex>
#include <QThread>
#include <vector>

#define DISPLAY 1

using namespace std;
struct _PMScan;
class OrientedPoint2D;
class CPolarMatch;
// class ThreadLocal2;
// class ThreadGlobal1;
class ThreadMapNode;
class ThreadG2OTcp;

class CServerBackend : public QObject
{
	Q_OBJECT
public:
	CServerBackend(int model=0, bool cov = true, QObject* parent=0);
	~CServerBackend();
	void setModel(int);
	void setConnections();
	void sendtoDisplay(struct _PMScan&);
	void constructPMScan(float,float,float,vector<float>&,struct _PMScan&);
	void constructPoseNode();
	bool bigChange(OrientedPoint2D&);
Q_SIGNALS:
	void sendPoseNode(void*);
	void sendMapInfo(float*,float*,int,double*,double*,double*);
	void sendPMAP2Dis(float*,float*,int,double,double,double);
	void sendUpdatePose(int, double,double,double,double*);
	void paintReady();
	void stopThreads();
	void finished();
public Q_SLOTS:
	void startBackend();
	void receScanFrame(float,float,float,float*,int,int);
	void receScanFrameCov(float,float,float,float*,int,int,double*);
	void recePMAP(float*, float*, int, void*);
	void receLoopInfo(int,void*,double*);
	void stopBackend();
public:
	QMutex m_mutex;
	// raw laser scan
	vector<float> px;
	vector<float> py;
	vector<float> pth;
	vector<int> psyn;
	vector<vector<float> > m_bearing;
	vector<vector<double> > pcov;
	// work model
	volatile int work_model;
	volatile bool use_cov;
	// stop variable
	volatile bool m_stop_backend;
	// trajectory
	vector<OrientedPoint2D*> m_traj;	// original trajectory
public:
	QThread threadMapNode;
	QThread threadG2O;
	// ThreadLocal2* m_pThreadLocal2; 
	// ThreadGlobal1* m_pThreadGlobal1; 
	ThreadMapNode* m_pThreadLocal2;
	ThreadG2OTcp* m_pThreadGlobal1;
	CPolarMatch* m_pPSM;
};


#endif
