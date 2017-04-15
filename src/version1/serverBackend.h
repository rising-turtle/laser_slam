#ifndef SERVER_BACKEND_H
#define SERVER_BACKEND_H
#include <QObject>
#include <QMutex>
#include <QThread>
#include <vector>

#define DISPLAY 1

using namespace std;
struct _PMScan;
namespace zhpsm{
class OrientedPoint2D;
}
class CPolarMatch;
// class ThreadLocal2;
// class ThreadGlobal1;
// class ThreadMapNode;
// class ThreadG2OTcp;

class CServerBackend : public QObject
{
	Q_OBJECT
public:
	CServerBackend(int model=0, QObject* parent=0);
	~CServerBackend();
	void setModel(int);
	// void setConnections();
	void sendtoDisplay(struct _PMScan&);
	void constructPMScan(float,float,float,vector<float>&,struct _PMScan&);
	void constructPoseNode();
	// bool bigChange(OrientedPoint2D&);
Q_SIGNALS:
	// void sendPoseNode(void*);
	void sendMapInfo(float*,float*,int,double*,double*,double*);
	// void sendUpdatePose(int, double,double,double);
	void paintReady();
	void stopThreads();
	void finished();
public Q_SLOTS:
	void startBackend();
	void receScanFrame(float,float,float,float*,int);
	// void receLoopInfo(int,void*);
	void stopBackend();
public:
	QMutex m_mutex;
	// raw laser scan
	vector<float> px;
	vector<float> py;
	vector<float> pth;
	vector<int> psyn;
	vector<vector<float> > m_bearing;
	// work model
	volatile int work_model;
	// stop variable
	volatile bool m_stop_backend;
	// trajectory
	vector<zhpsm::OrientedPoint2D*> m_traj;	// original trajectory
public:
	QThread threadMapNode;
	QThread threadG2O;
	// ThreadLocal2* m_pThreadLocal2; 
	// ThreadGlobal1* m_pThreadGlobal1; 
	// ThreadMapNode* m_pThreadLocal2;
	// ThreadG2OTcp* m_pThreadGlobal1;
	CPolarMatch* m_pPSM;
};


#endif
