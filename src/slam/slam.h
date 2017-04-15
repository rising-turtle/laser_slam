#ifndef CSLAM_H
#define CSLAM_H

#include <QObject>
#include <QWidget>
#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <QCoreApplication>
#include <string>

using namespace std;

typedef void (*CallBack_SLAM)(double,double,double);

class ThreadLocal1; // motion estimation
class ThreadLocal2; // construct MapNode
class ThreadGlobal1; // add MapNode to G2O

class CSlam : public QThread
{
	Q_OBJECT
public:
	CSlam();
	~CSlam();
	void InitLog(string, CallBack_SLAM cbslam =NULL);
	void InitSick(string, int , CallBack_SLAM = NULL);
public:
	void run();
public Q_SLOTS:
	void receCurrentPose(double,double,double);
	void stop();
Q_SIGNALS:
	void stopAllThread();
public:
	void setSystem();
public:
	string m_file_name;
public:
	string m_sick1_ip;
	string m_sick2_ip;
	unsigned int m_sick1_port;
	unsigned int m_sick2_port;
public:
	QThread m_threadlocal1;
	QThread m_threadlocal2;
	QThread m_threadglobal1;
	ThreadLocal1* m_pThreadLocal1;
	ThreadLocal2* m_pThreadLocal2;
	ThreadGlobal1* m_pThreadGlobal1;
public:
	CallBack_SLAM m_cbSLAM;
	QMutex m_mutex;
	QWaitCondition m_wait_cond;
	double m_px;
	double m_py; 
	double m_pth;
	volatile bool m_stopped;
	volatile bool m_received;
public:
	void startEventLoop(int& argc, char** argv);
	QCoreApplication* qt_app;
	
private:
	CSlam(const CSlam&);
	CSlam operator=(const CSlam&);
};


#endif
