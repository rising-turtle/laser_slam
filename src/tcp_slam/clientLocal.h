#ifndef CLIENT_LOCAL_H
#define CLIENT_LOCAL_H
#include <QTcpSocket>
#include <QObject>
#include <QMutex>
#include <QThread>
#include <stdio.h>
#include <vector>
#include <string>

class CClientFrontend;
using namespace std;

class CClientLocal : public QTcpSocket{
	Q_OBJECT
public:
	CClientLocal(QObject * parent=0);
	~CClientLocal();

	void setFile(const char* file);
	void setServerIP(QString, unsigned int);
	void setSICKIP(vector<string> , vector<unsigned int> );
	void setModel(int); // 0 RawSeed 1 Carmon 2 SICK 
Q_SIGNALS:
	void stopFrontend();
	void connectACK();
	void finished();
	void laserFrameReady();
	void laserFrameCovReady();
	void sendUpdatePose(int, void*, double*);
public Q_SLOTS:
	void tryToConnect();
	void startFrontend();
	void quit();
	void quitAll();
	// x,y,th,bearing,num
	void receLaserFrame(double,double,double,float*,int,int);
	void receLaserFrameCov(double,double,double,float*,int,int,double*);
	void sendLaserFrame();
	void sendLaserFrameCov();
	void readServer();
public:
	CClientFrontend* m_pFrontend;
	QThread threadFrontend;
	int work_model;
	string m_file;

	QString server_ip;
	unsigned int server_port;

	vector<string> sick_ip;
	vector<unsigned int> sick_port;

	QMutex m_mutex;
	vector<float> px;
	vector<float> py;
	vector<float> pth;
	vector<int> psyn; // syn_num for pose
	vector< vector<float> > m_bearing;
	vector< vector<double> > pcov;

	double glCov[6];
};


#endif
