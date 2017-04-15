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
class CClientBackend;
class LocalV1;
using namespace std;

class CClientLocal : public QTcpSocket{
	Q_OBJECT
public:
	CClientLocal(QObject * parent=0);
	~CClientLocal();
	void setFile(const char* file);
	void setSICKIP(vector<string> , vector<unsigned int> );
	void setModel(int); // 0 RawSeed 1 Carmon 2 SICK 3 fusion
Q_SIGNALS:
	void stopFrontend();
	void stopBackend();
	void stopLocalization();
	void connectACK();
	void finished();
	void laserFrameReady();
	void sendUpdatePose(int, void*);
public Q_SLOTS:
	void tryToConnect();
	void startFrontend();
	void quit();
	void quitAll();
	// x,y,th,bearing,num
	void receLaserFrame(double,double,double,float*,int);
	void sendLaserFrame();
	void readServer();
public:
	CClientFrontend* m_pFrontend;
	CClientBackend* m_pBackend;
	LocalV1* m_pLocalization;
	QThread threadFrontend;
	QThread threadBackend;
	QThread threadLocalization;
	int work_model;
	string m_file;

	vector<string> sick_ip;
	vector<unsigned int> sick_port;

	QMutex m_mutex;
	vector<float> px;
	vector<float> py;
	vector<float> pth;
	// vector<int> psyn; // syn_num for pose
	vector< vector<float> > m_bearing;
};


#endif
