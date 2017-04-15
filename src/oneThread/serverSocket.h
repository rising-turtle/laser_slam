#ifndef SERVER_SOCKET_H
#define SERVER_SOCKET_H

#include <QObject>
#include <QTcpSocket>
#include <QThread>
#include <fstream>
#include <QMutex>

class CServerSocket : public QTcpSocket
{
	Q_OBJECT
public:
	CServerSocket(QObject * parent = 0,bool cov = true);
	~CServerSocket();
	void writeUpdatePose();
Q_SIGNALS:
	void sendScanFrame(float,float,float,float*,int,int);
	void sendScanFrameCov(float,float,float,float*,int,int,double*);
private Q_SLOTS:
	void readClient();
	void receUpdatePose(int, double,double,double,double*);
private:
	quint16 nextBlockSize;
public:
	// update pose
	bool bUpdate;
	int pose_id;
	double px;
	double py;
	double pth;
	double pcov[6];
	volatile bool use_cov;
	QMutex m_mutex;
};

#endif
