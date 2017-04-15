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
	CServerSocket(QObject * parent = 0);
	~CServerSocket();
	void writeUpdatePose();
Q_SIGNALS:
	void sendScanFrame(float,float,float,float*,int);
private Q_SLOTS:
	void readClient();
	void receUpdatePose(int, double,double,double);
private:
	quint16 nextBlockSize;
public:
	// update pose
	bool bUpdate;
	int pose_id;
	double px;
	double py;
	double pth;
	QMutex m_mutex;
};

#endif
