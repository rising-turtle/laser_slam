#include "serverSocket.h"
#include <QDataStream>
#include <QMutexLocker>
#include <iostream>
#include <fstream>


using namespace std;

CServerSocket::CServerSocket(QObject* parent):
QTcpSocket(parent),
bUpdate(false)
{
	connect(this, SIGNAL(readyRead()), this, SLOT(readClient()));
	connect(this, SIGNAL(disconnected()), this, SLOT(deleteLater()));

	/*m_pThreadBackend->moveToThread(&threadBackend);
	// start
	connect(&threadBackend,SIGNAL(started()),m_pThreadBackend,SLOT(startBackend()));
	// quit
	connect(m_pThreadBackend,SIGNAL(finished()),&threadBackend,SLOT(quit()),Qt::DirectConnection);
	connect(this, SIGNAL(disconnected()),m_pThreadBackend,SLOT(stopBackend()),Qt::DirectConnection);
	connect(&threadBackend,SIGNAL(finished()),this,SLOT(deleteLater()));
	// data 
	connect(this,SIGNAL(sendScanFrame(float,float,float,float*,int)),m_pThreadBackend,SLOT(receScanFrame(float,float,float,float*,int)),Qt::DirectConnection);
	threadBackend.start();
*/
	nextBlockSize = 0;
}
CServerSocket::~CServerSocket(){}

void CServerSocket::receUpdatePose(int id, double x, double y, double th){
	bUpdate = true;
	QMutexLocker locker(&m_mutex);
	pose_id = id;
	px = x;
	py = y;
	pth = th;
}

void CServerSocket::writeUpdatePose(){
	QByteArray block;
	QDataStream out(&block, QIODevice::WriteOnly);
	{
		QMutexLocker locker(&m_mutex);
		out<<pose_id<<px<<py<<pth;
	}
	// cout<<"send update: id: "<<pose_id<<" pose: "<<px<<" "<<py<<" "<<pth<<endl;
	write(block);
	bUpdate = false;
}

void CServerSocket::readClient(){
	static ofstream out_f("server.log");
	QDataStream in(this);
	float x,y,th;
	int syn_num;
	static int cnt=0;
	while(1){
		// always send update first!
		if(bUpdate) 
			writeUpdatePose();
		if(nextBlockSize == 0){
			if(bytesAvailable()<sizeof(quint16))
				return ;
			in >> nextBlockSize;
		}
		if(bytesAvailable()<nextBlockSize) // not enough data
			return ;
		// next is the data that will received!
		// 1 pose: x,y,th, syn_num
		// 2 observation: 541 points
		// float x,y,th;
		// int num;
		// float bearing[];
		
		// cout<<"nextBlockSize: "<<nextBlockSize<<" bytesAvailable: "<<bytesAvailable()<<endl;

		QString timestamp;
		quint16 num_pts=-1;
		in>>x>>y>>th>>num_pts;
		// out_f<<x<<" "<<y<<" "<<th;
		vector<float> bearing(num_pts);
		for(int i=0;i<num_pts;i++){
			in>>bearing[i];
			// out_f<<bearing[i]<<" ";
		}
		// out_f<<endl;
		// cout<<"received "<<++cnt<<" frame: "<<x<<" "<<y<<" "<<th<<" "<<num_pts<<endl;
		// sendThis
		sendScanFrame(x,y,th,&bearing[0],num_pts);

		// send ACK 
		// QDataStream out(this);
		// out<< qint64(1);
		nextBlockSize = 0;
	}
}
