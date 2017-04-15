#include "serverSocket.h"
#include <QDataStream>
#include <QMutexLocker>
#include <iostream>
#include <fstream>


using namespace std;

CServerSocket::CServerSocket(QObject* parent, bool cov):
QTcpSocket(parent),
use_cov(cov),
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

void CServerSocket::receUpdatePose(int id, double x, double y, double th, double* cov){
	bUpdate = true;
	QMutexLocker locker(&m_mutex);
	pose_id = id;
	px = x;
	py = y;
	pth = th;
	memcpy(pcov, cov, 6*sizeof(double));
}

void CServerSocket::writeUpdatePose(){
	QByteArray block;
	QDataStream out(&block, QIODevice::WriteOnly);
	{
		QMutexLocker locker(&m_mutex);
		out<<pose_id<<px<<py<<pth;
		for(int i=0;i<6;i++)
			out<<pcov[i];
	}
	cout<<"send update: id: "<<pose_id<<" pose: "<<px<<" "<<py<<" "<<pth<<" ";
	cout<<"cov: "<<pcov[0]<<" "<<pcov[3]<<" "<<pcov[5]<<endl;
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
		double cov[6];
		if(!use_cov){
			in>>x>>y>>th>>syn_num>>num_pts;
		// out_f<<x<<" "<<y<<" "<<th;
		}else{
			in>>x>>y>>th>>syn_num;
			for(int i=0;i<6;i++)
				in>>cov[i];
			in>>num_pts;
		//	cout<<"received "<<++cnt<<"cov : "<<cov[0]<<" "<<cov[3]<<" "<<cov[5]<<endl;
		}
		vector<float> bearing(num_pts);
		for(int i=0;i<num_pts;i++){
			in>>bearing[i];
			// out_f<<bearing[i]<<" ";
		}
		// out_f<<endl;
		// cout<<"received "<<++cnt<<" frame: "<<x<<" "<<y<<" "<<th<<" "<<num_pts<<" syn: "<<syn_num<<endl;
		if(!use_cov){
			// sendThis
			sendScanFrame(x,y,th,&bearing[0],num_pts,syn_num);
		}else{
			sendScanFrameCov(x,y,th,&bearing[0],num_pts,syn_num,cov);
		}

		// send ACK 
		// QDataStream out(this);
		// out<< qint64(1);
		nextBlockSize = 0;
	}
}
