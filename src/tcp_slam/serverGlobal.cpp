#include "serverGlobal.h"
#include "serverSocket.h"
#include "serverBackend.h"
#include "dialog_server.h"
#include "points.h"
#include <iostream>
#include <QtGui>

/*
namespace {
	// 0 RawSeed LMS211 
	// 1 Carmon LMS151 
	// 2 SICK LMS151
	// 3 FUSION LMS151
	#define WORK_MODEL 0
}
*/
using namespace std;

CServerGlobal::CServerGlobal(QObject* parent):
QTcpServer(parent),
m_pThreadBackend(0),
m_pSocket(0),
m_pDialog(new Dialog_Server),
work_model(0),
use_cov(true)
{
	connect(m_pDialog->btnStart,SIGNAL(clicked()),this,SLOT(startServer()));
	connect(m_pDialog->btnStop,SIGNAL(clicked()),this,SLOT(stopServer()));
	connect(m_pDialog->btnQuit,SIGNAL(clicked()),this,SLOT(quitAll()));

	connect(m_pDialog->btnRawseed,SIGNAL(clicked()),this,SLOT(setRawseed()));
	connect(m_pDialog->btnCarmon,SIGNAL(clicked()),this,SLOT(setCarmon()));
	connect(m_pDialog->btnSICK,SIGNAL(clicked()),this,SLOT(setSICK()));
	connect(m_pDialog->btnFusion,SIGNAL(clicked()),this,SLOT(setFusion()));
}
CServerGlobal::~CServerGlobal(){}

void CServerGlobal::setRawseed(){
	work_model = 0;
	printf("set work model %d\n", work_model);
}
void CServerGlobal::setCarmon(){
	work_model = 1;
	printf("set work model %d\n", work_model);
}
void CServerGlobal::setSICK(){
	work_model = 2;
	printf("set work model %d\n", work_model);
}
void CServerGlobal::setFusion(){
	work_model = 3;
	printf("set work model %d\n", work_model);
}

void CServerGlobal::setCov(bool use){
	use_cov = use;
	if(use_cov){
		printf("use COV!");
	}else{
		printf("do not use COV!");
	}
}

void CServerGlobal::startServer(){
	while (isListening() && listen(QHostAddress::Any, 6188)) {
		QMessageBox::StandardButton ret = QMessageBox::critical(m_pDialog,
				tr("Loopback"),
				tr("Unable to start the test: %1.")
				.arg(errorString()),
				QMessageBox::Retry
				| QMessageBox::Cancel);
		if (ret == QMessageBox::Cancel)
			return;
	}
	cout<<" succeed to bind port 6188! "<<endl;
	m_pDialog->labelStatus->setText(tr("Bind port 6188"));
}

void CServerGlobal::setConnections(){
	// connect(this, SIGNAL(readyRead()), this, SLOT(readClient()));
	// connect(this, SIGNAL(disconnected()), this, SLOT(deleteLater()));
	m_pThreadBackend->moveToThread(&threadBackend);
	// start
	connect(&threadBackend,SIGNAL(started()),m_pThreadBackend,SLOT(startBackend()));
	// quit
	connect(m_pThreadBackend,SIGNAL(finished()),&threadBackend,SLOT(quit()),Qt::DirectConnection);
	connect(this, SIGNAL(stopBackend()),m_pThreadBackend,SLOT(stopBackend()),Qt::DirectConnection);
	// tcpSocket -> backend
	connect(m_pSocket,SIGNAL(sendScanFrame(float,float,float,float*,int,int)),m_pThreadBackend,SLOT(receScanFrame(float,float,float,float*,int,int)),Qt::DirectConnection);
	connect(m_pSocket,SIGNAL(sendScanFrameCov(float,float,float,float*,int,int,double*)),m_pThreadBackend,SLOT(receScanFrameCov(float,float,float,float*,int,int,double*)),Qt::DirectConnection);
	// backend -> tcpSocket
	connect(m_pThreadBackend,SIGNAL(sendUpdatePose(int,double,double,double,double*)),m_pSocket,SLOT(receUpdatePose(int,double,double,double,double*)),Qt::DirectConnection);

	// to display single frame
	QObject::connect(m_pThreadBackend,SIGNAL(sendMapInfo(float*,float*,int,double*,double*,double*)),(m_pDialog->m_points_dis),SLOT(receMapInfo(float*,float*,int,double*,double*,double*)),Qt::DirectConnection);
	// to display PMAP in a MapNode
	connect(m_pThreadBackend,SIGNAL(sendPMAP2Dis(float*, float*, int, double,double,double)),(m_pDialog->m_points_dis),SLOT(recePMAP(float*,float*,int,double,double,double)),Qt::DirectConnection);
	connect(m_pThreadBackend,SIGNAL(paintReady()),m_pDialog->m_points_dis,SLOT(paintReady()));
}

void CServerGlobal::incomingConnection(int socketId)
{
	cout<<"reveive new connection: "<<socketId<<endl;
	if(m_pSocket !=0 ){
		cout<<"socket has been used!"<<endl;
		return ;
	}
	
	cout<<" start server socket!!"<<endl;
	m_pDialog->labelStatus->setText("succeed connected!");
	m_pSocket = new CServerSocket(this,use_cov);
	m_pThreadBackend = new CServerBackend(work_model,use_cov);
	m_pSocket->setSocketDescriptor(socketId);
	setConnections();
	threadBackend.start();
	// CServerSocket* socket = new CServerSocket(this);
	// socket->setSocketDescriptor(socketId);
}

void CServerGlobal::stopServer(){
	stopBackend();
	while(threadBackend.isRunning()){
		//cout<<"wait for threadBackend to stop!"<<endl;
		QThread::yieldCurrentThread();
	}
	if(m_pSocket!=0)
	{	
		// delete m_pSocket;
		m_pSocket = 0;
	}
	if(m_pThreadBackend!=0){
		// delete m_pThreadBackend;
		m_pThreadBackend = 0;
	}
	cout<<"after delete these pSocket and pBackend!"<<endl;
}	
void CServerGlobal::quitAll(){
	stopServer();
	finished();
}

