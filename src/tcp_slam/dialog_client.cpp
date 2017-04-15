#include <QtGui>
#include <QtNetwork>
#include <QLineEdit>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
using namespace std;

#include "dialog_client.h"

namespace{
// QString initFile("/mnt/hgfs/SharedFold/dataset/rawseed/Bicocca_2009-02-25b/Bicocca_2009-02-25b-SICK_FRONT.csv");
QString initFile("/mnt/hgfs/SharedFold/dataset/lenovo/LMS151/time_send.log");
// QString initFile("/mnt/hgfs/SharedFold/dataset/slaminput/intel-lab.log");
// QString initFile("/media/ShareRegion/Datasets/LMS151_20130219/t1.txt");

 // QString initFile("/media/ShareRegion/Datasets/LMS151_20130108/t1.txt");


int getFileSize(const char* file){
	FILE* fp = fopen(file,"r");
	if(fp == NULL){
		printf("file: %s not exist!\n",file);
		return -1;
	}
	//* 获取文件大小
	fseek(fp , 0 , SEEK_END);
	int nLen1 = ftell (fp);  //tell the pointer drift number
	fclose(fp);
	return nLen1;
}

int getFileLines(const char* file){
	ifstream inf(file);
	char line[4096];
	int n = 0;
	while(inf.getline(line,4096)){
		n++;
	}
	return n;
}
}

Dialog_Client::Dialog_Client(QWidget *parent)
: QDialog(parent),
  m_file_name(initFile),
 //  m_server_ip("10.100.203.63"),
  m_server_ip("127.0.0.1"),
  m_server_port(6188),
  m_sick1_ip("192.168.1.2"),
  m_sick1_port(2112),
  m_sick2_ip("192.168.1.3"),
  m_sick2_port(2112)
{
	// initailize log widgets       
	openfileBtn = new QPushButton("open");
	editfileText = new QLineEdit(initFile);        
	editfileText->setReadOnly(true);
	QHBoxLayout * line1 = new QHBoxLayout;
	line1->addWidget(openfileBtn);
	line1->addWidget(editfileText);

	m_label_server_ip = new QLabel(tr("server:"));
	m_edit_server_ip = new QLineEdit(m_server_ip);
	m_edit_server_port = new QLineEdit(QString::number(m_server_port));

	QHBoxLayout * line2 = new QHBoxLayout;
	line2->addWidget(m_label_server_ip);
	line2->addWidget(m_edit_server_ip);
	line2->addWidget(m_edit_server_port);

	// initailize sick widgets
	m_label_sick1 = new QLabel(tr("sick1:"));
	m_label_sick2 = new QLabel(tr("sick2:"));
	m_edit_sick_ip1 = new QLineEdit(m_sick1_ip);
	m_edit_sick_ip2 = new QLineEdit(m_sick2_ip);
	m_edit_sick_port1 = new QLineEdit(QString::number(m_sick1_port));
	m_edit_sick_port2 = new QLineEdit(QString::number(m_sick2_port));


	QHBoxLayout* second_row = new QHBoxLayout;
	second_row->addWidget(m_label_sick1);
	second_row->addWidget(m_edit_sick_ip1);
	second_row->addWidget(m_edit_sick_port1);

	QHBoxLayout* third_row = new QHBoxLayout;
	third_row->addWidget(m_label_sick2);
	third_row->addWidget(m_edit_sick_ip2);
	third_row->addWidget(m_edit_sick_port2);

	clientStatusLabel = new QLabel(tr("Client ready"));

	RawSeedBtn = new QPushButton(tr("&RawSeed"));
	CarmonBtn = new QPushButton(tr("&Carmon"));
	SICKBtn = new QPushButton(tr("&SICK"));
	FusionBtn = new QPushButton(tr("&Fusion"));

	connectButton = new QPushButton(tr("&Connect"));
	quitButton = new QPushButton(tr("&Quit"));


	QHBoxLayout * line3 = new QHBoxLayout;
	line3->addWidget(RawSeedBtn);
	line3->addWidget(CarmonBtn);
	line3->addWidget(SICKBtn);
	line3->addWidget(FusionBtn);

	QHBoxLayout * line4 = new QHBoxLayout;
	line4->addWidget(connectButton);
	line4->addWidget(quitButton);

	QVBoxLayout *mainLayout = new QVBoxLayout;
	mainLayout->addLayout(line1);
	mainLayout->addLayout(line2);
	mainLayout->addLayout(second_row);
	mainLayout->addLayout(third_row);
	mainLayout->addWidget(clientStatusLabel);
	// mainLayout->addStretch(1);
	// mainLayout->addSpacing(10);
	mainLayout->addLayout(line3);
	mainLayout->addLayout(line4);
	setLayout(mainLayout);
	setWindowTitle(tr("Client_Diag"));

	inner_connection();
}

void Dialog_Client::inner_connection(){
	connect(openfileBtn,SIGNAL(clicked()),this,SLOT(loadFile()));
	connect(RawSeedBtn,SIGNAL(clicked()),this,SLOT(runRawSeed()));
	connect(CarmonBtn,SIGNAL(clicked()),this,SLOT(runCarmon()));
	connect(SICKBtn,SIGNAL(clicked()),this,SLOT(runSICK()));
	connect(FusionBtn,SIGNAL(clicked()),this,SLOT(runFusion()));

	connect(&tcpClient, SIGNAL(error(QAbstractSocket::SocketError)),
			this, SLOT(displayError(QAbstractSocket::SocketError)));


	connect(connectButton,SIGNAL(clicked()),&tcpClient,SLOT(tryToConnect()),Qt::DirectConnection);
	connect(quitButton,SIGNAL(clicked()),&tcpClient,SLOT(quitAll()),Qt::DirectConnection);
	connect(&clientThread,SIGNAL(finished()),this,SLOT(close()));
	// connect(startButton, SIGNAL(clicked()), this, SLOT(start()));
	// connect(quitButton, SIGNAL(clicked()), this, SLOT(close()));
}

void Dialog_Client::loadFile(){
	m_file_name = QFileDialog::getOpenFileName(this,tr("Open Log"),"/mnt/hgfs/SharedFold/dataset/lenovo/LMS151",tr("*.*"));
	editfileText->setText(m_file_name);
}

void Dialog_Client::runRawSeed(){
	cout<<"in runRawSeed()"<<endl;
	tcpClient.setFile(m_file_name.toAscii().constData());
	tcpClient.setModel(0); // 0 RawSeed
	start();
}

void Dialog_Client::runCarmon(){
	cout<<"in runCarmon()"<<endl;
	tcpClient.setFile(m_file_name.toAscii().constData());
	tcpClient.setModel(1); // 1 Carmon
	start();
}
void Dialog_Client::runSICK(){
	cout<<"Dialog_Client::runSICK(): Running!"<<endl;
	if(m_edit_sick_ip1->isModified()){
		m_sick1_ip = m_edit_sick_ip1->text();
	}

	if(m_edit_sick_port1->isModified()){
		m_sick1_port = (m_edit_sick_port1->text()).toInt();
	}

	v_sick_ips.push_back(string(m_sick1_ip.toAscii().constData()));
	v_sick_ports.push_back(m_sick1_port);

	tcpClient.setSICKIP(v_sick_ips, v_sick_ports);
	tcpClient.setModel(2); // 2 sick
	start();
}

void Dialog_Client::runFusion(){
	// read contents from widgets
	if(m_edit_sick_ip1->isModified()){
		m_sick1_ip = m_edit_sick_ip1->text();
	}
	if(m_edit_sick_port1->isModified()){
		m_sick1_port = (m_edit_sick_port1->text()).toInt();
	}
	if(m_edit_sick_ip2->isModified()){
		m_sick2_ip = m_edit_sick_ip2->text();
	}
	if(m_edit_sick_port2->isModified()){
		m_sick2_port = (m_edit_sick_port2->text()).toInt();
	}

	v_sick_ips.push_back(string(m_sick1_ip.toAscii().constData()));
	v_sick_ips.push_back(string(m_sick2_ip.toAscii().constData()));
	v_sick_ports.push_back(m_sick1_port);
	v_sick_ports.push_back(m_sick2_port);


	cout<<"clientFrontend"<<v_sick_ips.size()<<endl;
	tcpClient.setSICKIP(v_sick_ips, v_sick_ports);
	tcpClient.setModel(3); // 3 fusion
	start();
}

void Dialog_Client::connectACK()
{
	clientStatusLabel->setText(tr("Succeed Connected!"));
}
void Dialog_Client::start()
{
	clientStatusLabel->setText(tr("Connecting"));
	if(m_edit_sick_ip1->isModified()){
		m_server_ip = m_edit_sick_ip1->text();
	}

	if(m_edit_sick_port1->isModified()){
		m_server_port = (m_edit_sick_port1->text()).toInt();
	}
	tcpClient.setServerIP(m_server_ip, m_server_port);

	tcpClient.moveToThread(&clientThread);
	connect(&clientThread,SIGNAL(started()),&tcpClient,SLOT(tryToConnect()));
	connect(&tcpClient,SIGNAL(finished()),&clientThread,SLOT(quit()),Qt::DirectConnection);
	connect(&tcpClient,SIGNAL(connectACK()),this,SLOT(connectACK()));

	clientThread.start();
	// tcpClient.connectToHost(QHostAddress::LocalHost, /*tcpServer.serverPort()*/ 6188);
}
/*
void Dialog_Client::updateClientProgress(qint64 numBytes)
{
    // callen when the TCP client has written some bytes
    bytesWritten += (int)numBytes;
    cout<<"total: "<<TotalBytes<<" linesWritten: "<<bytesWritten<<" left: "<<TotalBytes-bytesWritten<<endl;

    // only write more if not finished and when the Qt write buffer is below a certain size.
    //if (bytesToWrite > 0 && tcpClient.bytesToWrite() <= 4*PayloadSize)
        bytesToWrite -= (int)tcpClient.write(QByteArray(qMin(bytesToWrite, PayloadSize), '@'));

    clientProgressBar->setMaximum(TotalBytes);
    clientProgressBar->setValue(bytesWritten);
    clientStatusLabel->setText(tr("Sent %1 Lines")
                               .arg(bytesWritten));
}
 */
void Dialog_Client::displayError(QAbstractSocket::SocketError socketError)
{
	if (socketError == QTcpSocket::RemoteHostClosedError)
		return;

	QMessageBox::information(this, tr("Network error"),
			tr("The following error occurred: %1.")
			.arg(tcpClient.errorString()));

	tcpClient.close();
	// clientProgressBar->reset();
	// clientStatusLabel->setText(tr("Client ready"));
	// startButton->setEnabled(true);
}
