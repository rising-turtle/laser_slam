
#include <QtGui>
#include <QtNetwork>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
using namespace std;

#include "dialog.h"

Dialog::Dialog(QWidget *parent)
    : QDialog(parent)
{
    clientProgressBar = new QProgressBar;
    clientStatusLabel = new QLabel(tr("Client ready"));
    serverProgressBar = new QProgressBar;
    serverStatusLabel = new QLabel(tr("Server ready"));

    startButton = new QPushButton(tr("&Start"));
    quitButton = new QPushButton(tr("&Quit"));

    buttonBox = new QDialogButtonBox;
    buttonBox->addButton(startButton, QDialogButtonBox::ActionRole);
    buttonBox->addButton(quitButton, QDialogButtonBox::RejectRole);

    connect(startButton, SIGNAL(clicked()), this, SLOT(start()));
    connect(quitButton, SIGNAL(clicked()), this, SLOT(close()));
    
   /* connect(&tcpServer, SIGNAL(newConnection()),
            this, SLOT(acceptConnection()));*/
   /* connect(&tcpClient, SIGNAL(connected()), this, SLOT(startTransfer()));*/
    connect(&tcpClient, SIGNAL(bytesSYN(qint64)),
            this, SLOT(updateClientProgress(qint64)));
    connect(&tcpClient, SIGNAL(error(QAbstractSocket::SocketError)),
            this, SLOT(displayError(QAbstractSocket::SocketError)));
    
    connect(&tcpClient, SIGNAL(bytesACK(qint64)),
    		this, SLOT(updateServerProgress(qint64)));

    QVBoxLayout *mainLayout = new QVBoxLayout;
    mainLayout->addWidget(clientProgressBar);
    mainLayout->addWidget(clientStatusLabel);
    mainLayout->addWidget(serverProgressBar);
    mainLayout->addWidget(serverStatusLabel);
    mainLayout->addStretch(1);
    mainLayout->addSpacing(10);
    mainLayout->addWidget(buttonBox);
    setLayout(mainLayout);

    setWindowTitle(tr("Loopback"));
}

namespace{
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

void Dialog::start()
{
    bytesWritten = 0;
    bytesReceived = 0;
    string file("/mnt/hgfs/SharedFold/dataset/lenovo/lms511_output.txt");
    // TotalBytes = getFileSize(file.c_str());
    TotalBytes = getFileLines(file.c_str());
    cout<<"lines of file: "<<TotalBytes<<endl;
    while (!tcpServer.isListening() && !tcpServer.listen(QHostAddress::Any, 6188)) {
        QMessageBox::StandardButton ret = QMessageBox::critical(this,
                                        tr("Loopback"),
                                        tr("Unable to start the test: %1.")
					.arg(tcpServer.errorString()),
                                        QMessageBox::Retry
					| QMessageBox::Cancel);
        if (ret == QMessageBox::Cancel)
            return;
    }
    
    serverStatusLabel->setText(tr("Listening"));
    clientStatusLabel->setText(tr("Connecting"));
    tcpClient.setFile(file.c_str());
    tcpClient.connectToHost(QHostAddress::LocalHost, /*tcpServer.serverPort()*/ 6188);
}

void Dialog::updateServerProgress(qint64 received)
{
    // bytesReceived += (int)tcpServerConnection->bytesAvailable();
    // tcpServerConnection->readAll();
    bytesReceived += (int)received;
    cout<<"receive: "<<bytesReceived<<" total: "<<TotalBytes<<endl;
    // cout<<"total: "<<TotalBytes<<" bytesReceived: "<<bytesReceived<<" left: "<<TotalBytes-bytesReceived<<endl;
    serverProgressBar->setMaximum(TotalBytes);
    serverProgressBar->setValue(bytesReceived);
    serverStatusLabel->setText(tr("Received %1 LINES")
                               .arg(bytesReceived));

    if (bytesReceived >= TotalBytes) {
        // tcpServerConnection->close();
	tcpClient.close();
	tcpServer.close();
        startButton->setEnabled(true);
	cout<<"FINISH send all datas!"<<endl;
    }
}

void Dialog::updateClientProgress(qint64 numBytes)
{
    // callen when the TCP client has written some bytes
    bytesWritten += (int)numBytes;
    cout<<"send: "<<bytesWritten<<endl;
    // cout<<"total: "<<TotalBytes<<" linesWritten: "<<bytesWritten<<" left: "<<TotalBytes-bytesWritten<<endl;

    // only write more if not finished and when the Qt write buffer is below a certain size.
    /*if (bytesToWrite > 0 && tcpClient.bytesToWrite() <= 4*PayloadSize)
        bytesToWrite -= (int)tcpClient.write(QByteArray(qMin(bytesToWrite, PayloadSize), '@'));
    */
    clientProgressBar->setMaximum(TotalBytes);
    clientProgressBar->setValue(bytesWritten);
    clientStatusLabel->setText(tr("Sent %1 Lines")
                               .arg(bytesWritten));
}

void Dialog::displayError(QAbstractSocket::SocketError socketError)
{
    if (socketError == QTcpSocket::RemoteHostClosedError)
        return;

    QMessageBox::information(this, tr("Network error"),
                             tr("The following error occurred: %1.")
                             .arg(tcpClient.errorString()));

    tcpClient.close();
    tcpServer.close();
    clientProgressBar->reset();
    serverProgressBar->reset();
    clientStatusLabel->setText(tr("Client ready"));
    serverStatusLabel->setText(tr("Server ready"));
    startButton->setEnabled(true);
}
