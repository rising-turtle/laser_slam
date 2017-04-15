#include "dialog_server.h"
#include "points.h"
#include <QtGui>
Dialog_Server::Dialog_Server(QWidget* parent):
m_points_dis(new CPoints)
{
	btnStart = new QPushButton("Start");
	btnStop = new QPushButton("Stop");
	btnQuit = new QPushButton("Quit");

	btnRawseed = new QRadioButton("Rawseed");
	btnCarmon = new QRadioButton("Carmon");
	btnSICK = new QRadioButton("SICK");
	btnFusion = new QRadioButton("Fusion");

	QHBoxLayout * line1 = new QHBoxLayout;
	line1->addWidget(btnStart);
	line1->addWidget(btnStop);
	line1->addWidget(btnQuit);
	
	QHBoxLayout * line2 = new QHBoxLayout;
	line2->addWidget(btnRawseed);
	line2->addWidget(btnCarmon);
	line2->addWidget(btnSICK);
	line2->addWidget(btnFusion);

	labelStatus = new QLabel(tr("Server ready"));

	QVBoxLayout * mainLay = new QVBoxLayout;
	mainLay->addLayout(line1);
	mainLay->addLayout(line2);
	mainLay->addWidget(labelStatus);
	mainLay->addWidget(m_points_dis);
	
	setWindowTitle("show Server info!");
	resizeWindowSize(1000,500);
	setLayout(mainLay);
	// set inner connections
	/*connect(btnStart,SIGNAL(clicked()),this,SLOT(startServer()));
	connect(btnStop,SIGNAL(clicked()),this,SLOT(stopServer()));
	connect(btnQuit,SIGNAL(clicked()),this,SLOT(quitAll()));*/
}
Dialog_Server::~Dialog_Server(){}

void Dialog_Server::resizeWindowSize(int width, int height)
{
	if(width<0 || height <0){
		return;
	}
	resize(width,height);
	m_points_dis->setMinimumSize(width,(int) height*0.9);
}



