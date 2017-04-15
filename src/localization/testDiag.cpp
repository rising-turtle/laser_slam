#include "testDiag.h"
#include "testState.h"
#include "ZHPolar_Match.h"
#include "points.h"
#include <QtGui>
#include <QString>
#include <QHBoxLayout>
#include <QVBoxLayout>

namespace{
	QString m_file_init("/mnt/hgfs/SharedFold/dataset/lenovo/LMS151");
}

TestDiag::TestDiag():
m_pPSM(new CPolarMatch("LMS151")),
m_inputFile(0),
m_pointDis(new CPoints)
{
	m_openFile = new QPushButton("Open");
	m_runFile = new QPushButton("Run");
	m_nextScan = new QPushButton("Next");
	m_quit = new QPushButton("Quit");
	QHBoxLayout * hor = new QHBoxLayout;
	hor->addWidget(m_openFile);
	hor->addWidget(m_runFile);
	hor->addWidget(m_nextScan);
	hor->addWidget(m_quit);
	QVBoxLayout * ver = new QVBoxLayout;
	ver->addLayout(hor);
	ver->addWidget(m_pointDis);

	resize(600,500);
	m_pointDis->setMinimumSize(600,500*0.9);
	setLayout(ver);
	// 
	connect(m_openFile,SIGNAL(clicked()),this,SLOT(loadFile()));
	connect(m_runFile,SIGNAL(clicked()),this,SLOT(readFile()));
	connect(m_nextScan,SIGNAL(clicked()),this,SLOT(showScan()));

	connect(this, SIGNAL(send2SLine(float,float,float,float)),m_pointDis,SLOT(receLine(float,float,float,float)),Qt::DirectConnection);
	connect(this, SIGNAL(sendMapInfo(float*,float*,int, double*,double*,double*)), m_pointDis, SLOT(receMapInfo(float*,float*,int, double*, double*, double*)),Qt::DirectConnection);
	connect(this, SIGNAL(paintReady()),m_pointDis,SLOT(paintReady()),Qt::DirectConnection);
}

TestDiag::~TestDiag(){}

void TestDiag::loadFile(){
	QString m_file_name = QFileDialog::getOpenFileName(this,tr("Open Log"),m_file_init,tr("*.*"));
	g_scan_file = string(m_file_name.toAscii().constData());	
	cout<<"testDiag.cpp: openfile: "<<g_scan_file<<endl;
	// editfileText->setText(m_file_name);
}

void TestDiag::readFile(){
	m_inputFile = new ifstream(g_scan_file.c_str());
	if(!m_inputFile->is_open()){
		cout<<"testDiag.cpp: failed to open file: "<<g_scan_file<<endl;
		return;
	}
	showScan();
}


void TestDiag::showScan(int step){
	char line[8192];
	static PMScan ls(541);
	float stop_dis = 1; 
	float slow_dis = 1.5;
	float angle;
	for(int i=0; i < step; i++)
	{
		if(m_inputFile!=0 && !m_inputFile->eof() && m_inputFile->getline(line,8192)){
			if(!constructPSMfromRawSeed(line,ls,m_pPSM))
			{
				cout<<"TestDiag.cpp: error!"<<endl;
			}
		}else{
			cout<<"testDiag.cpp: error, file eof or not open!"<<endl;
			return ;
		}
	}
	// frontObject(ls, stop_dis, slow_dis, angle);
	int status = Look4Window2(ls, angle, 0, 540, slow_dis, stop_dis);
	switch(status){
	case  0:
		cout<<"0 : speed up!"<<endl;
		break;
	case  1:
		cout<<"1: stop and turn angle!"<<endl;
		break;
	case  2:
		cout<<"2: stop and random angle!"<<endl;
		break;
	case  3: 
		cout<<"3: slow and turn angle!"<<endl;
		break;
	case  4:
		cout<<"4: slow!"<<endl;
		break;
	default:
		cout<<"testDiag.cpp: error happened!"<<endl;
		return ;
	}
	send2Display(ls);
	if(status!=2){
		float tangle = angle*(-1.);
		float ex = sin(tangle);
		float ey = cos(tangle);
		send2SLine(0,0,ex,ey);
	}else{
		send2SLine(0,0,0,0);
	}
	paintReady();
}

void TestDiag::send2Display(PMScan& ls){
	static vector<float> fx,fy;
	static double rx,ry,rth;
	rx = 0; ry = 0; rth = 0;
	if(fx.size() < ls.np){
		fx.resize(ls.np);
		fy.resize(ls.np);
	}
	int index = 0;
	for(int i=0;i<ls.np;i++){
		if(ls.bad[i] || ls.r[i] == 0){
			continue;	
		}
		fx[index] = ls.x[i]/100.;
		fy[index] = ls.y[i]/100.;
		index++;
	}
	sendMapInfo(&fx[0],&fy[0],index,&rx,&ry,&rth);
}
