#include "mainwin.h"

#include <QObject>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QFileDialog>
#include <string>
#include "points.h"
#include "localization.h"

using namespace std;

namespace {
	//string iniFile("/mnt/hgfs/SharedFold/dataset/lenovo/LMS151/work_circles_2.txt");
	string iniFile("/mnt/hgfs/SharedFold/dataset/lenovo/LMS151/20130219.txt");

}

MyWin::~MyWin(){}
MyWin::MyWin()
{
	m_openMap = new QPushButton("openMap");
	m_showMap = new QPushButton("showMap");
	m_readScan = new QPushButton("readScan");
	m_genScan = new QPushButton("genScan");
	m_ranSamps = new QPushButton("ranSamps"); 
	m_global = new QPushButton("globalize");
	m_local = new QPushButton("Loffline");
	m_local2 = new QPushButton("Lonline");
	m_scanMatch = new QPushButton("ScanMatch");
	m_quit = new QPushButton("quit");
	m_sa = new QScrollArea();
	
	// m_pw = new PixmapWidget(QString("test.png"),m_sa);
	m_pw = new PixmapWidget(m_sa);
	m_sa->setWidgetResizable(1);
	m_sa->setWidget(m_pw);
	m_sa->ensureWidgetVisible(m_pw);

	m_sim_scan = new CPoints;
	m_rel_scan = new CPoints;
	/*
	QVBoxLayout* right = new QVBoxLayout;
	right->addWidget(m_sa);
	// right->addStretch();
	right->addWidget(m_cancle);
	
	QHBoxLayout* hori = new QHBoxLayout;
	hori->addWidget(m_left);
	hori->addLayout(right);
	*/

	QHBoxLayout* hori = new QHBoxLayout;
	hori->addWidget(m_openMap);
	hori->addWidget(m_showMap);
	hori->addWidget(m_readScan);
	hori->addWidget(m_genScan);
	hori->addWidget(m_ranSamps);
	hori->addWidget(m_local);
	hori->addWidget(m_local2);
	hori->addWidget(m_global);
	hori->addWidget(m_scanMatch);
	hori->addWidget(m_quit);
	
	QVBoxLayout* ver_up = new QVBoxLayout;
	ver_up->addWidget(m_sim_scan);
	ver_up->addWidget(m_rel_scan);

	QHBoxLayout* hori_up = new QHBoxLayout;
	hori_up->addWidget(m_sa);
	hori_up->addLayout(ver_up);

	QVBoxLayout* verti = new QVBoxLayout;
	// verti->addWidget(m_sa);
	verti->addLayout(hori_up);
	verti->addLayout(hori);

	setLayout(verti);
	resize(1000,700);
	// m_left->resize(width()*0.5,height()*0.5);
	m_sa->resize(width()*0.95, height()*0.85);
	m_sim_scan->setMinimumSize(m_sa->width()*0.3, m_sa->height()*0.5);
	inner_connections();
}

void MyWin::inner_connections()
{
	m_pLocalization = new threadLocalization;
	m_pLocalization->moveToThread(&m_threadLoc);

	connect(m_openMap,SIGNAL(clicked()),this,SLOT(loadFile()));  // load file
	connect(m_readScan,SIGNAL(clicked()),this,SLOT(loadScan())); // load scan
	connect(m_showMap,SIGNAL(clicked()),this,SLOT(generatePMAP())); // generate PMAP
	connect(m_quit,SIGNAL(clicked()),&m_threadLoc,SLOT(quit()),Qt::DirectConnection); // quit thread  
	connect(m_genScan,SIGNAL(clicked()),m_pLocalization,SLOT(randomScan())); // random generate a scan
	connect(m_ranSamps,SIGNAL(clicked()),m_pLocalization,SLOT(ranParticles())); // random generate samples
	connect(m_global,SIGNAL(clicked()),m_pLocalization,SLOT(globalize())); // globalization
	connect(m_local,SIGNAL(clicked()),m_pLocalization,SLOT(localize())); // localization
	connect(m_local2,SIGNAL(clicked()),m_pLocalization,SLOT(localize2())); // localization
	connect(m_scanMatch,SIGNAL(clicked()),m_pLocalization,SLOT(enableSM())); // enable Scan Match

	// generate PMAP through QImage from pixmap -> threadLocalization
	connect(this,SIGNAL(sendQImage(QImage*)),m_pLocalization,SLOT(receivePMAP(QImage*)),Qt::DirectConnection);

	// update PMAP from threadLocalization -> pixmap
	connect(m_pLocalization,SIGNAL(updatePMAP(CPMap*)),m_pw,SLOT(updatePMap(CPMap*)),Qt::DirectConnection);

	// display scan from threadLocalization -> pixmap
	connect(m_pLocalization,SIGNAL(send2Scan(int*,int*,int,int*, float)),m_pw,SLOT(receScan(int*,int*,int,int*,float)),Qt::DirectConnection);
	connect(m_pLocalization,SIGNAL(send2Robot(int,int,float)),m_pw,SLOT(receRobot(int,int,float)),Qt::DirectConnection);
	connect(m_pLocalization,SIGNAL(sendParticles(int*,int*,int)),m_pw,SLOT(receParticles(int*,int*,int)),Qt::DirectConnection);
	connect(m_pLocalization,SIGNAL(sendParticle(int,int)),m_pw,SLOT(receParticle(int,int)),Qt::DirectConnection);

	// debug for point pos 
	connect(m_pw,SIGNAL(sendPointPos(int,int,float)),m_pLocalization,SLOT(localization1(int,int,float)));
	connect(m_pw,SIGNAL(sendPointPos2(int,int,float)),m_pLocalization,SLOT(localization2(int,int,float)));
	connect(m_pw,SIGNAL(sendPoint(int,int)),m_pLocalization,SLOT(simulateSM(int,int)));
	connect(m_pLocalization,SIGNAL(paintReady()),m_pw,SLOT(paintReady()));
	
	// display scan points
	connect(m_pLocalization,SIGNAL(sendSimScan(float*,float*,int,double*,double*,double*)),m_sim_scan,SLOT(receMapInfo(float*,float*,int,double*,double*,double*)),Qt::DirectConnection);
	connect(m_pLocalization,SIGNAL(sendRelScan(float*,float*,int,double*,double*,double*)),m_rel_scan,SLOT(receMapInfo(float*,float*,int,double*,double*,double*)),Qt::DirectConnection);
	connect(m_pLocalization,SIGNAL(paintReady()),m_sim_scan,SLOT(paintReady()));
	connect(m_pLocalization,SIGNAL(paintReady()),m_rel_scan,SLOT(paintReady()));

	// start localization
	m_threadLoc.start();
}

void MyWin::loadFile()
{
	QString filename = QFileDialog::getOpenFileName(this, tr("Open Map"),"/home/lyxp/work/mkproj/SVN/PFG/trunk/src/localization/maps",tr("*.*"));
	m_pw->setImageMap(filename);
}

void MyWin::loadScan()
{
	QString filename = QFileDialog::getOpenFileName(this, tr("Read Scan"),"/mnt/hgfs/SharedFold/dataset/lenovo/LMS151",tr("*.*"));
	string m_scanFile = string(filename.toAscii().constData());
	m_pLocalization->setScanFile(m_scanFile);	
	
	/*if(m_scanFile->is_open())
	{
		cout<<"mainwin.cpp: scan file: "<<m_scanFile<<endl;
	}else{
		cout<<"mainwin.cpp: failed to open file: "<<m_scanFile<<endl;
		delete m_scanFile;
		m_scanFile = 0;
	}*/
}

void MyWin::generatePMAP()
{
	sendQImage(m_pw->m_pm);
}

