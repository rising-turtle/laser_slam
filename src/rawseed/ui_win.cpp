#include "ui_win.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QScrollArea>
#include <QString>
#include <QtGui>
// #include <QMessageBox.h>
#include <qmessagebox.h>
// #include "imagemap.h"
// #include "graphics.h"

#include "threadLocal1.h"
#include "threadLocal2.h"
#include "threadGlobal1.h"

#include "threadFusion.h"
#include "threadSICK.h"

namespace{
//QString initFile("/home/lyxp/work/mkproj/SVN/PFG/trunk/data/LMS151/t2/SICK_up.txt");
QString initFile("/mnt/hgfs/SharedFold/dataset/rawseed/Bicocca_2009-02-25b/Bicocca_2009-02-25b-SICK_FRONT.log");
// QString initFile("/media/ShareRegion/Datasets/Lenovo_data_duoLMS151/t2/SICK_up.txt");
// QString initFile("/media/ShareRegion/Datasets/Rawseeds/Bicocca_2009-02-25b/Bicocca_2009-02-25b-SICK_FRONT.csv");
QString initIP("0.0.0.0");
}

namespace{
const float bad_range = -100000;
bool isBadRange(float r){
	return (r==bad_range);
}

void filterMapinfo(vector<float>& fx,vector<float>& fy){
	vector<float>::iterator it;
	it = remove_if(fx.begin(),fx.end(),(bool(*)(float))isBadRange);
	if(it!=fx.end()){
		fx.erase(it,fx.end());
		it = remove_if(fy.begin(),fy.end(),(bool(*)(float))isBadRange);
		fy.erase(it,fy.end());
	}
}
}

CUIWindow::CUIWindow(QWidget* parent):
	m_file_name(initFile),
	m_pThreadFusion(new ThreadFusion),
	m_pThreadMainSICK(new ThreadSICK),
	m_pThreadMinorSICK(new ThreadSICK),
	m_pThreadLocal1(new ThreadLocal1),
	m_pThreadLocal2(new ThreadLocal2),
	m_pThreadGlobal1(new ThreadGlobal1)
{

	setWindowTitle("Show PFG Map");
	// initailize control widgets
	m_btn_start_log = new QPushButton(tr("LOG"));
	m_btn_quit = new QPushButton(tr("QUIT"));
	m_btn_start_fusion = new QPushButton(tr("FUSION"));

	// initailize display widgets
	m_points_dis = new CPoints;
	// m_final_map = new CMap2D;
	// m_map_painter = new MapPainter;	

	// initailize log widgets 
	m_btn_openfile = new QPushButton("open");
	m_edit_filename = new QLineEdit(m_file_name);
	m_edit_filename->setReadOnly(true);

	resizeWindowSize(700,700);
	
	// manage these widgets
	QHBoxLayout* first_row = new QHBoxLayout;
	first_row->addWidget(m_btn_openfile);
	first_row->addWidget(m_edit_filename);

	QHBoxLayout* second_row = new QHBoxLayout;
	second_row->addWidget(m_btn_start_log);
	second_row->addWidget(m_btn_start_fusion);
	second_row->addWidget(m_btn_quit);

	QHBoxLayout* last_row = new QHBoxLayout;
	last_row->addWidget(m_points_dis);
	// last_row->addWidget(m_map_painter);

	QVBoxLayout * final = new QVBoxLayout;
	final->addLayout(first_row);
	final->addLayout(second_row);
	final->addLayout(last_row);

	setLayout(final);

	// set inner connections
	inner_connections();

}
CUIWindow::~CUIWindow(){
	delete m_pThreadFusion;
	delete m_pThreadMainSICK;
	delete m_pThreadMinorSICK;
	delete m_pThreadLocal1;
	delete m_pThreadLocal2;
	delete m_pThreadGlobal1;
}

void CUIWindow::inner_connections(){
	// QObject::connect(m_final_map,SIGNAL(sendMap(CMap2D*)),m_map_painter,SLOT(receMap(CMap2D*)),Qt::DirectConnection);
	QObject::connect(m_btn_start_fusion,SIGNAL(clicked()),this,SLOT(runFusion()));
	QObject::connect(m_btn_start_log,SIGNAL(clicked()),this,SLOT(runLog()));
	// QObject::connect(m_btn_start_sick,SIGNAL(clicked()),this,SLOT(runSick()));
	QObject::connect(m_btn_openfile,SIGNAL(clicked()),this,SLOT(loadLogFile()));
}

// these settings are important
void CUIWindow::setSystemFusion()
{
	m_pThreadFusion->moveToThread(&m_threadFusion);
	m_pThreadLocal2->moveToThread(&m_threadlocal2);
	m_pThreadGlobal1->moveToThread(&m_threadglobal1);

	QObject::connect(&m_threadFusion,SIGNAL(started()),m_pThreadFusion,SLOT(prepareFusedNode()));
	QObject::connect(&m_threadlocal2,SIGNAL(started()),m_pThreadLocal2,SLOT(prepareMapNode()));
	QObject::connect(&m_threadglobal1,SIGNAL(started()),m_pThreadGlobal1,SLOT(addMapNode()));

	// quit process
	QObject::connect(m_pThreadMainSICK,SIGNAL(finished()),&m_threadMainSick,SLOT(quit()),Qt::DirectConnection);
	QObject::connect(m_pThreadMinorSICK,SIGNAL(finished()),&m_threadMinorSick,SLOT(quit()),Qt::DirectConnection);
	QObject::connect(m_pThreadFusion,SIGNAL(finished()),&m_threadFusion,SLOT(quit()),Qt::DirectConnection);
	QObject::connect(m_pThreadLocal2,SIGNAL(finished()),&m_threadlocal2,SLOT(quit()),Qt::DirectConnection);
	QObject::connect(m_pThreadGlobal1,SIGNAL(finished()),&m_threadglobal1,SLOT(quit()),Qt::DirectConnection);

	// send btn_quit -> thread1 -> thread2 (send last MapNode) -> thread3 (record trajectory)
	QObject::connect(m_btn_quit,SIGNAL(clicked()),m_pThreadMainSICK,SLOT(stopThreadSICK()),Qt::DirectConnection);
	QObject::connect(m_btn_quit,SIGNAL(clicked()),m_pThreadMinorSICK,SLOT(stopThreadSICK()),Qt::DirectConnection);
	QObject::connect(&m_threadMainSick,SIGNAL(finished()),m_pThreadFusion,SLOT(stopThreadFusion()),Qt::DirectConnection);
	QObject::connect(&m_threadFusion,SIGNAL(finished()),m_pThreadLocal2,SLOT(stopThreadLocal2()),Qt::DirectConnection);
	QObject::connect(&m_threadlocal2,SIGNAL(finished()),m_pThreadGlobal1,SLOT(stopThreadGlobal()),Qt::DirectConnection);
	QObject::connect(&m_threadglobal1,SIGNAL(finished()),this,SLOT(stopApp()));

	// MainSICK node -> thread fusion
	QObject::connect(m_pThreadMainSICK,SIGNAL(sendPoseNode(void*)),m_pThreadFusion,SLOT(recvMainSICKNode(void*)),Qt::DirectConnection);
	// MinorSICK node -> thread fusion
	QObject::connect(m_pThreadMinorSICK,SIGNAL(sendPoseNode(void*)),m_pThreadFusion,SLOT(recvMinorSICKNode(void*)),Qt::DirectConnection);
	// thread fusion's poseNode -> thread2
	QObject::connect(m_pThreadFusion,SIGNAL(sendFusedNode(void*)),m_pThreadLocal2,SLOT(recePoseNode(void*)),Qt::DirectConnection);
	// thread2's mapNode -> thread3
	QObject::connect(m_pThreadLocal2,SIGNAL(sendMapNode(void*)),m_pThreadGlobal1,SLOT(receMapNode(void*)),Qt::DirectConnection);
	// thread3's update -> thread1
	QObject::connect(m_pThreadGlobal1,SIGNAL(updateLocalPose(int,void*)),m_pThreadFusion,SLOT(synFromGlobal(int,void*)),Qt::DirectConnection);

	// display local

	QObject::connect(m_pThreadFusion,SIGNAL(sendObservation(float*,float*,int,double*,double*,double*)),m_points_dis,SLOT(receMapInfo(float*,float*,int,double*,double*,double*)),Qt::DirectConnection);
	QObject::connect(m_pThreadFusion,SIGNAL(paintReady()),m_points_dis,SLOT(paintReady()));
	QObject::connect(m_pThreadFusion,SIGNAL(cleanObservation(double,double)),m_points_dis,SLOT(cleanPoints(double,double)));

	// disable buttons
	m_btn_start_log->setEnabled(false);
	// m_btn_start_sick->setEnabled(false);
	m_btn_start_fusion->setEnabled(false);

}

// these settings are important
void CUIWindow::setSystem()
{
	m_pThreadLocal2->moveToThread(&m_threadlocal2);
	m_pThreadGlobal1->moveToThread(&m_threadglobal1);

	QObject::connect(&m_threadlocal2,SIGNAL(started()),m_pThreadLocal2,SLOT(prepareMapNode()));
	QObject::connect(&m_threadglobal1,SIGNAL(started()),m_pThreadGlobal1,SLOT(addMapNode()));

	// quit process
	QObject::connect(m_pThreadLocal1,SIGNAL(finished()),&m_threadlocal1,SLOT(quit()),Qt::DirectConnection);
	QObject::connect(m_pThreadLocal2,SIGNAL(finished()),&m_threadlocal2,SLOT(quit()),Qt::DirectConnection);
	QObject::connect(m_pThreadGlobal1,SIGNAL(finished()),&m_threadglobal1,SLOT(quit()),Qt::DirectConnection);

	// send btn_quit -> thread1 -> thread2 (send last MapNode) -> thread3 (record trajectory)
	QObject::connect(m_btn_quit,SIGNAL(clicked()),m_pThreadLocal1,SLOT(stopThreadLocal1()),Qt::DirectConnection);
	QObject::connect(&m_threadlocal1,SIGNAL(finished()),m_pThreadLocal2,SLOT(stopThreadLocal2()),Qt::DirectConnection);

	QObject::connect(&m_threadlocal2,SIGNAL(finished()),m_pThreadGlobal1,SLOT(stopThreadGlobal()),Qt::DirectConnection);
	QObject::connect(&m_threadglobal1,SIGNAL(finished()),this,SLOT(stopApp()));


	// thread1's poseNode -> thread2
	QObject::connect(m_pThreadLocal1,SIGNAL(sendPoseNode(void*)),m_pThreadLocal2,SLOT(recePoseNode(void*)),Qt::DirectConnection);
	// thread2's mapNode -> thread3 
	QObject::connect(m_pThreadLocal2,SIGNAL(sendMapNode(void*)),m_pThreadGlobal1,SLOT(receMapNode(void*)),Qt::DirectConnection);
	// thread3's update -> thread1
	QObject::connect(m_pThreadGlobal1,SIGNAL(updateLocalPose(int,void*)),m_pThreadLocal1,SLOT(synFromGlobal(int,void*)),Qt::DirectConnection);

	// display local

	QObject::connect(m_pThreadLocal1,SIGNAL(sendObservation(float*,float*,int,double*,double*,double*)),m_points_dis,SLOT(receMapInfo(float*,float*,int,double*,double*,double*)),Qt::DirectConnection);
	QObject::connect(m_pThreadLocal2,SIGNAL(sendMapNodeUncertainty(float,float,float,float,float)),m_points_dis,SLOT(receMapNodeUncertainty(float,float,float,float,float)),Qt::DirectConnection);
	/*QObject::connect(m_pThreadLocal1,SIGNAL(sendUncertainty(float,float)),m_points_dis,SLOT(receUncertainty(float,float)),Qt::DirectConnection);*/
	QObject::connect(m_pThreadLocal1,SIGNAL(paintReady()),m_points_dis,SLOT(paintReady()));
	QObject::connect(m_pThreadLocal1,SIGNAL(cleanObservation(double,double)),m_points_dis,SLOT(cleanPoints(double,double)));

	// disable buttons
	m_btn_start_log->setEnabled(false);
	m_btn_start_fusion->setEnabled(false);
	//m_btn_start_sick->setEnabled(false);
}

void CUIWindow::loadLogFile(){
	m_file_name = QFileDialog::getOpenFileName(this,tr("Open Log"),
			m_file_name,tr("*.*"));
	m_edit_filename->setText(m_file_name);
}

void CUIWindow::runLog()
{
	setSystem();
	// read contents from widgets
	/*if(m_edit_filename->isModified())
		m_file_name = m_edit_filename->text();*/
	m_pThreadLocal1->setLogPath(m_file_name.toAscii().constData());
	m_pThreadLocal1->moveToThread(&m_threadlocal1);

	QObject::connect(&m_threadlocal1,SIGNAL(started()),m_pThreadLocal1,SLOT(runRawSeed()));

	m_threadglobal1.start();
	m_threadlocal2.start();
	m_threadlocal1.start();
}

void CUIWindow::runFusion(){
	setSystemFusion();

	/*user predefined file names*/
	/*
	char mainSICKName[] = "/media/ShareRegion/Datasets/Lenovo_data_duoLMS151/t2/SICK_up_1.txt";
	char minorSICKName[] = "/media/ShareRegion/Datasets/Lenovo_data_duoLMS151/t2/SICK_up_2.txt";
	*/
	char mainSICKName[] = "/mnt/hgfs/SharedFold/dataset/rawseed/Bicocca_2009-02-25b/Bicocca_2009-02-25b-SICK_FRONT.csv";
	char minorSICKName[] = "/mnt/hgfs/SharedFold/dataset/rawseed/Bicocca_2009-02-25b/Bicocca_2009-02-25b-SICK_REAR.csv";

	// char mainSICKName[] = "/media/ShareRegion/Datasets/Rawseeds/Bicocca_2009-02-25b/Bicocca_2009-02-25b-SICK_FRONT.csv";
	// char minorSICKName[] = "/media/ShareRegion/Datasets/Rawseeds/Bicocca_2009-02-25b/Bicocca_2009-02-25b-SICK_REAR.csv";


	//char mainSICKName[] = "/home/lyxp/work/mkproj/SVN/PFG/trunk/data/LMS151/t2/SICK_up.txt";
	//char minorSICKName[] = "/home/lyxp/work/mkproj/SVN/PFG/trunk/data/LMS151/t2/SICK_down.txt";

	m_pThreadMainSICK->setLogPath(mainSICKName);
	m_pThreadMainSICK->moveToThread(&m_threadMainSick);
	m_pThreadMinorSICK->setLogPath(minorSICKName);
	m_pThreadMinorSICK->moveToThread(&m_threadMinorSick);

	/*Use Manually Chosen*/
	/*
	m_pThreadMainSICK->setLogPath(m_file_name.toAscii().constData());
	m_pThreadMainSICK->moveToThread(&m_threadMainSick);

	m_pThreadMinorSICK->setLogPath(m_file_name.toAscii().constData());
	m_pThreadMinorSICK->moveToThread(&m_threadMinorSick);
	*/


	//QObject::connect(&m_threadMainSick,SIGNAL(started()),m_pThreadMainSICK,SLOT(runLog()));
	//QObject::connect(&m_threadMinorSick,SIGNAL(started()),m_pThreadMinorSICK,SLOT(runLog()));

	//For Rawseed
	QObject::connect(&m_threadMainSick,SIGNAL(started()),m_pThreadMainSICK,SLOT(runRawseed()));
	QObject::connect(&m_threadMinorSick,SIGNAL(started()),m_pThreadMinorSICK,SLOT(runRawseed()));



	m_threadglobal1.start();
	m_threadlocal2.start();
	m_threadFusion.start();
	m_threadMainSick.start();
	//m_threadMinorSick.start();
	return ;
}

void CUIWindow::runSick(){
	setSystem();
	cout<<"not finished yet!"<<endl;
	return ;
}

void CUIWindow::resizeWindowSize(int width, int height){

	if(width< 0 || height < 0)
	{
		return ;
	}

	resize(width,height);
	// m_points_dis->resize((int)(width*0.5),int(height*0.85));
	// m_points_dis->setMinimumSize((int)(width*0.5),int(height*0.6));
	// m_map_painter->resize((int)(width*0.45), int(height*0.5));
}

void CUIWindow::stopApp(){
	cout<<"thread: "<<(int)QThread::currentThreadId()<<" stop MainThread!"<<endl;
	stopAll();
}

