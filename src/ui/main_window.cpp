#include "main_window.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QScrollArea>
#include <QString>
// #include <QMessageBox.h>
#include <qmessagebox.h>
#include "imagemap.h"
#include "graphics.h"

namespace{
	QString initFile("/home/lyxp/work/mkproj/SVN/PFG/trunk/data/LMS151/t2/SICK_up.txt");
	QString initIP("0.0.0.0");
}

pthread_mutex_t CMainWindow::s_mutex_pmap_prepare = PTHREAD_MUTEX_INITIALIZER;

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
void* CMainWindow::thread_p_map(void* param)
{
	CMainWindow* pmap = static_cast<CMainWindow*>(param);
	if(pmap == NULL){
		cout<<"error in thread_p_map()!"<<endl;
		return NULL;
	}
	cout<<"drawing thread active!"<<endl;
	vector<CMapNode*> mNodes;
	while(1){
		cout<<"!!!!!!in pmap"<<endl;
		pthread_mutex_lock(&CMainWindow::s_mutex_pmap_prepare);
			if(pmap->m_buf_map.size()<=0){
				pthread_mutex_unlock(&CMainWindow::s_mutex_pmap_prepare);
				sleep(20);
				continue;
			}
			mNodes.swap(pmap->m_buf_map);
		pthread_mutex_unlock(&CMainWindow::s_mutex_pmap_prepare);
		// may conflict with globalSLAM thread, 
		// send update mapinfo
		vector<float> fx;
		vector<float> fy;
		float rx,ry,th;
		for(int i=0;i<mNodes.size();i++){
			CMapNode* pmNode = mNodes[i];
			for(int j=0;j<pmNode->m_bearing.size();j++){
				pmNode->getOneObs(j,fx,fy,rx,ry,th);
				// cout<<"###update2DPMap!"<<endl;
				filterMapinfo(fx,fy);
				pmap->update2DPMap(&fx[0],&fy[0],(int)fx.size(),rx,ry,th);
			}
			// draw this map
			pmap->update2DPMap(NULL,NULL,-1,0,0,0);
		}
		mNodes.clear();
	}
	return NULL;
}

CMainWindow::CMainWindow(QWidget* parent):
m_file_name(initFile),
m_sick1_ip(initIP),
m_sick2_ip(initIP)
{

	setWindowTitle("Show PFG Map");
	// initailize control widgets
	m_btn_start = new QPushButton("start");
	m_btn_stop = new QPushButton("stop");
	m_btn_quit = new QPushButton("quit");
	
	// initailize display widgets
	m_points_dis = new CPoints;
	m_final_map = new CMap2D;
	m_map_painter = new MapPainter;	
	
	// initailize log widgets 
	m_btn_openfile = new QPushButton("open");
	m_edit_filename = new QLineEdit(m_file_name);
	
	// initailize sick widgets
	m_label_sick1 = new QLabel(tr("sick1:"));
	m_label_sick2 = new QLabel(tr("sick2:"));
	m_edit_sick_ip1 = new QLineEdit(m_sick1_ip);
	m_edit_sick_ip2 = new QLineEdit(m_sick2_ip);
	m_edit_sick_port1 = new QLineEdit;
	m_edit_sick_port2 = new QLineEdit;
	
	resizeWindowSize(700,700);
// manage these widgets	
	QHBoxLayout* first_row = new QHBoxLayout;
	first_row->addWidget(m_btn_openfile);
	first_row->addWidget(m_edit_filename);
	first_row->addWidget(m_btn_start);
	
	QHBoxLayout* second_row = new QHBoxLayout;
	second_row->addWidget(m_label_sick1);
	second_row->addWidget(m_edit_sick_ip1);
	second_row->addWidget(m_edit_sick_port1);
	second_row->addWidget(m_btn_stop);

	QHBoxLayout* third_row = new QHBoxLayout;
	third_row->addWidget(m_label_sick2);
	third_row->addWidget(m_edit_sick_ip2);
	third_row->addWidget(m_edit_sick_port2);
	third_row->addWidget(m_btn_quit);
	
	QHBoxLayout* last_row = new QHBoxLayout;
	last_row->addWidget(m_points_dis);
	last_row->addWidget(m_map_painter);

	QVBoxLayout * final = new QVBoxLayout;
	final->addLayout(first_row);
	final->addLayout(second_row);
	final->addLayout(third_row);
	final->addLayout(last_row);

	setLayout(final);
	
	// set inner connections
	inner_connections();
	
}
CMainWindow::~CMainWindow(){}

void CMainWindow::inner_connections(){
	QObject::connect(m_final_map,SIGNAL(sendMap(CMap2D*)),m_map_painter,SLOT(receMap(CMap2D*)),Qt::DirectConnection);
	QObject::connect(m_btn_start,SIGNAL(clicked()),this,SLOT(prepareSLAM()));
}

void CMainWindow::receSubmap(void* param)
{
	CMapNode* pNode = static_cast<CMapNode*>(param);
	if(pNode== NULL){
		cout<<"error in receSubmap!"<<endl;
		return ;
	}
	static int cnt=0;
	std::cout<<"receSubmap: "<<++cnt<<std::endl;
	pthread_mutex_lock(&CMainWindow::s_mutex_pmap_prepare);
		m_buf_map.push_back(pNode);
	pthread_mutex_unlock(&CMainWindow::s_mutex_pmap_prepare);
}
void CMainWindow::prepareSLAM()
{
	// thread for drawing first
	if(pthread_create(&thread_pmap_prepare,NULL,CMainWindow::thread_p_map,(void*)(this))!=0){
		cout<<"failed to create thread draw!"<<endl;
		return ;
	}

	// read contents from widgets
	if(m_edit_filename->isModified())
		m_file_name = m_edit_filename->text();
	if(m_edit_sick_ip1->isModified())
		m_sick1_ip = m_edit_sick_ip1->text();
	if(m_edit_sick_ip2->isModified())
		m_sick2_ip = m_edit_sick_ip2->text();
	if(m_edit_sick_port1->isModified())
		m_sick1_port = (m_edit_sick_port1->text()).toInt();
	if(m_edit_sick_port2->isModified())
		m_sick2_port = (m_edit_sick_port2->text()).toInt();
	

	// whether read log or read sick
	int readlog=-1;
	if(m_sick1_ip != initIP || m_sick2_ip != initIP)	{
		readlog = 0;
	}
	else {
		readlog = 1;
	}
	
	// 
	bool blog; 
	if(readlog < 0)
	{	
		QMessageBox::warning(NULL,"warning","Neither log and sick is selected!",QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
		return ;
	}
	else if(readlog ==0){
		if(m_file_name != initFile){
			QMessageBox::information(NULL,"info","Both log and sick are selected, choose to use sick!",QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
		}
		blog = false;
		startGlobal();
		startLocal(blog,m_sick1_ip.toAscii().constData(),m_sick1_port,m_sick2_ip.toAscii().constData(),m_sick2_port);
		return ;
	}
	else {
		blog = true;
		startGlobal();
		startLocal(blog,m_file_name.toAscii().constData(),0,NULL,0);
	}
	
	return ;
}
void CMainWindow::resizeWindowSize(int width, int height){
	
	if(width< 0 || height < 0)
	{
		return ;
	}
	
	resize(width,height);
	// m_points_dis->resize((int)(width*0.5),int(height*0.85));
	m_points_dis->setMinimumSize((int)(width*0.45),int(height*0.6));
	m_map_painter->resize((int)(width*0.5), int(height*0.5));
}


