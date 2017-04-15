#include "pixmap.h"
#include <QWheelEvent>
#include <QPaintEvent>
#include <QPainter>
#include <iostream>
#include <cmath>
#include "PMap.h"

using namespace std;

PixmapWidget::PixmapWidget(QWidget* parent):
m_pm(0),
m_pS(0),
m_bDrawRobot(false),
m_bDrawRobotLine(false)
{
	// m_pm = new QPixmap(filename);
	// m_pm = new QImage(filename);
	m_pt = new QPainter(this);
	zoomFactor = 1.2; 
	// setMinimumSize(m_pm->width()*zoomFactor, m_pm->height()*zoomFactor);
	// setMinimumSize(m_pm->width()*zoomFactor, m_pm->height()*zoomFactor);

}
PixmapWidget::~PixmapWidget(){}

void PixmapWidget::setImageMap(QString filename)
{
	cout<<"pixmap.cpp: filename: "<<filename.toAscii().constData()<<endl;
	m_pm = new QImage(filename);
	if(m_pm->isNull()){
		cout<<"pixmap: failed to set ImageMap"<<endl;
		return ;
	}
	cout<<"pixmap: succeed to set ImageMap: "<<m_pm->width()<<" * "<<m_pm->height()<<endl;
	// repaint();
	return ;
}

void PixmapWidget::setZoomFactor(float factor)
{
	int w,h;
	if(factor == zoomFactor)
		return ;
	zoomFactor = factor;
	emit(zoomFactorChanged(zoomFactor));
	if(!m_pm->isNull()){
		w = m_pm->width()*zoomFactor;
		h = m_pm->height()*zoomFactor;
		setMinimumSize(w,h);
	}
	QWidget *p = dynamic_cast<QWidget*>(parent());
	if(p!=0)
		resize(p->width(),p->height());	
	repaint();
}

// void PixmapWidget::zoomFactorChanged(float factor)

void PixmapWidget::paintEvent(QPaintEvent* event)
{
	int xoffset = 0;
	int yoffset = 0;
	if(m_pm == 0 || m_pm->isNull() || m_pS==0 || m_pS->isNull())
	{
		return ;
	}
	int pixwidth = m_pm->width()*zoomFactor;
	int pixheight = m_pm->height()*zoomFactor;
	bool drawBoarder = false;
	if( width() > pixwidth )
	{
		xoffset = (width()-pixwidth)/2;
		drawBoarder = true;
	}
	if( height() > pixheight)
	{
		yoffset = (height()-pixheight)/2;
		drawBoarder = true;
	}
	
	// draw scans
	if(m_pts.size()>0)
	{
		// drawScan();
		// cout<<"pixmap: begin to draw the scan!"<<endl;
		// draw points
		QPainter ps(m_pS);
		ps.initFrom(this);
		ps.setPen(QPen(Qt::red,3));
		ps.drawPoints(&m_pts[0],m_pts.size());
		// draw robot
		/*ps.setPen(QPen(Qt::blue,5));
		// ps.drawPoint(m_pose);
		// ps.drawEllipse(m_pose,10,10);
		ps.drawLine(m_pose,m_pose1);
		ps.setPen(QPen(Qt::red,10));
		ps.drawPoint(m_pose);*/
		// drawLine 
		m_pts.clear();
	}
	if(m_bDrawRobotLine)
	{
		QPainter ps(m_pS);
		ps.initFrom(this);
		// draw robot
		ps.setPen(QPen(Qt::blue,5));
		ps.drawLine(m_pose,m_pose1);
		ps.setPen(QPen(Qt::red,10));
		ps.drawPoint(m_pose);
		m_bDrawRobotLine = false;
	}
	// draw particles
	if(m_pps.size()>0)
	{
		cout<<"pixmap: begin to draw the particles!"<<endl;
		QPainter ps(m_pS);
		ps.initFrom(this);
		ps.setPen(QPen(Qt::green,5));
		ps.drawPoints(&m_pps[0],m_pps.size());	
		m_pps.clear();
	}
	if(m_bDrawRobot)
	{
		QPainter ps(m_pS);
		ps.initFrom(this);
		ps.setPen(QPen(Qt::red, 12));
		// ps.setBrush(QBrush(Qt::green, Qt::SolidPattern));
		// p.drawEllipse(xoffset, yoffset, pixwidth, pixheight);
		ps.drawPoint(m_robot);
		m_bDrawRobot = false;
	}

	QPainter p(this);
	p.save(); // save current transformation matrix
	p.translate(xoffset,yoffset);
	p.scale(zoomFactor,zoomFactor);
	p.drawPixmap(0,0,*m_pS);
	// p.drawImage(0,0,*m_pm);
	
	p.restore();
	if(drawBoarder)
	{
		p.setPen(Qt::black);
		p.drawRect(xoffset-1,yoffset-1,pixwidth,pixheight);
	}

	// draw a ellipse
	/*p.setRenderHint(QPainter::Antialiasing, true);
	p.setPen(QPen(Qt::black, 12, Qt::DashDotLine, Qt::RoundCap));
	p.setBrush(QBrush(Qt::green, Qt::SolidPattern));
	p.drawEllipse(xoffset, yoffset, pixwidth, pixheight);*/
}

void PixmapWidget::mousePressEvent(QMouseEvent* e)
{
	m_p1_x = e->x();
	m_p1_y = e->y();
}


void PixmapWidget::mouseReleaseEvent(QMouseEvent* e)
{
	const float rad90 = 0.5*M_PI;
	const float rad90_minus = -rad90;
	m_p2_x = e->x();
	m_p2_y = e->y();
	int delta_y = m_p1_y - m_p2_y;
	int delta_x = m_p2_x - m_p1_x;
	float angle; 
	if(delta_x == 0)
	{
		if(delta_y == 0) 
			angle = 0;
		else{
			angle = delta_y > 0? rad90 : rad90_minus;
		}
	} else
	{
		angle = atan2(delta_y, delta_x);
	}

	// cout<<"pixmap.cpp : start: "<<m_p1_x<<" "<<m_p1_y<<" end: "<<m_p2_x<<" "<<m_p2_y<<endl;
	int mx, my;
	if(map2Image(m_p1_x,m_p1_y,mx,my))
	{
		sendPointPos(mx,my,angle);
		sendPointPos2(mx,my,rad90);
		sendPoint(mx,my);
	}else{
		cout<<"pixmap.cpp : invalid point position!"<<endl;
	}
	// cout<<"pixmap.cpp : pose: "<<mx<<" "<<my<<" "<<(angle/M_PI)*180<<endl;
}

bool PixmapWidget::map2Image(int x, int y, int& mx, int& my)
{
	int xoffset = 0;
	int yoffset = 0;
	int pixwidth = m_pm->width()*zoomFactor;
	int pixheight = m_pm->height()*zoomFactor;
	if( width() > pixwidth )
	{
		xoffset = (width()-pixwidth)/2;
	}
	if( height() > pixheight)
	{
		yoffset = (height()-pixheight)/2;
	}
	int px = x - xoffset;
	int py = y - yoffset;
	if( px <=0 || px >= pixwidth || py<=0 || py >= pixheight)
	{
		mx = 0; 
		my = 0;
		return false;
	}
	mx = px/zoomFactor;
	my = py/zoomFactor;
	return true;
}

void PixmapWidget::wheelEvent(QWheelEvent* event)
{
	if(m_pm == 0 || m_pm->isNull())
		return ;
	float f;
	f = zoomFactor + 0.001*event->delta();
	if(f< 32.0/m_pm->width())
		f = 32.0/m_pm->width();
	setZoomFactor(f);
}

void PixmapWidget::updatePMap(CPMap* map)
{
	// set Color
	m_pm = new QImage(map->m_size_x,map->m_size_y,8,256);
	for(int i=0;i<256;i++)
		m_pm->setColor(i,qRgb(i,i,i));

	// reset the final map
	for(int i=0;i<map->m_size_x;i++)
		for(int j=0;j<map->m_size_y;j++){
			m_pm->setPixel(i,j,(int)(255-255*(map->m_mapprob[i][j])));
	}
	m_pS = new QPixmap(QPixmap::fromImage(*m_pm));
	// m_pS->covertFromImage(*m_pm);
	// cout<<"before coverting!"<<endl;
	// m_pS->fromImage(*m_pm);
	// cout<<"after coverting!"<<endl;
	repaint();
}

void PixmapWidget::receParticle(int px, int py)
{
	m_robot.setX(px);
	m_robot.setY(py);
	m_bDrawRobot = true;
}

void PixmapWidget::receParticles(int* px, int* py, int np)
{
	if(m_pps.size()<np){
		m_pps.resize(np);
	}
	int* ptx = px;
	int* pty = py;
	
	// m_robot.setX(*ptx);
	// m_robot.setY(*pty);

	for(int i=0;i<np;i++){
		m_pps[i].setX(*ptx);
		m_pps[i].setY(*pty);
		++ptx;
		++pty;
	}
}

void PixmapWidget::calculateLine(int p1x, int p1y, float th)
{
	float ccos = cosf(th);
	float ssin = sinf(th);
	
	int rx = p1x;
	int ry = p1y;
	static const int m_ori_dis = 20;

	int angle_dir_x = rx + m_ori_dis*ccos ;
	int angle_dir_y = ry - m_ori_dis*ssin;
	m_pose.setX(rx);
	m_pose.setY(ry);
	m_pth = th;
	if(angle_dir_x < m_pm->width() && angle_dir_y > 0)
	{
		m_pose1.setX(angle_dir_x);
		m_pose1.setY(angle_dir_y);
		// cout<<"pixmap.cpp : end_x: "<<angle_dir_x<<" end_y: "<<angle_dir_y<<endl;
	}else{
		m_pose1.setX(rx);
		m_pose1.setY(ry);
	}
	m_bDrawRobotLine = true;
}

void PixmapWidget::receRobot(int px, int py, float th)
{
	m_pose.setX(px);
	m_pose.setY(py);
	calculateLine(px,py,th);
}

void PixmapWidget::receScan(int* px, int* py, int np, int* pose, float th)
{	
	/*if(m_pts.size()<np){
		m_pts.resize(np);
	}*/
	unsigned int shift = m_pts.size();
	m_pts.resize(m_pts.size()+np);
	int* ptx = px;
	int* pty = py;
	for(int i=0;i<np;i++)
	{
		m_pts[shift+i].setX(*ptx);
		m_pts[shift+i].setY(*pty);
		++ptx;
		++pty;
	}
	
	int rx = *pose;
	int ry = *(pose+1);
	receRobot(rx,ry,th);
	/*
	m_pose.setX(rx);
	m_pose.setY(ry);
	// m_pth = th;
	calculateLine(rx,ry,th);
	float ccos = cosf(th);
	float ssin = sinf(th);
	static const int m_ori_dis = 20;
	int angle_dir_x = rx + m_ori_dis*ccos ;
	int angle_dir_y = ry - m_ori_dis*ssin;
	if(angle_dir_x < m_pm->width() && angle_dir_y > 0)
	{
		m_pose1.setX(angle_dir_x);
		m_pose1.setY(angle_dir_y);
		// cout<<"pixmap.cpp : end_x: "<<angle_dir_x<<" end_y: "<<angle_dir_y<<endl;
	}else{
		m_pose1.setX(rx);
		m_pose1.setY(ry);
	}
	*/
	// cout<<"pixmap.cpp: receive pose: "<<rx<<", "<<ry<<", "<<m_pth<<endl;
	// cout<<"pixmap.cpp: with scans: "<<np<<endl;
	// repaint();
}

void PixmapWidget::paintReady()
{
	repaint();
}

void PixmapWidget::drawScan()
{
	if(m_pS == 0 || m_pS->isNull()){
	  m_pS = new QPixmap(m_pm->width(),m_pm->height());
	}
	// draw points
	QPainter p(m_pS);
	p.setPen(QPen(Qt::red));

	p.drawPoints(&m_pts[0],m_pts.size());
	// draw robot
	p.setPen(QPen(Qt::blue,12));
	p.drawPoint(m_pose);
	m_pts.clear();
}


