#include "points.h"
#include <GL/glu.h>
#include <QMutex>
#include <QMutexLocker>
#include <iostream>
#include <cmath>
#include <algorithm>

CPoints::CPoints(QWidget* parent):QGLWidget(parent),m_IsReady(false)
{
	setFormat(QGLFormat(QGL::DoubleBuffer | QGL::DepthBuffer));
}

void CPoints::initializeGL()
{
	qglClearColor(Qt::black);
	glShadeModel(GL_FLAT);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
}

void CPoints::resizeGL(int width, int height)
{
	glViewport(0,0,width,height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	GLfloat x = GLfloat(width)/ height;
	GLfloat y = 40.0; // 50 m 
	// gluOrtho2D(-y,y,-y,y);
	gluOrtho2D(-x*y,x*y,-y,y);
	// glFrustum(-x*y,+x*y,-y,y,4.0,15.0);
	// glMatrixMode(GL_MODELVIEW);
	// glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(0,0,-0.5);
}

void CPoints::paintGL()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	draw();
}

#define SQ(x) ((x)*(x))
// x^2/a^2 + y^2/b^2 = 1
float CPoints::computeY(float x, float a, float b){
	float ret = (SQ(b)*(1-SQ(x)/SQ(a)));
	if(ret < 0 ) return 0;
	return sqrtf(ret);
}

#define MIN_NUM_PTS 21
#define MIN_DIS 0.5

void CPoints::receMapNodeUncertainty(float x, float y, float th,float a, float b){
	vector<float> ex;
	vector<float> ey;
	int n = (int)(2*a/MIN_DIS);
	if(n<MIN_NUM_PTS) n = MIN_NUM_PTS;
	float delta_dis = 2*a/n;
	float min_x = -1.*a;
	float max_x = a;
	float cx,cy;
	for(int i=0;i<=n;i++)
	{
		cx = min_x + i*delta_dis;
		cy = computeY(cx,a,b);
		ex.push_back(cx);
		ey.push_back(cy);
	}
	for(int i=n;i>=0;i--)
	{
		ex.push_back(ex[i]);
		ey.push_back(-1.*ey[i]);
	}
	computeTransformation(x,y,th,ex,ey);

	{
	QMutexLocker locker(&m_mutex);
	m_ex.push_back(ex);
	m_ey.push_back(ey);
	m_ori_x.push_back(x); m_ori_x.push_back(x-b*sinf(th));
	m_ori_y.push_back(y); m_ori_y.push_back(y+b*cosf(th));
	}
	// cout<<"in receMapNodeUncertainty! n: "<<m_ex.size()<<endl;
}

void CPoints::computeEllipsold(){
	vector<float > ex;
	vector<float > ey;
	static int index = 0;
	int n;
	float delta_dis,min_x,max_x,cx,cy;
	float a,b,x,y,th;
	QMutexLocker locker(&m_mutex2);
	for(int i=index;i<m_syn_bits.size();i++){
		if(m_syn_bits[i]) continue;
		a = m_a[i];
		b = m_b[i];
		x = m_rx[i];
		y = m_ry[i];
		th = m_th[i];
		n = (int)(2*a/MIN_DIS);
		if(n<MIN_NUM_PTS) n = MIN_NUM_PTS;
		delta_dis = 2*a/n;
		min_x = -1.*a;
		max_x = a;
		for(int i=0;i<=n;i++){
			cx=min_x + i*delta_dis;
			cy=computeY(cx,a,b);
			ex.push_back(cx);
			ey.push_back(cy);
		}
		for(int i=n;i>=0;i--){
			ex.push_back(ex[i]);
			ey.push_back(-1.*ey[i]);
		}
		// compute coordinates according to pose (x,y,theta)
		computeTransformation(x,y,th,ex,ey);
		// compute orientation of current pose
		m_ori_x.push_back(x); m_ori_x.push_back(x+a*cosf(th));
		m_ori_y.push_back(y); m_ori_y.push_back(y+a*sinf(th));

		m_ex.push_back(ex);
		m_ey.push_back(ey);
		m_syn_bits[i] = true;
	}
}

void CPoints::paintReady(){
	updateGL();
}

void CPoints::receUncertainty(float a,float b)
{
	QMutexLocker locker(&m_mutex);
	m_a.push_back(a);
	m_b.push_back(b);
	m_syn_bits.push_back(false);
}

void CPoints::computeTransformation(float x, float y, float theta, vector<float>& outx, vector<float>& outy){
	vector<float> tx(outx.size());
	vector<float> ty(outy.size());
	if(tx.size()!=ty.size()){
		cout<<"error in computeTransformation()!"<<endl;
		return ;
	}
	float ctheta = cosf(theta);
	float stheta = sinf(theta);
	for(int i=0;i<outx.size();i++){
		tx[i] = x + outx[i]*ctheta - outy[i]*stheta;
		ty[i] = y + outx[i]*stheta + outy[i]*ctheta;
	}
	outx.swap(tx);
	outy.swap(ty);
}

void CPoints::recePoints(float* px, float* py, float* pz, int n)
{	
	m_IsReady = false;
	if(m_px.size()<n)
	{
		m_px.resize(n);
		m_py.resize(n);
		m_pz.resize(n);
	}
	copy(px,px+n,m_px.begin());
	copy(py,py+n,m_py.begin());
	copy(pz,pz+n,m_pz.begin());
	m_IsReady = true;
	// std::cout<<"receive points!"<<std::endl;
	updateGL();
}

void CPoints::receMapInfo(float* fx, float* fy, int n,  double* rx, double* ry, double* th)
{
	m_px.insert(m_px.end(),fx,fx+n);
	m_py.insert(m_py.end(),fy,fy+n);
	/*if(m_px.size()<n){
		m_px.resize(n);
		m_py.resize(n);
	}
	copy(fx,fx+n,m_px.begin());
	copy(fy,fy+n,m_py.begin());
	*/
	m_rx.push_back(*rx);
	m_ry.push_back(*ry);
	m_th.push_back(*th);
	m_IsReady = true;
	
	curr_rx = *rx;
	curr_ry = *ry;

	// std::cout<<"receive MapInfo!"<<std::endl;
	// updateGL();
	// for(int i;i<50000;i++){}
	// synRead();
}

void CPoints::cleanPoints(double rx, double ry)
{
	// std::cout<<"really clean points!"<<std::endl;
	m_px.clear();
	m_py.clear();
	m_pz.clear();

/*	std::vector<float> tx;
	std::vector<float> ty;
	std::vector<float> tz;

	m_px.swap(tx);
	m_py.swap(ty);
	m_pz.swap(tz);*/
	
	/*m_rx.clear();
	m_ry.clear();
	m_th.clear();*/

	curr_rx = rx;
	curr_ry = ry;
	m_IsReady = true;
	/*m_s_rx.push_back(rx);
	m_s_ry.push_back(ry);*/
}
void CPoints::drawEllipsold(){
	// draw the ellipsold
	QMutexLocker locker(&m_mutex2);
	for(int i=0;i<m_ex.size();i++){
		// glLineWidth(2.0);
		glBegin(GL_LINE_LOOP);
		qglColor(Qt::green);
		for(int j=0;j<m_ex[i].size();j++){
			glVertex2f(m_ex[i][j],m_ey[i][j]);
		}
		glEnd();
	}
	for(int i=0;i<m_ori_x.size();i+=2){
		glLineWidth(3.0);
		glBegin(GL_LINES);
		qglColor(Qt::red);
		glVertex2f(m_ori_x[i],m_ori_y[i]);
		glVertex2f(m_ori_x[i+1],m_ori_y[i+1]);
		glEnd();
	}

}
void CPoints::drawMap()
{
	// std::cout<<"draw map!"<<std::endl;
	/*
	for(int i=0;i<m_px.size();i+=10)
	{
		std::cout<<"points at ("<<m_px[i]<<","<<m_py[i]<<")"<<std::endl;
	}*/
	// draw the bearing points
	glPointSize(1.0);
	glBegin(GL_POINTS);
	qglColor(Qt::red);
	for(int i=0;i<m_px.size();i++)
		glVertex2f(m_px[i],m_py[i]);
	glEnd();

	// draw the trajectory
	glLineWidth(1.0);
	glBegin(GL_LINE_STRIP);
	qglColor(Qt::yellow);
	for(int i=0;i<m_rx.size();i++)
		glVertex2f(m_rx[i],m_ry[i]);
	glEnd();

	// draw Uncertainty
	drawEllipsold();

	glPointSize(3.0);
	qglColor(Qt::blue);
	glBegin(GL_POINTS);
	for(int i=0;i<m_rx.size();i++)
		glVertex2f(m_rx[i],m_ry[i]);
	glEnd();
}

void CPoints::draw(){
	if(!m_IsReady)
		return;
	if(m_syn_bits.size()>0 && !m_syn_bits[m_syn_bits.size()-1]){
		computeEllipsold();
	}
	// std::cout<<"drawing!"<<std::endl;
	// drawPoints(); // draw random point
	static int nframe = 0;
	// std::cout<<"this is "<<++nframe<<std::endl;
	drawMap();	
}






/*
void CPoints::draw(){
	static const GLfloat P1[3] = {0.0,-1.0,+2.0};
	static const GLfloat P2[3] = {+1.7,-1.0,-1.0};
	static const GLfloat P3[3] = {-1.7,-1.0,-1.0};
	static const GLfloat* const coords[1][3] ={{P1,P2,P3}};
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(0,0,-10.0);
	
	glBegin(GL_TRIANGLES);
	qglColor(Qt::red);
	for(int i=0;i<3;i++)
		glVertex3f(coords[0][i][0],coords[0][i][1],coords[0][i][2]);
	glEnd();
}*/
