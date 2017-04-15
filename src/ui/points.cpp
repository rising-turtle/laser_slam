#include "points.h"
#include <iostream>

CPoints::CPoints(QWidget* parent):QGLWidget(parent),m_IsReady(false)
{
	setFormat(QGLFormat(QGL::DoubleBuffer | QGL::DepthBuffer));
	curr_rx = curr_ry = 0;
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
	updateGL();
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

void CPoints::drawPoints()
{
	glBegin(GL_POINTS);
	qglColor(Qt::red);
	for(int i=0;i<m_px.size();i++)
		glVertex3f(m_px[i],m_py[i],/*m_pz[i]*/0);
	glEnd();
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
	glBegin(GL_POINTS);
	qglColor(Qt::red);
	for(int i=0;i<m_px.size();i++)
		glVertex2f(m_px[i],m_py[i]);
	glEnd();

	// draw the trajectory
	glBegin(GL_LINE_STRIP);
	qglColor(Qt::yellow);
	for(int i=0;i<m_rx.size();i++)
		glVertex2f(m_rx[i],m_ry[i]);
	glEnd();

	/*glPointSize(2.0);
	qglColor(Qt::blue);
	glBegin(GL_POINTS);
	for(int i=0;i<m_rx.size();i++)
		glVertex2f(m_rx[i],m_ry[i]);
	glEnd();*/
	
	// TODO: glPointSize(5.0);
	qglColor(Qt::green);
	glBegin(GL_POINTS);
		glVertex2f(curr_rx, curr_ry);
	glEnd();
}

void CPoints::draw(){
	if(!m_IsReady)
		return;
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
