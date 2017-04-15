#include <QApplication>
#include <QObject>
#include <QGLFormat>
#include <QPushButton>
#include <iostream>
#include "points.h"
#include "map2d.h"
#include "main_window.h"
#include "runPFGLocal.h"
#include "runPFGGlobal.h"

int main(int argc, char* argv[])
{
	QApplication app(argc,argv);
	if(!QGLFormat::hasOpenGL()){
		std::cerr<<"No OpenGL!"<<std::endl;
		return -1;
	}

	CMainWindow ui;
	CRunPFGLocal pfgLocal;
	CRunPFGGlobal pfgGlobal;
/*QObject::connect(&generate,SIGNAL(sendGlbalMapInfo(float*, float*,int, float, float, float)),ui.m_final_map,SLOT(updateMap(float*, float*, int, float, float, float)));
*/
	QObject::connect(&ui,SIGNAL(startLocal(bool, const char*,unsigned int,const char*,unsigned int)),&pfgLocal,SLOT(startLocal(bool,const char*,unsigned int,const char*,unsigned int)));
	QObject::connect(&ui,SIGNAL(startGlobal()),&pfgGlobal,SLOT(startGlobal()));
	QObject::connect(&pfgLocal,SIGNAL(sendMapNode(void*)),&pfgGlobal,SLOT(receMapNode(void*)),Qt::DirectConnection);
	QObject::connect(&pfgGlobal,SIGNAL(updateMap(int,void*)),&pfgLocal,SLOT(synFromGlobal(int,void*)),Qt::DirectConnection);
	/* to display local */
	QObject::connect(&pfgLocal,SIGNAL(sendObservation(float*,float*,int,double*,double*,double*)),ui.m_points_dis,SLOT(receMapInfo(float*,float*,int,double*,double*,double*)));
	QObject::connect(&pfgLocal,SIGNAL(cleanObservation(double,double)),ui.m_points_dis,SLOT(cleanPoints(double,double)),Qt::DirectConnection);
	
	/* to display global map*/
	/*
	QObject::connect(&pfgGlobal,SIGNAL(sendSubmap(void*)),&ui,SLOT(receSubmap(void*)),Qt::DirectConnection);
	QObject::connect(&ui,SIGNAL(update2DPMap(float*,float*,int,float,float,float)),ui.m_final_map,SLOT(updateMap(float*,float*,int,float,float,float)),Qt::DirectConnection);
*/
	QObject::connect(ui.m_btn_quit,SIGNAL(clicked()),&app,SLOT(quit()));
	
	ui.show();
	return app.exec();
}
