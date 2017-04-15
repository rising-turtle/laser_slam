#include <QApplication>
#include <QObject>
#include <QGLFormat>
#include <iostream>
#include "ui_win.h"

int main(int argc, char* argv[])
{
	QApplication app(argc,argv);
	if(!QGLFormat::hasOpenGL()){
		std::cerr<<"No OpenGL!"<<std::endl;
		return -1;
	}
	CUIWindow ui;
	QObject::connect(&ui,SIGNAL(stopAll()),&app,SLOT(quit()));
	ui.show();
	return app.exec();
}
