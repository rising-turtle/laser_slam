#include <QApplication>
#include <QPushButton>
#include <iostream>
#include "serverGlobal.h"
#include "dialog_server.h"

using namespace std;

int main(int argc, char *argv[])
{
	QApplication app(argc, argv);
	CServerGlobal server;
	
	if(!server.listen(QHostAddress::Any, 6188)){
		cout<<" failed to bind to port 6188"<<endl;
		return -1;
	}
	
	/*QPushButton btn(QObject::tr("Quit"));
	QPushButton stopBtn(("Stop"));
	btn.setWindowTitle("Server");
	QObject::connect(&btn,SIGNAL(clicked()),&app,SLOT(quit()));
	QObject::connect(&stopBtn,SIGNAL(clicked()),&server,SLOT(quitAll()));
	btn.show();
	stopBtn.show();*/
	QObject::connect(&server,SIGNAL(finished()),&app,SLOT(quit()));
	server.m_pDialog->show();

	return app.exec();
}

