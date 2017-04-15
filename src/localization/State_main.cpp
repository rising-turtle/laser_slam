#include <QApplication>
#include <QObject>
#include <QMainWindow>
#include <QScrollArea>
#include "testDiag.h"

int main(int argc, char* argv[]){
	
	QApplication app(argc,argv);
	/*QMainWindow* main_win = new QMainWindow();
	QScrollArea* scroll_area = new QScrollArea(main_win);
	PixmapWidget * pw = new PixmapWidget(QString("test.png"),(QWidget*)scroll_area);
	scroll_area->setWidgetResizable(1);
	scroll_area->setWidget(pw);

	main_win->setCentralWidget(scroll_area);
	main_win->show();
	*/
	
	TestDiag* main_win = new TestDiag;
	QObject::connect(main_win->m_quit,SIGNAL(clicked()),&app,SLOT(quit()));
	QObject::connect(&app,SIGNAL(lastWindowClosed()),&app,SLOT(quit()));
	main_win->show();

	return app.exec();
}
