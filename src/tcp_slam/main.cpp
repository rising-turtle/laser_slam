#include <QApplication>

#include "dialog.h"

int main(int argc, char *argv[])
{
	QApplication app(argc, argv);
	Dialog dialog;
#ifdef Q_OS_SYMBIAN
	dialog.showMaximized();
#else   
	dialog.show();
#endif  
	return app.exec();
}

