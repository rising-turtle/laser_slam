#include <QApplication>
#include "dialog_client.h"

int main(int argc, char *argv[])
{
	QApplication app(argc, argv);
	Dialog_Client dialog;
	dialog.show();
	return app.exec();
}

