#ifndef DIALOG_SERVER_H
#define DIALOG_SERVER_H

#include <QWidget>
#include <QPushButton>
#include <QLabel>

class CPoints;

class Dialog_Server : public QWidget
{
	Q_OBJECT
public:
	Dialog_Server(QWidget* parent=0);
	~Dialog_Server();
	void resizeWindowSize(int,int);
public:
	QPushButton* btnStart;
	QPushButton* btnStop;
	QPushButton* btnQuit;
	QLabel* labelStatus;
	CPoints* m_points_dis;
};


#endif
