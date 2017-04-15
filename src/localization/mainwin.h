#ifndef _MAINWIN_H_
#define _MAINWIN_H_

#include <QMainWindow>
#include <QPushButton>
#include <QWidget>
#include <QScrollArea>
#include <QThread>
#include <QImage>
#include <fstream>
#include "pixmap.h"

class CPoints;
class threadLocalization;

class MyWin : public QWidget//QMainWindow
{
	Q_OBJECT
public:
	MyWin();
	~MyWin();
	void inner_connections();
public Q_SLOTS:
	void loadFile();
	void loadScan();
	void generatePMAP();
Q_SIGNALS:
	void sendQImage(QImage*);
public: 
	QPushButton* m_openMap;
	QPushButton* m_showMap;
	QPushButton* m_readScan;
	QPushButton* m_genScan;
	QPushButton* m_ranSamps;
	QPushButton* m_global;
	QPushButton* m_local;
	QPushButton* m_local2;
	QPushButton* m_scanMatch;
	PixmapWidget* m_pw;
	QPushButton* m_quit;
	QScrollArea* m_sa;
	
	// scan show
	CPoints * m_sim_scan; // to display simulated scan
	CPoints * m_rel_scan; // to display real scan
	// thread Localization 
	threadLocalization * m_pLocalization;
	QThread m_threadLoc;
};

#endif
