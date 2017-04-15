#ifndef UI_WINDOW_H
#define UI_WINDOW_H

#include <QObject>
#include <QWidget>
#include <QThread>
#include <QPushButton>
#include <QScrollArea>
#include <QLineEdit>
#include <QLabel>
#include <QString>
#include "points.h"
#include "map2d.h"
#include "gen_points.h"
#include "graphics.h"
#include "qobjectdefs.h"


#include <unistd.h>

class ThreadLocal1; // motion estimation
class ThreadLocal2; // construct MapNode
class ThreadGlobal1; // add MapNode to G2O
class ThreadFusion; // add sensor fusion
class ThreadSICK;
class ThreadOdo;

class CUIWindow : public QWidget
{
	Q_OBJECT
public:
	CUIWindow(QWidget* parent=NULL);
	~CUIWindow();
	void inner_connections();
	void setSystem();
	void setSystemFusion();
	void resizeWindowSize(int width, int height);
public Q_SLOTS:
	void loadLogFile(); // read log file
	void runLog();
	void runFusion_rawseed();
	void runFusion_online();
	void runSick();
	void stopApp();
Q_SIGNALS:
	void startLog(const char* );
	void startSick(const char*, unsigned int, const char*, unsigned int);
	void stopAll();
public:
	QPushButton* m_btn_start_log;
	QPushButton* m_btn_start_sick;
	QPushButton* m_btn_start_fusion;
	QPushButton* m_btn_quit;
	CPoints* m_points_dis; 
	CMap2D* m_final_map; 
	MapPainter* m_map_painter;

	QPushButton* m_btn_openfile;
	QLineEdit* m_edit_filename;
	QString m_file_name;

	QLabel* m_label_sick1;
	QLabel* m_label_sick2;
	QLineEdit* m_edit_sick_ip1;
	QLineEdit* m_edit_sick_ip2;
	QLineEdit* m_edit_sick_port1;
	QLineEdit* m_edit_sick_port2;

	QString m_sick1_ip;
	QString m_sick2_ip;
	unsigned int m_sick1_port;
	unsigned int m_sick2_port;
public:
	//For Fusion
	QThread m_threadFusion;
	QThread m_threadMainSick;
	QThread m_threadMinorSick;
	QThread m_threadOdo;

	QThread m_threadlocal1;
	QThread m_threadlocal2;
	QThread m_threadglobal1;

	ThreadFusion* m_pThreadFusion;
	ThreadSICK* m_pThreadMainSICK;
	ThreadSICK* m_pThreadMinorSICK;
	ThreadOdo* m_pThreadOdo;

	ThreadLocal1* m_pThreadLocal1;
	ThreadLocal2* m_pThreadLocal2;
	ThreadGlobal1* m_pThreadGlobal1;
};


#endif
