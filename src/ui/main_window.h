#ifndef _MAIN_WINDOW_H
#define _MAIN_WINDOW_H

#include <QObject>
#include <QWidget>
#include <QPushButton>
#include <QScrollArea>
#include <QLineEdit>
#include <QLabel>
#include <QString>
#include "points.h"
#include "map2d.h"
#include "gen_points.h"
#include "graphics.h"

// here, opearte obs in MapNode,
// but, do not write feature and pose of MapNode
#include "MapNode.h"

#include <pthread.h>
#include <unistd.h>

class CMainWindow : public QWidget
{
	Q_OBJECT
public:
	CMainWindow(QWidget* parent=NULL);
	~CMainWindow();
	void inner_connections();
	void resizeWindowSize(int width, int height);
public Q_SLOTS:
	void prepareSLAM();
	void receSubmap(void*);
Q_SIGNALS:
	void startLocal(bool blog, const char* str1,unsigned int port1, const char* str2, unsigned int port2);	
	void startGlobal();
	void update2DPMap(float*, float* ,int, float, float, float);
public:
	static void* thread_p_map(void*);
	static pthread_mutex_t s_mutex_pmap_prepare;
	pthread_t thread_pmap_prepare;
	vector<CMapNode*> m_buf_map;
public:
	QPushButton* m_btn_start;
	QPushButton* m_btn_stop;
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
};


#endif
