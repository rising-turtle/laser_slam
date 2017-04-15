#ifndef _PIXMAP_H_
#define _PIXMAP_H_

#include <QWidget>
#include <QPixmap>
#include <QImage>
#include <QPainter>
#include <QObject>
#include <vector>

using namespace std;

class CPMap;

class PixmapWidget : public QWidget{
	Q_OBJECT
public:
	PixmapWidget(QWidget *parent=0);
	~PixmapWidget();
	bool map2Image(int, int, int&, int&);
	void calculateLine(int,int,float);
public Q_SLOTS:
	void setZoomFactor(float);
	void setImageMap(QString);
	void updatePMap(CPMap*);
	void receScan(int*,int*,int,int*,float);
	void receParticles(int *, int*, int);
	void receParticle(int,int);
	void receRobot(int,int,float);
	void paintReady();
Q_SIGNALS:
	void zoomFactorChanged(float);
	void sendPointPos(int, int , float);
	void sendPointPos2(int, int , float);
	void sendPoint(int, int);
	// void sendImage(QImage*);
protected:
	void paintEvent(QPaintEvent*);
	void wheelEvent(QWheelEvent*);
	void mousePressEvent(QMouseEvent*);
	void mouseReleaseEvent(QMouseEvent*);
public:
	// QPixmap * m_pm;
	QImage * m_pm;
	QPixmap * m_pS;
	QPainter * m_pt;
	float zoomFactor;
	
	// mouse event point
	int m_p1_x;
	int m_p1_y;
	int m_p2_x;
	int m_p2_y;

	// draw scans
	void drawScan();
	vector<QPoint> m_pts; // scan points
	vector<QPoint> m_pps; // particle samples
	bool m_bDrawRobot; 
	QPoint m_robot; // robot place

	bool m_bDrawRobotLine; 
	QPoint m_pose;
	QPoint m_pose1;
	float m_pth;
};

#endif
