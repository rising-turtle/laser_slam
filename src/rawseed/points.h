#ifndef POINTS_H
#define POINTS_H

#include <QObject>
#include <QGLWidget>
#include <QWidget>
#include <QMutex>
#include <vector>

using namespace std;

class CPoints : public QGLWidget
{
	Q_OBJECT
public:
	CPoints(QWidget* parent=0);
protected:
	void initializeGL();
	void resizeGL(int width, int height);
	void paintGL();
	void computeEllipsold();
public Q_SLOTS:
	void receMapInfo(float* fx, float* fy, int n, double* rx, double* ry, double* th);
	void receUncertainty(float,float);
	void receMapNodeUncertainty(float,float,float,float,float);
	void recePoints(float* px, float* py, float* pz, int n);
	void cleanPoints(double ,double);
	void paintReady();
Q_SIGNALS:
	void synRead();
private:
	void drawEllipsold();
	void drawMap();
	void draw();
	float computeY(float,float,float);
	void computeTransformation(float,float,float,vector<float>&,vector<float>&);
private:
	// map points
	std::vector<float> m_px;
	std::vector<float> m_py;
	std::vector<float> m_pz;
	
	// robot pose 
	std::vector<float> m_rx;
	std::vector<float> m_ry;
	std::vector<float> m_th;
	
	// ellipsold info
	vector<vector<float> > m_ex;
	vector<vector<float> > m_ey;
	vector<float> m_ori_x;
	vector<float> m_ori_y;
	
	vector<bool> m_syn_bits;
	vector<float> m_a;
	vector<float> m_b;
	
	double curr_rx;
	double curr_ry;

	bool m_IsReady;
public:
	QMutex m_mutex;
	QMutex m_mutex2;
};

#endif
