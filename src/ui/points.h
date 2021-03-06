#ifndef POINTS_H
#define POINTS_H

#include <QObject>
#include <QGLWidget>
#include <QWidget>
#include <vector>
#include <GL/glu.h>

class CPoints : public QGLWidget
{
	Q_OBJECT
public:
	CPoints(QWidget* parent=0);
protected:
	void initializeGL();
	void resizeGL(int width, int height);
	void paintGL();
public Q_SLOTS:
	void receMapInfo(float* fx, float* fy, int n, double* rx, double* ry, double* th);
	void recePoints(float* px, float* py, float* pz, int n);
	void cleanPoints(double rx, double ry );
Q_SIGNALS:
	void synRead();
private:
	void drawPoints();
	void drawMap();
	void draw();
private:
	// map points
	std::vector<float> m_px;
	std::vector<float> m_py;
	std::vector<float> m_pz;
	
	// robot pose 
	std::vector<double> m_rx;
	std::vector<double> m_ry;
	std::vector<double> m_th;
	
	// session start 
	// std::vector<float> m_s_rx;
	// std::vector<float> m_s_ry;
	double curr_rx;
	double curr_ry;

	bool m_IsReady;
};

#endif
