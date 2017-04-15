#ifndef PMAP_H
#define PMAP_H

// #include <QObject>
// #include <QWidget>
// #include <QGLWidget>

#include <vector>
#include <string>
#include <geometry/point.h>

using namespace std;

typedef struct _ivector2d{
	int x;
	int y; 
	 _ivector2d(int _x,int _y):x(_x),y(_y){}
	 _ivector2d():x(0),y(0){}
	 _ivector2d(const struct _ivector2d& other):x(other.x),y(other.y){}
}ivector2d;

extern void grid_line(ivector2d start, ivector2d end, 
			vector<ivector2d>& line, int& cnt);

class CPMap 
{
public:
	CPMap();
	~CPMap();
public:
	void initializeMap(int size_x, int size_y, int start_x, int start_y,float offset_x, float offset_y, double resolution);
	void uninitMap();
	void updateMap(float* gx, float* gy, int n, float rx, float ry, float th);
public:
	void mapPosfromPoint(Point2D point, ivector2d& pos);
	void computeProbsOfMap();	
	void getPointCloud(vector<float>& , vector<float>&);
public:
	int m_size_x;	// boarder of x in this map
	int m_size_y; 	// boarder of y in this map
	int m_center_x; // center x of the map
	int m_center_y; // center y of the map 
	float m_offset_x; // offset x 
	float m_offset_y; // offset y
	double m_resolution; // resolution of this map
	int m_max_line_lens; // max length of this mapw
	vector< vector<float> > m_maphit; // hit grids
	vector< vector<float> > m_mapsum; // map sum
	vector< vector<float> > m_mapprob; // prob of the grids

private:
	void default_init();
private:
	CPMap(const CPMap&);
	CPMap& operator=(const CPMap&);
};


#endif
