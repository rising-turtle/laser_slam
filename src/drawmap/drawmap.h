#ifndef _DRAWMAP_H_
#define _DRAWMAP_H_

#include <QObject>
#include <QWidget>
#include <QImage>
#include <QPixmap>
#include <QPainter>

#include <vector>
#include <string>

using namespace std;

class CPolarMatch;
class CPMap;

class CDrawMap : public QObject
{
	Q_OBJECT
public:
	CDrawMap(string laser="LMS151", double cell_size=0.02);
	~CDrawMap();
	void drawLog(string input, string out="map.png");
	void translate2GlobalFrame(vector<float>&, float, float, float);
	bool readLog(CPolarMatch*, string);
	void draw2DMap(CPMap*);
	void drawTrajectory(CPMap*);
public:
	CPolarMatch* m_pLaser;
	vector<vector<float> > m_bearings;
	vector<vector<float> > m_obs_x;
	vector<vector<float> > m_obs_y;
	vector<float> m_px;
	vector<float> m_py;
	vector<float> m_pth;
	vector<bool> m_bIsRoot;
public:
	CPMap * m_pPMAP;
	// map size
	float min_x;
	float max_x;
	float min_y;
	float max_y;
	int m_size_x;
	int m_size_y;
	float m_cell_size;
public:
	QImage *m_pImg;
	QPixmap *m_pPix;
};

#endif
