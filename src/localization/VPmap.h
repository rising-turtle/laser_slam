#ifndef VPMAP_H
#define VPMAP_H

#include "PMap.h"
#include <QImage>
#include <vector>

struct _PMScan;
class OrientedPoint2D;
class CICPWarpper;

const float c_min_angle_45 = -0.7853981635;
const float c_min_angle_135 = 3*c_min_angle_45;
const float c_fov_270 = 4.712388981;

class CVPmap : public CPMap
{
public:
	CVPmap();
	~CVPmap();
public:
	bool constructFromImage(QImage* );
	void getMinMax(QImage*, int& pmin, int& pmax);
	bool isValidPose(OrientedPoint2D&);

	void translate2GlobalFrame(struct _PMScan*, vector<float>&, vector<float>&);
	void gen2Display(vector<int>&, vector<int>&, int p[2], struct _PMScan*);
	void laserScanSimulator(struct _PMScan*, OrientedPoint2D&, \ 
				float fov=c_fov_270, \
				float min_angle=c_min_angle_135,\
				float max_range=30);
	void simulateScanRay(double sx, double sy, double angle, \ 
			float& out_range,bool& out_valid, \ 
			int max_ray_len = 15, float threshold_free = 0.65);
	int y2idx(float);
	int x2idx(float);
	float idx2y(unsigned int);
	float idx2x(unsigned int);
	
	float m_max_y_len;
	// Likelihood 
	vector< vector<float> > m_likCache;
public:
	CICPWarpper * m_pICP;
	float obsLikelyhood(struct _PMScan*, OrientedPoint2D& , OrientedPoint2D& );
	float obsLikelyhood2(struct _PMScan*, OrientedPoint2D& );
	float obsLikelyhood3(struct _PMScan*, OrientedPoint2D& );
};


#endif
