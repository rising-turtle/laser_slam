#include "VPmap.h"
#include "globaldef.h"
#include <iostream>
#include <QColor>
#include "PolarParameter.h"
#include "point.h"
#include "ZHIcp_Warpper.h"
#include <fstream>
#include <sstream>

using namespace std;

CVPmap::CVPmap():
m_pICP(new CICPWarpper)
{}
CVPmap::~CVPmap(){
	if(m_pICP) delete m_pICP;
}

// static const float pixResolution = 0.0194; // 19.4 mm
// static const float g_free_threshold = 0.3; // threshold for free cell

bool CVPmap::constructFromImage(QImage* pm)
{
	if(pm ==0 || pm->isNull())
	{
		cout<<"CVPmap: QImage pm is NULL!"<<endl;
		return false;
	}
	initializeMap(pm->width(),pm->height(),0,0,0,0,g_pixResolution);
	cout<<"VPmap.cpp: map has: "<<pm->width()*g_pixResolution<<" * "<<\
	pm->height()*g_pixResolution<<endl;

	// height of the PMap along y-axis
	m_max_y_len = pm->height()*g_pixResolution;

	// assign hit probability
	int pmin, pmax;
	getMinMax(pm,pmin,pmax);
	cout<<"VPmap.cpp: image has gray [ "<<pmin<<"-"<<pmax<<" ]"<<endl;
	int range = pmax - pmin;

	for(int i=0;i<pm->width();i++)
	for(int j=0;j<pm->height();j++)
	{
		// gray = (R*11 + G*16 + B*5)/32
		// qRgb(200,200,255) -> 208.5 is unknown
		int gray  = qGray(pm->pixel(i,j));
		if(gray >= 208 && gray <= 209) 
		{
			m_mapprob[i][j] = g_unknown_value; // unknown place
		}else
		{
			m_mapprob[i][j] = (float)(pmax-gray)/(float)range;
			m_mapsum[i][j]++;
		}
	}
	return true;
}

bool CVPmap::isValidPose(OrientedPoint2D& pose)
{
	Point2D& p = pose;
	ivector2d pos;
	mapPosfromPoint(p,pos);
	if(pos.x > m_size_x || pos.y > m_size_y)
	{
		cout<<"VPmap: pos: "<<pos.x<<", "<<pos.y<<endl;
		cout<<"VPmap: max: "<<m_size_x<<", "<<m_size_y<<endl;
		return false;
	}
	if(m_mapprob[pos.x][pos.y] > g_free_threshold)
	{
		return false;
	}
	if(fabs(m_mapprob[pos.x][pos.y] - g_unknown_value)< 0.01)
	{
		return false;
	}
	return true;
}

void CVPmap::getMinMax(QImage* pm, int& pmin, int& pmax){
	pmin = 256; pmax = -1;
	for(int i=0;i<pm->width();i++)
	for(int j=0;j<pm->height();j++)
	{
		int qray = qGray(pm->pixel(i,j));
		if(qray<pmin) pmin = qray;
		if(qray>pmax) pmax = qray;
	}
}

int CVPmap::y2idx(float ry){
	return (int)(m_center_y + (ry-m_offset_y + 0.0005)/(m_resolution));
}

int CVPmap::x2idx(float rx){
	return (int)(m_center_x + (rx-m_offset_x + 0.0005)/(m_resolution));
}

float CVPmap::idx2x(unsigned int x){
	return m_offset_x + (x-m_center_x)*m_resolution;
}
float CVPmap::idx2y(unsigned int y){
	return (m_offset_y + (y-m_center_y)*m_resolution);
}
void CVPmap::translate2GlobalFrame(PMScan* scan, vector<float>& ox, vector<float>& oy)
{
	static bool initAngle = false;
	static vector<float> An(scan->np);
	static vector<float> cosA(scan->np);
	static vector<float> sinA(scan->np);
	if(!initAngle)
	{
		initAngle = true;
		static float minA = (-M_PI)/4.0;
		static float dfi = (float)((M_PI)/360.0);
		for(int i=0;i<scan->np;i++)
		{
			An[i] = minA + dfi*i;
			cosA[i] = cosf(An[i]);
			sinA[i] = sinf(An[i]);
		}
		// cout<<"minAngle: "<<PM_R2D*minA<<" maxAngle: "<<PM_R2D*An[scan->np-1]<<endl;
	}
	if(ox.size() != scan->np)
	{
		ox.resize(scan->np,0);
		oy.resize(scan->np,0);
	}

	float rx = scan->rx;
	float ry = scan->ry;
	float th = normAngle(scan->th-M_PI/2.,-M_PI);
	float x,y,tx,ty,nIdx,nIdy;
	float fcos = cosf(th);
	float fsin = sinf(th);
	int index = 0;
	for(int i=0;i<scan->np;i++)
	{
		if(scan->bad[i]) continue;

		// x=(scan->r[i])*cosA[i];
		// y=(scan->r[i])*sinA[i];
		x = scan->x[i];
		y = scan->y[i];
		tx = fcos*x-fsin*y+rx;
		ty = -fcos*y-fsin*x+ry;
		// ty=fsin*x+fcos*y+ry;

		// scan->x[i] = tx;
		// scan->y[i] = ty;
		ox[index] = tx;
		// oy[index] = m_max_y_len - ty;
		oy[index] = ty;
		++index; 
	}
	ox.resize(index);
	oy.resize(index);
}

void CVPmap::gen2Display(vector<int>& px, vector<int>& py, int p[2], PMScan* scan)
{
	int index = 0;
	static vector<float> gx;
	static vector<float> gy;
	translate2GlobalFrame(scan,gx,gy);
	px.resize(gx.size());
	py.resize(gx.size());
	for(int i=0;i<gx.size();i++)
	{
		px[i] = x2idx(gx[i]);
		py[i] = y2idx(gy[i]);
	}
	p[0] = x2idx(scan->rx);
	p[1] = y2idx(scan->ry);
}

void CVPmap::laserScanSimulator(PMScan* outScan, OrientedPoint2D& pose, \
				float fov, float min_angle, float max_range)
{
	double A,AA;
	AA = fov/(float)(outScan->np - 1);
	static bool once = true;
	static vector<float> angle_positive(outScan->np);
	static vector<float> angle_negative(outScan->np);
	
	static vector<float> ccosA(outScan->np,0);
	static vector<float> ssinA(outScan->np,0);

	if(once)
	{
		once = false;
		for(int i=0;i<outScan->np;i++)
		{
			angle_positive[i] = min_angle + i*AA;
			angle_negative[i] = -angle_positive[i];
		}
		// 90' as the initial angle
		float begin = -M_PI/4.0;
		for(int i=0;i<outScan->np;i++)
		{
			float angle = begin + i*AA;
			ccosA[i] = cosf(angle);
			ssinA[i] = sinf(angle);
		}
	}
	
	double theta = -1.*normAngle(pose.theta,-M_PI);
	// cout<<"pose.theta: "<<theta<<endl;
	// A = min_angle + pose.theta;
	A = angle_negative[0] + theta;
	static unsigned int max_ray_len = round(max_range / m_resolution);

	// cout<<"VPmap: max_ray_len is "<<max_ray_len<<endl;
	// cout<<"VPmap: size_x: "<<m_size_x<<" size_y: "<<m_size_y<<endl;
	int valid_count = 0;
	for(int i=0; i< outScan->np; i++, A-=AA)
	{
		bool valid;
		// cout<<"laser: "<<i<<" angle: "<<A*180./M_PI<<endl;
		simulateScanRay(pose.x,pose.y,A,outScan->r[i],valid,max_ray_len);
		outScan->bad[i] = valid?0:1;
		outScan->x[i] = outScan->r[i] * ccosA[i];
		outScan->y[i] = outScan->r[i] * ssinA[i];
		/*outScan->r[i]*=100.;
		outScan->x[i]*=100.;
		outScan->y[i]*=100.;*/
	}
	outScan->rx = pose.x;
	outScan->ry = pose.y;
	outScan->th = pose.theta;
}

void CVPmap::simulateScanRay(double sx, double sy, double angle,\
				float& out_range, bool& out_valid,\
				int max_ray_len, float threshold_free) 
{
	const double A_ = angle ; // + randomGenerator.drawGaussian1D_normalized()*angleNoiseStd;

	// Unit vector in the directorion of the ray:
	double cosA = cos(A_);
	double sinA = sin(A_);
	const double Arx =  cosA*m_resolution;
	const double Ary =  sinA*m_resolution;

	// Ray tracing, until collision, out of the map or out of range:
	unsigned int ray_len=0;
	unsigned int firstUnknownCellDist=max_ray_len+1;
	double rx=sx;
	double ry=sy;
	float hitCellOcc = 0.5f;
	int x, y=y2idx(ry);

	while ( (x=x2idx(rx))>=0 && (y=y2idx(ry))>=0 &&
		x<m_size_x && y< m_size_y && (hitCellOcc=m_mapprob[x][y])<threshold_free && ray_len<max_ray_len )
	{
		if(fabs(hitCellOcc - g_unknown_value)<0.01f)
		{
			if(ray_len < firstUnknownCellDist)
				firstUnknownCellDist = ray_len;
			break;
		}
		rx+=Arx;
		ry+=Ary;
		// ry-=Ary;
		ray_len++;
	}

	// Store:
	// Check out of the grid?
	// Tip: if x<0, (unsigned)(x) will also be >>> size_x ;-)
	if (fabs(hitCellOcc-g_unknown_value)<0.01f || static_cast<unsigned>(x)>=m_size_x || static_cast<unsigned>(y)>=m_size_y )
	{
		// cout<<"x: "<<x<<" y: "<<y<<endl;
		out_valid = false;
		if(firstUnknownCellDist < ray_len)
		{
			out_range = firstUnknownCellDist*m_resolution;
		}else{
			out_range = ray_len * m_resolution;
		}

		/*if (firstUnknownCellDist<ray_len)
			out_range = firstUnknownCellDist*m_resolution;
		else	out_range = ray_len*m_resolution;*/
	}
	else
	{ 	// No: The normal case:
		// out_range = ray_len*m_resolution;
		// cout<<"ray_len: "<<ray_len<<"max_ray_len: "<<max_ray_len<<endl;
		out_valid = ray_len < max_ray_len;
		out_range = ray_len*m_resolution;
		// Add additive Gaussian noise:
		/* if (noiseStd>0 && out_valid)
			out_range+=  noiseStd*randomGenerator.drawGaussian1D_normalized();*/
	}

	// out_range = ray_len*m_resolution;
	// out_x = out_range*cosA;
	// out_y = out_range*sinA;
}

namespace{
	ofstream rec("matched_nodes.log");
	void recordScans(int i, int j, PMScan* si, PMScan* sj)
	{
		OrientedPoint2D pose;
		rec<<i<<" "<<j<<" 0 "<<pose.x<<" "<<pose.y<<" "<<pose.theta<<endl;
		stringstream s2;
		s2<<j<<".log";
		ofstream rj(s2.str().c_str());
		for(int i=0;i<sj->np;i++)
		{
			rj<<sj->x[i]<<" "<<sj->y[i]<<endl;
		}
		if(j>1)
		{
			stringstream s1;
			s1<<i<<".log";
			ofstream ri(s1.str().c_str());
			for(int i=0;i<si->np;i++)
			{
				ri<<si->x[i]<<" "<<si->y[i]<<endl;
			}
		}
		return ;
	}
}

namespace{
	double square(double x) {return x*x;}
}

float CVPmap::obsLikelyhood3(PMScan* cs, OrientedPoint2D& p)
{
	double ret = 0;
	int K = (int)ceil(g_maxCorrDistance/g_pixResolution); 
	double maxCorrDist_sq = square(g_maxCorrDistance);
	const double constDist2DiscrUints = 100./(m_resolution*m_resolution);
	const double constDist2DiscrUints_INV = 1.0/constDist2DiscrUints;
	
	float stdHit = 0.35f;
	float zHit = 0.95f;
	float zRandom = 0.05f;
	float zRandomMaxRange = 50.f;
	float zRandomTerm = zRandom/zRandomMaxRange;

	double Q = -0.5f/square(stdHit);
#define LIK_CACHE_INVALID (66)
	if(m_likCache.size() != m_size_x)
	{
		m_likCache.resize(m_size_x, vector<float>(m_size_y, LIK_CACHE_INVALID) );
	}
	unsigned int size_x_1 = m_size_x - 1;
	unsigned int size_y_1 = m_size_y - 1;
	int M = 0;
	
	size_t decimation = g_scan_decimation;
	if(cs->np < 100) decimation = 1;
	
	double thisLik;
	double minimumLik = zRandomTerm + zHit*exp(Q*maxCorrDist_sq);

	double th = normAngle(p.theta-M_PI/2.,-M_PI);
	float ccos = cosf(th);
	float ssin = sinf(th);
	float gx, gy;

	double occupiedMinDist;
	for(size_t j=0;j<cs->np;j+=decimation)
	{
		occupiedMinDist = maxCorrDist_sq*10;

		// translate to the global coordinate
		// gx = p.x + cs->x[j]*ccos - cs->y[j]*ssin;
		// gy = p.y + cs->x[j]*ssin + cs->y[j]*ccos;

		gx = ccos*cs->x[j]-ssin*cs->y[j]+p.x;
		gy = -ccos*cs->y[j]-ssin*cs->x[j]+p.y;

		// translate to the PMap
		int cx = x2idx(gx);
		int cy = y2idx(gy);

		// outrange
		if(static_cast<unsigned>(cx) >= size_x_1 || \
		static_cast<unsigned>(cy) >= size_y_1)
		{
			thisLik = minimumLik;
		}else
		{
			thisLik = m_likCache[cx][cy];
			if(thisLik == LIK_CACHE_INVALID)
			{
				// compute now
				// Find the cloest occupied cell in K
				int xx1 = max(0,cx-K);
				int xx2 = min(size_x_1,(unsigned)(cx+K));
				int yy1 = max(0,cy-K);
				int yy2 = min(size_y_1,(unsigned)(cy+K));
				
				for(int yy=yy1; yy<=yy2; yy++)
					for(int xx=xx1; xx<=xx2; xx++)
					{
						float cellProb = m_mapprob[xx][yy];
						if(cellProb > g_free_threshold && fabs(cellProb-g_unknown_value) > 0.1f)
						{
							occupiedMinDist = min(occupiedMinDist, square(idx2x(xx)-gx) + square(idx2y(yy) - gy));
						}
					}
				
				/*
				{
					float* mapPtr = &m_mapprob[xx1][yy1];
					unsigned int incrAfterRow = m_size_x - ((xx2-xx1)+1);
					signed int Ax0 = 10*(xx1-cx);
					signed int Ay = 10*(yy1-cy);

					unsigned int occupiedMinDistInt = round(maxCorrDist_sq*constDist2DiscrUints)*10;
					for(int yy=yy1; yy<= yy2; yy++)
					{
						unsigned int Ay2 = square((unsigned int)(Ay));
						signed short Ax = Ax0;
						float cellProb;
						for(int xx = xx1; xx <= xx2; xx++)
						{
							cellProb = *mapPtr;
							mapPtr++;
							if(cellProb > g_free_threshold && fabs(cellProb-g_unknown_value) < 0.01f)
							{
								unsigned int d = square((unsigned int)(Ax)) + Ay2;
								occupiedMinDistInt = occupiedMinDistInt>d?d:occupiedMinDistInt;
							}
							Ax+=10;
						}
						mapPtr += incrAfterRow;
						Ay+=10;
					}
					occupiedMinDist = occupiedMinDistInt * constDist2DiscrUints_INV;
				}
				*/
				thisLik = zRandomTerm + zHit*exp(Q*occupiedMinDist);
				m_likCache[cx][cy] = thisLik;
			}
			}
		ret += log(thisLik);
		// ret += log(thisLik);
		}
	return ret;
}

float CVPmap::obsLikelyhood2(PMScan* cs, OrientedPoint2D& p)
{
	static PMScan* rs = new PMScan(cs->np);
	double pinit[3] = {0,};
	double pout[3] = {0,};
	laserScanSimulator(rs,p);
	
	static double stdLaser = 0.02; // laser noise
	static double stdSqrt2 = sqrt(2.0f)*stdLaser;
	// Compute likelihoods
	float ret = 1 ;
	float r_sim, r_obs;
	double likelihood;
	for(int j=0;j<cs->np;j++)
	{
		r_sim = rs->r[j];
		r_obs = cs->r[j];
		if(!cs->bad[j])
		{
			if(rs->bad[j])
			{
				likelihood = exp(-square((2.0f)/stdSqrt2));
			}else{
				likelihood = exp(-square(min(fabs(r_sim-r_obs),2.0f)/stdSqrt2));
			}
			ret += likelihood;
		}
	}
	return ret;
}

float CVPmap::obsLikelyhood(PMScan* cs, OrientedPoint2D& p, OrientedPoint2D& rel_p)
{
	static PMScan* rs = new PMScan(cs->np);
	double pinit[3] = {0,};
	double pout[3] = {0,};
	laserScanSimulator(rs,p);
	float weight = m_pICP->ICPMatch(rs->x,rs->y,cs->x,cs->y,rs->np,pinit,pout);
	if(weight > 0.5){
		rel_p =  OrientedPoint2D(pout[0],pout[1],pout[2]);
		// p = p.oplus(trans);
	}
	/*static int index = 1 ;
	if(weight > 0.4)
	{
		recordScans(0,index++,cs,rs);
		cout<<"VPmap.cpp: this match weight: "<<weight<<endl;
	}*/	
	return weight;
}



