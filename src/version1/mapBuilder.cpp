
#include <cmath>
#include <vector>
using namespace std;

void translate2GlobalFrame(float* bearing, vector<float>& fx,vector<float>& fy, double rx, double ry, double th)
{
#define PI 3.141592654
#define D2R(d) ((d)*PI/180.)
	const float bad_range = -100000; 
	// Parameters for SICK 151
	static int BEAR_NUM = 541;
	static int MAX_RANGE = 5000;
	static int MIN_RANGE = 10;
	static float MIN_ANGLE = D2R(-45);
	static float MAX_ANGLE = D2R(225);
	static int FOV = 270;
	static bool bfirst = true;
	static float pm_co[BEAR_NUM];
	static float pm_si[BEAR_NUM];
	
	if(bfirst){
		bfirst = false;
		float pm_dfi = D2R(FOV)/(BEAR_NUM-1.);
		for(int i=0;i<BEAR_NUM;i++){
			float angle = ((float)i)*pm_dfi + MIN_ANGLE;
			pm_si[i] = sinf(angle);
			pm_co[i] = cosf(angle);
		}
	}

	if(fx.size()<BEAR_NUM){
		fx.resize(BEAR_NUM);
		fy.resize(BEAR_NUM);
	}

	float x,y,tx,ty;
	float fcos = cosf(th);
	float fsin = sinf(th);
	int index = 0;
	for(int i=0;i<BEAR_NUM;i++){
		if(bearing[i] > MAX_RANGE || bearing[i] < MIN_RANGE){ // this is bad range 
			continue;
		}
		x=(bearing[i]/100.0)*pm_co[i];
		y=(bearing[i]/100.0)*pm_si[i];

		tx=fcos*x-fsin*y+rx;
		ty=fsin*x+fcos*y+ry;
	
		fx[index] = tx;
		fy[index] = ty;
		index++;
	}
	fx.resize(index);
	fy.resize(index);
}
