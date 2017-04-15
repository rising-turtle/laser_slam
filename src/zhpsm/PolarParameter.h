#ifndef POLAR_PARAMETER
#define POLAR_PARAMETER

// Description of range reading errors. Each range measurement may be tagged with one of these:
#define PM_RANGE     1  ///< Measurement tag: range reading is longer than PM_MAX_RANGE.
#define PM_MOVING    2  ///< Measurement tag: range reading corresponds to a moving point. Not implemented yet.
#define PM_MIXED     4  ///< Measurement tag: range reading is a mixed pixel.
#define PM_OCCLUDED  8  ///< Measurement tag: range reading is occluded.
#define PM_EMPTY     16 ///< Measurement tag: no measurment (between 2 segments there is no interpolation!)

#define PM_LASER_Y          0 ///<@brief [cm] Y coordinate of the laser on the robot.
#define PM_MIN_RANGE        10.0f // 10.0f ///< [cm] Minimum valid laser range for the reprojected current scan.
#define PM_SEG_MAX_DIST     20.0 ///< The distance between points to break a segment. 
#define PM_WEIGHTING_FACTOR 70*70 ///< Parameter used for weighting associated points in the position calculation of PSM. Try setting it to 30*30 for laser odometry.
#define PM_CHANGE_WEIGHT_ITER 10 ///< The number of iterations after which the weighting factor is reduced to weight down outliers.

#define PM_TYPE             float ///< The variable type used in calculations. Change it to double for higher accuracy and lower speed.

#define PM_MAX_ERROR        100  ///< [cm] Maximum distance between associated points used in pose estimation. Try setting it to 30 for laser odometry.
#define PM_STOP_COND         0.4  ///< If the pose change (|dx|+|dy|+|dth|) is smaller than this PSM scan matching stops.
#define PM_MAX_ITER         30   ///< Maximum number of iterations for PSM.
#define PM_MAX_ITER_ICP     60   ///< Maximum number of iterations for ICP
#define PM_STOP_COND_ICP    0.1  ///< Stopping condition for ICP. The pose change has to be smaller than this.
#define PM_CORRIDOR_THRESHOLD 25.0 ///< Threshold for angle variation between points to determine if scan was taken of a corridor.


#define _BEARING_181 "LMS211"
#define _BEARING_361 "LMS511"
#define _BEARING_541 "LMS151"

#include <string>
#include <map>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <vector>

#define M_PI  3.141592654
#define PM_D2R  (M_PI/180.0) // degrees to rad
#define PM_R2D  (180.0/M_PI) // rad to degrees

typedef struct _PARAM{
public:
	std::string pm_laser_name;///< The name of the laser range finder.
	int pm_l_points;///< Maximum number of points in a scan.
	int pm_fov;///< Field of view of the laser range finder in degrees.
	int pm_max_range;///< [cm] Maximum valid laser range .
	int pm_min_range;///< [cm] Minimum valid laser range .
	int pm_min_valid_points;///< Minimum number of valid points for scan matching.
	int pm_scan_window;///< Half window size which is searched for correct orientation.
	float pm_corridor_threshold; ///< Threshold for angle variation between points to determine if scan was taken of a corridor.

	float pm_fi_min;//[rad] bearing from which laser scans start
	float pm_fi_max;//[rad] bearing at which laser scans end
	float pm_dfi; //[rad] angular resolution of laser scans

	_PARAM(std::string laser_name, int points,int fov, int max_range,int valid_points,\
		int scan_window,int corridor_thre, int fi_min, int fi_max):
	pm_laser_name(laser_name),pm_l_points(points),pm_fov(fov),\
	pm_max_range(max_range),pm_min_valid_points(valid_points),\
	pm_scan_window(scan_window),pm_corridor_threshold(corridor_thre),\
	pm_fi_min(fi_min*PM_D2R),pm_fi_max(fi_max*PM_D2R)
	{
		pm_min_range = PM_MIN_RANGE;
		pm_dfi = (pm_fov*PM_D2R)/(pm_l_points - 1.0);
	}
	virtual ~_PARAM(){}
}Base_PARAM;

typedef struct _PM_SICK_LMS511 : public Base_PARAM{
public:
	_PM_SICK_LMS511():Base_PARAM("LMS511",361,180,5000,80,40,25.0,0,180){}
}PM_SICK_LMS511;

typedef struct _PM_SICK_LMS211 : public Base_PARAM{
public:
	_PM_SICK_LMS211():Base_PARAM("LMS211",181,180,5000,40,20,25.0,-90,90){}
}PM_SICK_LMS211;

typedef struct _PM_SICK_LMS151 : public Base_PARAM{
public:
	_PM_SICK_LMS151():Base_PARAM("LMS151",541,270,5000,100,50,25.0,-45,225){}
}PM_SICK_LMS151;


template<typename PARAMT>
Base_PARAM* ObtainParam()
{
	return new PARAMT;
}

/** @brief Structure describing a laser scan.

The robot pose (rx,ry,th) describes the odometry center if PM_LASER_Y 
does not equal 0, or the laser center if PM_LASER_Y=0. <br>

In a laser scan, the middle laser bearing coincides with the laser coordinate frame's Y axis.

TODO: Consider using doubles for rx,ry in large environments.
*/

#define NEW_OP
//#define VEC_OP 

typedef struct _PMScan
{
	double   t;    ///<[s] Time when scan was taken.
	float  rx;   ///<[cm] Robot odometry X coordinate.
	float  ry;   ///<[cm] Robot odometry Y coordinate.
	float  th;   ///<[rad] Robot orientation.
	int np; // number of points*/
#ifdef NEW_OP
	
	float* r;///<[cm] Laser range readings. 0 or negative ranges denote invalid readings.
	float* x;///<[cm] Laser reading X coordinates in Cartesian coordinates.
	float* y;///<[cm] Laser reading Y coordinates in Cartesian coordinates.
	int*   bad;///< @brief Tag describing the validity of a range measurement.
	///< 0 if OK; sources of invalidity - out of range reading;
	///< reading belongs to moving object (not implemented); occlusion; mixed pixel.
	int*   seg;///< Describes which segment the range reading belongs to.
#endif

#ifdef VEC_OP
	std::vector<float> r;
	std::vector<float> x;
	std::vector<float> y;
	std::vector<int> bad;
	std::vector<int> seg;
#endif

#ifdef VEC_OP
	_PMScan(int pts):np(pts),r(pts),x(pts),y(pts),bad(pts),seg(pts){}
#endif

#ifdef NEW_OP
	_PMScan(int pm_l_poitns):np(pm_l_poitns){
		r = new float[pm_l_poitns];
		x = new float[pm_l_poitns];
		y = new float[pm_l_poitns];
		bad = new int[pm_l_poitns];
		seg = new int[pm_l_poitns];
	}
	_PMScan(const _PMScan& ref){
		t = ref.t; rx = ref.rx; ry = ref.ry; th = ref.th;
		np = ref.np;
		r = new float[np];
		x = new float[np];
		y = new float[np];
		bad = new int[np];
		seg = new int[np];
		memcpy(r,ref.r,sizeof(float)*np);
		memcpy(x,ref.x,sizeof(float)*np);
		memcpy(y,ref.y,sizeof(float)*np);
		memcpy(bad,ref.bad,sizeof(int)*np);
		memcpy(seg,ref.seg,sizeof(int)*np);
	}
	_PMScan& operator=(_PMScan& ref){
		if(this!=&ref){
			t = ref.t; rx = ref.rx; ry = ref.ry; th = ref.th;
			if(np<ref.np){
				r = (float*)realloc(r,sizeof(float)*ref.np);
				x = (float*)realloc(x,sizeof(float)*ref.np);
				y = (float*)realloc(y,sizeof(float)*ref.np);
				bad = (int*)realloc(bad,sizeof(int)*ref.np);
				seg = (int*)realloc(seg,sizeof(int)*ref.np);
			}
			np = ref.np;
			memcpy(r,ref.r,sizeof(float)*np);
			memcpy(x,ref.x,sizeof(float)*np);
			memcpy(y,ref.y,sizeof(float)*np);
			memcpy(bad,ref.bad,sizeof(int)*np);
			memcpy(seg,ref.seg,sizeof(int)*np);
		}
		return (*this);
	}
	~_PMScan(){
		delete []r;
		delete []x;
		delete []y;
		delete []bad;
		delete []seg;
	}
#endif
}PMScan;


#endif
