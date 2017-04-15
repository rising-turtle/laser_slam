#ifndef ZHCANONICAL_H
#define ZHCANONICAL_H
#include <vector>
#include <string>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>


using namespace std;

struct _PMScan;
struct _PARAM;

// struct used in CSM
struct laser_data;
struct sm_params;
struct sm_result;

class CCanonicalMatcher{
public:
	CCanonicalMatcher(struct _PARAM* p=0);
	~CCanonicalMatcher();
	void pm_init();
	void sm_init(struct _PARAM*);
	void setCov(bool);
public:
	void fromPMScan2LDP(struct _PMScan*,struct laser_data** ,struct _PARAM*);
	void median_filter(struct _PMScan*);
	float FMatch(struct _PMScan* lsr, struct _PMScan* lsa, bool bFirst = false);
	float FMatch(struct laser_data*, struct laser_data*);
	void resetKeyFrame(struct _PMScan*);

	// How to define KeyFrame? Based on dis or err or both?
	float FMatchKeyFrame2(struct _PMScan*, Eigen::Matrix3d&);
	float FMatchKeyFrame(struct _PMScan* ); // match with key frame
	float FMatchKeyFrame(struct laser_data*);
	void testKeyFrameMatch(string, int); // test the validity of Keyframe Matching
	bool newKeyframeNeeded(double[3]);
	struct laser_data* m_pKeyframe;
	
	void setThreshold(float, float);
	float threshold_angular_;
	float threshold_dist_sq_;

	sm_params* input;
	sm_result* output;
	struct _PARAM* m_pParam;
public:
	void runlog(string file, int num); // for test, read from file
	void runlogImproved(string file, int num);
	void compareCSM2PSM_RawSeed(string, int);
	void compareCSM2PSM_Lenovo(string, int);
	void compareCSM2PSM_511(string , int);
	bool constructPSM(char* line, struct _PMScan& scan, struct _PARAM*);
	bool constructPSM511(char* line, struct _PMScan& scan, struct _PARAM*);
	vector<struct laser_data* > m_Scans; 
	struct laser_data* m_prev;
	struct laser_data* m_pcur;
public:
	float * pm_fi;
	float * pm_si;
	float * pm_co;
private:
	// Normalize angle to be within [-pi,pi)
	inline float norm_a(float a){
		int m = (int)(a/(2.0*M_PI));
		a = a-(float)m*M_PI;
		if(a<(-M_PI)){
			a+=2.0*M_PI;
		}
		if(a>=M_PI){
			a-=2.0*M_PI;
		}
		return a;
	}

private:
	CCanonicalMatcher& operator= (const CCanonicalMatcher&);
	CCanonicalMatcher(const CCanonicalMatcher&);
	//CCanonicalMatcher operator=(const CCanonicalMatcher&);

};

#endif 
