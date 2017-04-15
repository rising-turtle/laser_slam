#ifndef POLAR_MATCH_H
#define POLAR_MATCH_H

#include <string>
#include <vector>
#include <utility>
#include "PolarParameter.h"
using namespace std;

struct _PMScan;

// To handle the front_end of our SLAM algorithm, that is,
// Input  / two scan_matchings
// Output / relative motions

class CPolarMatch
{
public:
	CPolarMatch(string laser_name);
	~CPolarMatch();

	void runSICKFile(string filename, int run_num =-1);// run PSM using SICK log 
	void runFlirtFile(string filename, int run_num = -1); // run Flirt log
	void runOurFile(string filename, int run_num = -1); // run our file log like carmen log
	void run511Data(string filename, int run_num = -1); // run 511 Data to compare with original psm 
	bool construct511Data(char* , PMScan&);
	// interface for matching two scans
	pair<float,float> FMatch(PMScan* lsr, PMScan* lsa, bool bFirst=false); // square dis

// protected:
public:
	// PSM Match Functions
	void pm_median_filter(PMScan* ls); // median filter the scan-matching
	void pm_find_far_points (PMScan *ls ); // far filter
	void pm_segment_scan ( PMScan *ls );	// segment
	bool pm_is_corridor(PMScan * ); // corridor judgement
	void pm_scan_project(const PMScan *act,  PM_TYPE   *new_r,  int *new_bad); //Performs scan projection.
	PM_TYPE pm_orientation_search(const PMScan *ref, const PM_TYPE *new_r, const int *new_bad); //Performs one iteration of orientation alignment of current scan.
	PM_TYPE pm_translation_estimation(const PMScan *ref, const PM_TYPE *new_r, const int *new_bad, PM_TYPE C, PM_TYPE *dx, PM_TYPE *dy); // Estimate the postion of the current scan with respect to a reference scan.
	pair<PM_TYPE,PM_TYPE> pm_error_index2(PMScan *ref,PMScan *cur, int* associatedPoints=NULL);
	void pm_init(); // init bearing variables
	void pm_preprocessScan(PMScan *ls); // preprocess the scan-matching
	PM_TYPE pm_psm ( const PMScan *lsr,PMScan *lsa ); //Match two laser scans using polar scan matching. 

	// ICP Match Functions
	PM_TYPE pm_icp ( const PMScan *lsr,PMScan *lsa ); //Match two laser scans using polar scan matching. 
	PM_TYPE point_line_distance ( PM_TYPE x1, PM_TYPE y1, PM_TYPE x2, PM_TYPE y2,
		PM_TYPE x3, PM_TYPE y3,PM_TYPE *x, PM_TYPE *y );

public:
	bool readSICK(string filename);  // Input from Sick File
	bool readFlirt(string filename); // Input from Flirt File
	bool readCarmon(string filename, string laser=""); // Input from Carmon File
	bool readRawSeed(string filename); // Input from RawSeed file

public:
	PM_TYPE*   pm_fi;//contains precomputed angles 
	PM_TYPE*   pm_si;//contains sinus of angles
	PM_TYPE*   pm_co;//contains cos of angles

public:
	Base_PARAM* m_pParam;	// Parameters for laser 
	bool m_bReady;			// Whether the laser info is right
	vector<_PMScan*> m_SickScans;	// Record scans
public:
	void clearRecord(); // clear previous records
	void runlog(int run_num); // rum slam using log data
	void runlogImproved(int run_num);	// rum slam using log data
	void testAccuracy(string filename); // test accuracy of psm and icp
private:
	CPolarMatch(const CPolarMatch&);
	CPolarMatch& operator=(const CPolarMatch&);

private:
	/** @brief Normalize angle.
	Normalize angle to be within [-pi,pi).*/
	inline PM_TYPE norm_a ( PM_TYPE a )
	{
		int m;
		m = (int) ( a / ( 2.0*M_PI ) );
		a = a - (PM_TYPE)m * M_PI;
		if ( a < (-M_PI) )
			a += 2.0*M_PI;
		if ( a >= M_PI )
			a -= 2.0*M_PI;
		return ( a );
	}
};

#endif
