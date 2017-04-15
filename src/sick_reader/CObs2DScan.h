/*
 * CObs2DScan.h
 *
 *  Created on: Nov 26, 2012
 *      Author: liu
 */

#ifndef COBS2DSCAN_H_
#define COBS2DSCAN_H_

#include "cpp_utils.h"

#include <math.h>


using namespace std;

/** Auxiliary struct that holds all the relevant *geometry* information about a 2D scan.
 * This class is used in CSinCosLookUpTableFor2DScans
 * \ingroup mrpt_obs_grp
 * \sa CObservation2DRangeScan and CObservation2DRangeScan::getScanProperties */

struct  T2DScanProperties {
	size_t  nRays;
	double  aperture;
	bool    rightToLeft;
};
bool  operator<(const T2DScanProperties&a, const T2DScanProperties&b);	//!< Order operator, so T2DScanProperties can appear in associative STL containers.



//DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CObservation2DRangeScan, CObservation, OBS_IMPEXP)

/** A "CObservation"-derived class that represents a 2D range scan measurement (typically from a laser scanner).
 *  The data structures are generic enough to hold a wide variety of 2D scanners and "3D" planar rotating 2D lasers.
 *
 *  These are the most important data fields:
 *    - CObservation2DRangeScan::scan -> A vector of float values with all the range measurements (in meters).
 *    - CObservation2DRangeScan::validRange -> A vector (of <b>identical size</b> than <i>scan<i>), has non-zeros for those ranges than are valid (i.e. will be zero for non-reflected rays, etc.)
 *    - CObservation2DRangeScan::aperture -> The field-of-view of the scanner, in radians (typically, M_PI = 180deg).
 *    - CObservation2DRangeScan::sensorPose -> The 6D location of the sensor on the robot reference frame (default=at the origin).
 *
 * \sa CObservation, CPointsMap, T2DScanProperties
 * \ingroup mrpt_obs_grp
 */
class  CObs2DScan
{
public:

	/** Default constructor */
	CObs2DScan( );

	/** Destructor */
	virtual ~CObs2DScan( );


	/** @name Scan data
		    @{ */

	//system time
	TTimeStamp timestamp;

	//sensor label
	std::string			sensorLabel;

	/** The range values of the scan, in meters.
	 */
	std::vector<float>	    scan;

	/** It's false (=0) on no reflected rays, referenced to elements in "scan"
	 *  (Added in the streamming version #1 of the class)
	 */
	std::vector<char>	validRange;

	/** The aperture of the range finder, in radians (typically M_PI = 180 degrees).
	 */
	float				aperture;

	/** The scanning direction
	 */
	bool				rightToLeft;

	/** The maximum range allowed by the device, in meters (e.g. 80m, 50m,...)
	 */
	float				maxRange;

	/** The 6D pose of the sensor on the robot.
	 */
	CPose3D				sensorPose;

	/** The "sigma" error of the device in meters, used while inserting the scan in an occupancy grid.
	 */
	float				stdError;

	/** The aperture of each beam, in radians, used to insert "thick" rays in the occupancy grid.
	 * (Added in the streamming version #4 of the class)
	 */
	float				beamAperture;

	/** If the laser gathers data by sweeping in the pitch/elevation angle, this holds the increment in "pitch" (=-"elevation") between the beginning and the end of the scan (the sensorPose member stands for the pose at the beginning of the scan).
	 */
	double				deltaPitch;

	/** Fill out a T2DScanProperties structure with the parameters of this scan */
	void getScanProperties(T2DScanProperties& p) const;

	/** @} */


	/** @name Cached points map
		    @{  */

protected:


public:


	/** A general method to truncate the scan by defining a minimum valid distance and a maximum valid angle as well as minimun and maximum heights
		   (NOTE: the laser z-coordinate must be provided).
	 */
	void truncateByDistanceAndAngle(float min_distance, float max_angle, float min_height = 0, float max_height = 0, float h = 0 );

	/** Mark as invalid the ranges in any of a given set of "forbiden angle ranges", given as pairs<min_angle,max_angle>.
	 * \sa C2DRangeFinderAbstract::loadExclusionAreas
	 */
	void filterByExclusionAngles( const std::vector<std::pair<double,double> >  &angles );

}; // End of class def.

#endif /* COBS2DSCAN_H_ */
