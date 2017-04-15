/*
 * CObs2DScan.cpp
 *
 *  Created on: Nov 26, 2012
 *      Author: liu
 */

#include "CObs2DScan.h"

using namespace std;



/*---------------------------------------------------------------
							Constructor
 ---------------------------------------------------------------*/
CObs2DScan::CObs2DScan( ) :
	scan(),
	validRange(),
	aperture( M_PI ),
	rightToLeft( true ),
	maxRange( 80.0f ),
	sensorPose(),
	stdError( 0.01f ),
	beamAperture(0),
	deltaPitch(0)
{
}

/*---------------------------------------------------------------
							Destructor
 ---------------------------------------------------------------*/
CObs2DScan::~CObs2DScan()
{
}


/*---------------------------------------------------------------
  Filter out invalid points by a minimum distance, a maximum angle and a certain distance at the end (z-coordinate of the lasers must be provided)
 ---------------------------------------------------------------*/
void CObs2DScan::truncateByDistanceAndAngle(float min_distance, float max_angle, float min_height, float max_height, float h )
{
	// FILTER OUT INVALID POINTS!!
	std::vector<float>::iterator		itScan;
	std::vector<char>::iterator itValid;
	CPose3D						pose;
	unsigned int				k;
	unsigned int				nPts = scan.size();

	for( itScan = scan.begin(), itValid = validRange.begin(), k = 0;
		 itScan != scan.end();
		 itScan++, itValid++, k++ )
	{
		float ang	= fabs(k*aperture/nPts - aperture*0.5);
		float x		= (*itScan)*cos(ang);

		if( min_height != 0 || max_height != 0 )
		{
			//ASSERT_( max_height > min_height );
			if( *itScan < min_distance || ang > max_angle || x > h - min_height || x < h - max_height )
				*itValid = false;
		} // end if
		else
			if( *itScan < min_distance || ang > max_angle )
				*itValid = false;
	}
}



/*---------------------------------------------------------------
						filterByExclusionAngles
 ---------------------------------------------------------------*/
void CObs2DScan::filterByExclusionAngles( const std::vector<std::pair<double,double> >  &angles )
{
	if (angles.empty()) return;



	double	Ang, dA;
	const size_t  sizeRangeScan = scan.size();

	//ASSERT_(scan.size()==validRange.size());

	if (!sizeRangeScan) return;

	if (rightToLeft)
	{
		Ang = - 0.5 * aperture;
		dA  = aperture / (sizeRangeScan-1);
	}
	else
	{
		Ang = + 0.5 * aperture;
		dA  = - aperture / (sizeRangeScan-1);
	}

	// For each forbiden angle range:
	for (vector<pair<double,double> >::const_iterator itA=angles.begin();itA!=angles.end();++itA)
	{
		int ap_idx_ini = wrapTo2Pi(itA->first-Ang) / dA;  // The signs are all right! ;-)
		int ap_idx_end = wrapTo2Pi(itA->second-Ang) / dA;

		if (ap_idx_ini<0) ap_idx_ini=0;
		if (ap_idx_end<0) ap_idx_end=0;

		if (ap_idx_ini>(int)sizeRangeScan) ap_idx_ini=sizeRangeScan-1;
		if (ap_idx_end>(int)sizeRangeScan) ap_idx_end=sizeRangeScan-1;

		const size_t idx_ini = ap_idx_ini;
		const size_t idx_end = ap_idx_end;

		if (idx_end>=idx_ini)
		{
			for (size_t i=idx_ini;i<=idx_end;i++)
				validRange[i]=false;
		}
		else
		{
			for (size_t i=0;i<idx_end;i++)
				validRange[i]=false;

			for (size_t i=idx_ini;i<sizeRangeScan;i++)
				validRange[i]=false;
		}
	}


}



/** Fill out a T2DScanProperties structure with the parameters of this scan */
void CObs2DScan::getScanProperties(T2DScanProperties& p) const
{
	p.nRays       = this->scan.size();
	p.aperture    = this->aperture;
	p.rightToLeft = this->rightToLeft;
}


