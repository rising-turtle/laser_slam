/*
 * config.hpp
 *
 *  Created on: Dec 3, 2012
 *      Author: liu
 */

#ifndef CONFIG_HPP_
#define CONFIG_HPP_

#include <stdio.h>
#include <stdlib.h>


#include "unsFlt.hpp"
#include <boost/numeric/ublas/io.hpp>

using namespace std;
using namespace Bayesian_filter;
using namespace Bayesian_filter_matrix;
/*
 * Simple Prediction model
 */
class Simple_predict : public Linear_predict_model
{
public:
	Simple_predict() : Linear_predict_model(1,1)
	// Construct a constant model
	{
		// Stationary Prediction model (Identity)
		Fx(0,0) = 1.;
		// Constant Noise model with a large variance
		q[0] = 2.;
		G(0,0) = 1.;
	}
};

/*
 * Simple Observation model
 */
class Simple_observe : public Linear_uncorrelated_observe_model
{
public:
	Simple_observe () : Linear_uncorrelated_observe_model(1,1)
	// Construct a constant model
	{
		// Linear model
		Hx(0,0) = 1;
		// Constant Observation Noise model with variance of one
		Zv[0] = 1.;
	}
};


/*
 * robot Prediction model
 */
class Robot_predict : public Linear_predict_model
{
public:
	Robot_predict() : Linear_predict_model(3,3)
	// Construct a constant model
	{
		// Stationary Prediction model (Identity)
		identity(Fx);
		// Constant Noise model with a large variance
		q[0] = 100000;//0.1;//0.1;
		q[1] = 100000;// 0.1;//0.1;
		q[2] = 100000;//0.02;//0.02;
		identity(G);
	}
};

/*
 * SICK Observation model
 */
class SynGlobal_observe : public Linear_uncorrelated_observe_model
{
public:
	SynGlobal_observe () : Linear_uncorrelated_observe_model(3,3)
	// Construct a constant model
	{
		// Linear model
		identity(Hx);
		// Constant Observation Noise model with variance of one

		Zv[0] = 0.005;
		Zv[1] = 0.005;
		Zv[2] = 0.005;
	}
};

/*
 * Localization Observation model
 */
class SynLocalize_observe : public Linear_uncorrelated_observe_model
{
public:
	SynLocalize_observe () : Linear_uncorrelated_observe_model(3,3)
	// Construct a constant model
	{
		// Linear model
		identity(Hx);
		// Constant Observation Noise model with variance of one

		Zv[0] = 0.005;
		Zv[1] = 0.005;
		Zv[2] = 0.005;
	}
};


/*
 * All in one Observation model
 */
class AIO_observe : public Linear_uncorrelated_observe_model
{
public:
	AIO_observe () : Linear_uncorrelated_observe_model(3,6)
	// Construct a constant model
	{
		// Linear model
		//identity(Hx);

		for(int i=0;i<3;i++)
		{
			Hx(i,i) = 1;
			Hx(i+3,i) = 1;
		}

		/*
		for(int i=0;i<6;i++)
		{
			for(int j=0;j<3;j++)
			{
				printf("Hx %f", Hx(i,j));
			}
			printf("\n");
		}
		*/

		// Constant Observation Noise model with variance of one
		// Zv[0] = 0.02;
		// Zv[1] = 0.02;
		// Zv[2] = 0.02;
		Zv[0] = 0.25;
		Zv[1] = 0.25;
		Zv[2] = 0.02;
		Zv[3] = 0.25;
		Zv[4] = 0.25;
		Zv[5] = 0.02;
	}
};

/*
 * SICK Observation model
 */
class SICK_observe : public Linear_uncorrelated_observe_model
{
public:
	SICK_observe () : Linear_uncorrelated_observe_model(3,3)
	// Construct a constant model
	{
		// Linear model
		identity(Hx);
		// Constant Observation Noise model with variance of one
		// Zv[0] = 0.02;
		// Zv[1] = 0.02;
		// Zv[2] = 0.02;
		Zv[0] = 0.25;
		Zv[1] = 0.25;
		Zv[2] = 0.02;
	}
};

/*
 * GPS Observation model
 */

class GPS_observe : public Uncorrelated_additive_observe_model
{
public:
	GPS_observe () : Uncorrelated_additive_observe_model(1), hx(1)
	{
		Zv[0] = 0;
	}

	const FM::Vec& h(const FM::Vec& x) const
	{
		hx[0] = x[0]*x[0] + x[1]*x[1];
		return hx;
	}

private:
	 mutable Vec hx;

};



/*
 * ODO Observation model
 */
class ODO_observe : public Linear_uncorrelated_observe_model
{
public:
	ODO_observe () : Linear_uncorrelated_observe_model(3,3)
	// Construct a constant model
	{
		// Linear model
		identity(Hx);

		// Constant Observation Noise model with variance of one
		Zv[0] = 0.25;
		Zv[1] = 0.25;
		Zv[2] = 0.02;
	}
};


/*
 * GPS Observation model
 */
class BN_observe : public Linear_uncorrelated_observe_model
{
public:
	BN_observe () : Linear_uncorrelated_observe_model(3,3)
	// Construct a constant model
	{
		// Linear model
		identity(Hx);

		// Constant Observation Noise model with variance of one
		Zv[0] = 0.0001;
		Zv[1] = 0.0001;
		Zv[2] = 0.0001;
	}
};


#endif /* CONFIG_HPP_ */
