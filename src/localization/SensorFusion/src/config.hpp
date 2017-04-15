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
		q[0] = 0.02; //about 14cm error;
		q[1] = 0.02; //about 14cm error;
		q[2] = 0.02; //about 10 deg error;
		identity(G);
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
		Zv[0] = 0.001;
		Zv[1] = 0.001;
		Zv[2] = 0.001;
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
 * Odometry Observation model
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
		Zv[0] = 0.001;
		Zv[1] = 0.001;
		Zv[2] = 0.001;
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
