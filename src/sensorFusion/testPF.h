/*
 * testPF.h
 *
 *  Created on: Mar 11, 2013
 *      Author: liu
 */

#ifndef TESTPF_H_
#define TESTPF_H_


		// Bayes++ Bayesian filtering schemes
#include "SIRFlt.hpp"
#include "covFlt.hpp"
#include "unsFlt.hpp"
#include "models.hpp"
#include "matSup.hpp"

#include <algorithm>
#include <iterator>
#include <vector>
		// Types required for SLAM classes
#include <vector>
#include <map>
		// Bayes++ SLAM
#include "SLAM.hpp"
#include "fastSLAM.hpp"
#include "kalmanSLAM.hpp"

#include "random.hpp"
#include <iostream>
#include <boost/numeric/ublas/io.hpp>
#include <boost/lexical_cast.hpp>

using namespace SLAM_filter;


class SLAM_random : public Bayesian_filter_test::Boost_random, public BF::SIR_random
/*
 * Random numbers for SLAM test
 */
{
public:
	FM::Float normal (const FM::Float mean, const FM::Float sigma)
	{
		return Boost_random::normal (mean, sigma);
	}
	void normal (FM::DenseVec& v)
	{
		Boost_random::normal (v);
	}
	void uniform_01 (FM::DenseVec& v)
	{
		Boost_random::uniform_01 (v);
	}
	void seed ()
	{
		Boost_random::seed();
	}
};


/*
 * Demonstrate a SLAM example
 */
struct SLAMDemo
{
	const unsigned nParticles;

	SLAMDemo (unsigned setnParticles) : nParticles(setnParticles)
	{}
	void OneDExperiment ();
	void InformationLossExperiment ();
	void pfExp();

	SLAM_random goodRandom;

	// Relative Observation with  Noise model
	struct Simple_observe : BF::Linear_uncorrelated_observe_model
	{
		Simple_observe (Float i_Zv) : Linear_uncorrelated_observe_model(2,1)
		// Construct a linear model with const Hx
		{
			Hx(0,0) = -1.;	// Location
			Hx(0,1) = 1.;	// Map
			Zv[0] = i_Zv;
		}
	};
	struct Simple_observe_inverse : BF::Linear_uncorrelated_observe_model
	{
		Simple_observe_inverse (Float i_Zv) : Linear_uncorrelated_observe_model(2,1)
		{
			Hx(0,0) = 1.;	// location
			Hx(0,1) = 1.;	// observation
			Zv[0] = i_Zv;
		}
	};

	struct Kalman_statistics : public BF::Kalman_state_filter
	// Kalman_statistics without any filtering
	{
		Kalman_statistics (std::size_t x_size) : Kalman_state_filter(x_size) {}
		void init() {}
		void update() {}
	};

	template <class Filter>
	struct Generic_kalman_generator : public Kalman_filter_generator
	// Generate and dispose of generic kalman filter type
	{
		Filter_type* generate( unsigned full_size )
		{
			return new Filter(full_size);
		}
		void dispose( Filter_type* filter )
		{
			delete filter;
		}
	};


	void display( const std::string label, const BF::Kalman_state_filter& stats)
	{
		std::cout << label << stats.x << stats.X << std::endl;
	}
};


void SLAMDemo::pfExp()
// Experiment with a one dimensional problem
//  Use to look at implication of highly correlated features
{
	// State size
	const unsigned nL = 1;	// Location
	const unsigned nM = 2;	// Map

	// Construct simple Prediction models
	BF::Sampled_LiAd_predict_model location_predict(nL,1, goodRandom);
	// Stationary Prediction model (Identity)
	FM::identity(location_predict.Fx);
				// Constant Noise model
	location_predict.q[0] = 1000.;
	location_predict.G.clear();
	location_predict.G(0,0) = 1.;

	// Relative Observation with  Noise model
	Simple_observe observe0(5.), observe1(3.);
	Simple_observe_inverse observe_new0(5.), observe_new1(3.);

	// Setup the initial state and covariance
	// Location with no uncertainty
	FM::Vec x_init(nL); FM::SymMatrix X_init(nL, nL);
	x_init[0] = 20.;
	X_init(0,0) = 0.;

	// Truth model : location plus one map feature
	FM::Vec true0(nL+1), true1(nL+1);
	true0.sub_range(0,nL) = x_init; true0[nL] = 50.;
	true1.sub_range(0,nL) = x_init; true1[nL] = 70.;
	FM::Vec z(1);

	//PF
	BF::SIR_scheme mPF(nL, nParticles, goodRandom);

	// Samples at mean
	const std::size_t nSamples = mPF.S.size2();
	for (std::size_t i = 0; i != nSamples; ++i) {
		FM::ColMatrix::Column Si(mPF.S,i);
		noalias(Si) = x_init;
	}
	// Decorrelate init state noise
	/* FM::Matrix UD(x_init.size(),x_init.size());
	FM::Float rcond = FM::UdUfactor (UD, X_init);
	BF::Numerical_rcond rclimit;
	rclimit.check_PSD(rcond, "Init X not PSD");

	// Sampled predict model for initial noise variance
	FM::identity (location_predict.Fx);
	FM::UdUseperate (location_predict.G, location_predict.q, UD);
	location_predict.init_GqG ();
	*/
	// Predict using model to apply initial noise
	mPF.predict (location_predict);
	mPF.init_S();

	// Initial feature states
	z = observe0.h(true0);		// Observe a relative position between location and map landmark
	z[0] += 0.5;
	mPF.observe_likelihood(z);
	mPF.update_resample();

}




#endif /* TESTPF_H_ */
