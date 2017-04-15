#ifndef PARTICLES_H
#define PARTICLES_H

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

// for random distribution
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>

using namespace SLAM_filter;
using namespace std;

class BF::SIR_scheme;
class BF::SIR_kalman_scheme;
class OrientedPoint2D;
class CVPmap;
class OrientedPoint2D;
struct _PMScan;
/*
class CNormal_random{
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
public:
	boost::mt19937 eng1;
	
};
*/

class SLAM_random : public Bayesian_filter_test::Boost_random, public BF::SIR_random
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
	void normal (FM::DenseVec& v, const FM::Float mean, const FM::Float sigma)
	{
		Boost_random::normal (v,mean,sigma);
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


class CParticles{
public:
	CParticles(unsigned int nParticls=100,unsigned int nL=3);
	~CParticles();
	void init(float* init_pose, int nL); // init particles
	void setOriginalPose(float* ori_pose); // set original pose
	void predict(OrientedPoint2D& );
	void predict2(OrientedPoint2D& );
	double neff();
	bool update(CVPmap*, struct _PMScan*);
	void translate2RealPose(OrientedPoint2D&, OrientedPoint2D&);
	void translate2MapPose(OrientedPoint2D&, OrientedPoint2D&);
	void getMax(OrientedPoint2D&);
	void getMean(OrientedPoint2D&);
	OrientedPoint2D* getMean2();
	void calMaxWeight();
	double weightMean(vector<int>&, vector<float>&);
	double meanDis(vector<int>&);
	bool m_bUseMeanDis; // whether use mean dis of the particles to filter local result
	int m_maxIndexParticle; // particle with max Weight
	unsigned int m_nParticles; // num of particles
	unsigned int m_nL; // location vector for each particle
	SLAM_random m_goodRandom; // normal distribution
	BF::SIR_scheme* m_PF;	// particle filter 
	// BF::SIR_kalman_scheme* m_PF;    // 
	BF::Sampled_LiAd_predict_model* m_LP; // location predtiction
	
	FM::Matrix UD; // UD decorrelate state noise
	FM::SymMatrix X_COV; // current covariance
	
	bool isNoisyParticle(int );
	vector<OrientedPoint2D*> m_pParticlePose;
	OrientedPoint2D* m_ori_pose;
	OrientedPoint2D* m_mean_pose;
};

#endif

