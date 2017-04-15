#include "particles.h"
#include "VPmap.h"
#include "point.h"
#include "ZHPolar_Match.h"
#include "globaldef.h"
#include <fstream>
#include "localization.h"

#define NO_CHECK_CORRIDOR FALSE

using namespace std;

namespace{
	const double rad10 = 0.1745329252;
	const double sigma_x = g_sigma_x; 
	const double sigma_y = g_sigma_y; 
	const double sigma_th = g_sigma_th; // 10 deg 
}

CParticles::CParticles(unsigned int nParticles, \
	unsigned int nL):
	m_nParticles(nParticles),
	m_ori_pose(new OrientedPoint2D),
	m_mean_pose(new OrientedPoint2D),
	m_nL(nL),
	UD(m_nL,m_nL),
	X_COV(m_nL,m_nL),
	m_PF(new BF::SIR_scheme(m_nL,m_nParticles,m_goodRandom)),
	// m_PF(new BF::SIR_kalman_scheme(m_nL,m_nParticles,m_goodRandom)),
	m_LP(new BF::Sampled_LiAd_predict_model(m_nL,m_nL,m_goodRandom)),
	m_bUseMeanDis(NO_CHECK_CORRIDOR)
{
	// Stationary Prediction Model(Identity)
	FM::identity(m_LP->Fx);
	// Init Cov as Constant Matrix
	FM::identity(X_COV);
	X_COV(0,0) = sigma_x; // 0.5m
	X_COV(1,1) = sigma_y;
	X_COV(2,2) = sigma_th; // 10 degree
	
	// sampling according to X_COV
	FM::Float rcond = FM::UdUfactor(UD,X_COV);
	FM::UdUseperate(m_LP->G,m_LP->q,UD);
	m_LP->init_GqG();

	// Init weight in PF
	m_PF->init_S();
	// m_PF->init();
}

CParticles::~CParticles()
{	
	delete m_LP;
	delete m_PF;
	for(int i=0;i<m_pParticlePose.size();i++)
	{
		delete m_pParticlePose[i];
		m_pParticlePose[i] = 0;
	}
	delete m_ori_pose;
	delete m_mean_pose;
}

void CParticles::setOriginalPose(float * ori_pose)
{
	float *p = ori_pose;
	m_ori_pose->x = *p;
	m_ori_pose->y = *(p+1);
	m_ori_pose->theta = *(p+2);
}

void CParticles::init(float* init_pose, int nL)
{
	if(nL!=m_nL)
	{
		cout<<"particles.cpp: nL!="<<m_nL<<endl;
		return ;
	}
	FM::Vec x_init(nL);
	float * p = init_pose;
	for(int i=0;i<nL;i++,p++)
		x_init[i] = *p;
	OrientedPoint2D iniP(x_init[0],x_init[1],x_init[2]);
	// Samples at mean
	const std::size_t nSamples = m_PF->S.size2();
	for (std::size_t i = 0; i != nSamples; ++i) 
	{
		FM::ColMatrix::Column Si(m_PF->S,i);
		noalias(Si) = x_init;
		m_pParticlePose.push_back(new OrientedPoint2D(iniP));
	}
}	

void CParticles::predict2(OrientedPoint2D& abs_pose)
{
	*m_mean_pose = abs_pose;
	for(int i=0;i<m_nParticles;i++)
	{
		FM::ColMatrix::Column Si(m_PF->S,i);
		// cache[i].x = Si[0]; 
		// cache[i].y = Si[1];
		// cache[i].theta = Si[2];
		
		// *m_pParticlePose[i] = m_pParticlePose[i]->oplus(rel_pose);
		*m_pParticlePose[i] = *m_mean_pose;
		Si[0] = m_pParticlePose[i]->x + m_goodRandom.normal(0,sigma_x);
		Si[1] = m_pParticlePose[i]->y + m_goodRandom.normal(0,sigma_y);
		Si[2] = m_pParticlePose[i]->theta + m_goodRandom.normal(0,sigma_th);
		m_pParticlePose[i]->x = Si[0];
		m_pParticlePose[i]->y = Si[1];
		m_pParticlePose[i]->theta = Si[2];
		// cout<<"particles: "<<i<<" oripose: "<<*m_pParticlePose[i]<<endl;
		// cout<<"particles: aftpose: "<<Si[0]<<" "<<Si[1]<<" "<<Si[2]<<endl;
		// of<<*m_pParticlePose[i]<<endl;
	}

}

void CParticles::predict(OrientedPoint2D& rel_pose)
{
	// static vector<OrientedPoint2D> cache(m_nParticles);
	// update mean_pose;
	*m_mean_pose = m_mean_pose->oplus(rel_pose);
	for(int i=0;i<m_nParticles;i++)
	{
		FM::ColMatrix::Column Si(m_PF->S,i);
		// cache[i].x = Si[0]; 
		// cache[i].y = Si[1];
		// cache[i].theta = Si[2];
		
		// *m_pParticlePose[i] = m_pParticlePose[i]->oplus(rel_pose);
		*m_pParticlePose[i] = *m_mean_pose;
		Si[0] = m_pParticlePose[i]->x + m_goodRandom.normal(0,sigma_x);
		Si[1] = m_pParticlePose[i]->y + m_goodRandom.normal(0,sigma_y);
		Si[2] = m_pParticlePose[i]->theta + m_goodRandom.normal(0,sigma_th);
		m_pParticlePose[i]->x = Si[0];
		m_pParticlePose[i]->y = Si[1];
		m_pParticlePose[i]->theta = Si[2];
		// cout<<"particles: "<<i<<" oripose: "<<*m_pParticlePose[i]<<endl;
		// cout<<"particles: aftpose: "<<Si[0]<<" "<<Si[1]<<" "<<Si[2]<<endl;
		// of<<*m_pParticlePose[i]<<endl;
	}
	// sampling according to the current covariance 
	// FM::Float rcond = FM::UdUfactor(UD,X_COV);
	// BF::Numberical_rcond rclimit;
	// rclimit.check_PSD(rcond, "X_COV not PSD");

	// FM::UdUseperate(m_LP->G,m_LP->q,UD);
	// m_LP->init_GqG();
	
	// if no need to multiple F 
	// m_PF->predict(*m_LP);
}

double CParticles::neff()
{
	double sum=0;
	// normalize the weights pf the particles
	for(int i=0;i<m_nParticles;i++){
		sum += m_PF->wir[i];
	}
	double cum=1;
	double w;
	for (int i=0;i<m_nParticles;i++){
		w=m_PF->wir[i]/sum;
		// m_PF->wir[i] = w;
		cum+=w*w;
	}
	return 1./cum;
	// return 1;
}

bool CParticles::isNoisyParticle(int pi)
{
	if(pi<0 || pi>=m_nParticles)
	{
		cout<<"particles.cpp: no particle: "<<pi<<endl;
		return true;
	}
	OrientedPoint2D& p1 = *m_pParticlePose[pi];
	OrientedPoint2D& p2 = *m_mean_pose;
	double dis = (p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y);
	if(dis >= g_noisy_particle_dis) // no more than 2m
	{
		return true;
	}
	double angle_dis = fabs(p1.theta - p2.theta);
	if(angle_dis >= g_noisy_particle_ang)
	{
		return true;
	}
	return false;
}

/*
void CParticles::weightMean()
{
	double sumW = 0;
	vector<bool> noise_flag(m_nParticles,false);
	for(int i=0;i<m_nParticles;i++)
	{
		if(isNoisyParticle(i))
		{
			noise_flag[i] = true;
			continue;
		}
		sumW += m_PF->wir[i];
	}
	if(sumW <= 0.0001)
	{
		cout<<"particles.cpp: error in weightMean()!"<<endl;
		return ;
	}
	double score;
	double x = 0 ;
	double y = 0 ; 
	double th = 0;
	for(int i=0;i<m_nParticles;i++)
	{
		if(noise_flag[i])
		{
			continue;
		}
		score = m_PF->wir[i]/sumW;
		x += score*m_pParticlePose[i]->x;
		y += score*m_pParticlePose[i]->y;
		th += score*m_pParticlePose[i]->theta;
	}
	*m_mean_pose = OrientedPoint2D(x,y,th);
}*/

namespace{
	double disPose(OrientedPoint2D* p1, OrientedPoint2D* p2)
	{
		return ((p1->x-p2->x)*(p1->x-p2->x) + (p1->y-p2->y)*(p1->y-p2->y));
	}
}

double CParticles::meanDis(vector<int>& index)
{
	vector<float> dis_set;
	double sum = 0;
	unsigned int N = index.size();
	double sum_i;
	for(int i=0;i<N;i++)
	{
		sum_i = 0;
		for(int j=0;j<N;j++)
		{
			sum_i += disPose(m_pParticlePose[index[j]],m_pParticlePose[index[i]]);
		}
		sum_i /= N;
		sum += sum_i;
	}
	return (sum/N);
} 

double CParticles::weightMean(vector<int>& index, vector<float>& weight)
{
	double meanKW = 0;
	double sumW = 0;
	for(int i=0;i<index.size();i++)
	{
		sumW += weight[index[i]];
	}
	meanKW = sumW/index.size();
	// cout<<"particles.cpp: mean weight: "<<sumW/index.size()<<endl;
	double score; 
	double x = 0;
	double y = 0; 
	double th = 0;
	for(int i=0;i<index.size();i++)
	{
		score = weight[index[i]]/sumW;
		x += score*m_pParticlePose[index[i]]->x;
		y += score*m_pParticlePose[index[i]]->y;
		th += score*m_pParticlePose[index[i]]->theta;
	}
	*m_mean_pose = OrientedPoint2D(x,y,th);
	return meanKW;
}

void CParticles::calMaxWeight()
{
	double maxW = 0.000001;
	m_maxIndexParticle = -1;
	for(int i=0;i<m_nParticles;i++)
	{
		if(m_PF->wir[i] > maxW)
		{
			maxW = m_PF->wir[i];
			m_maxIndexParticle = i;
		}
	}
	if(m_maxIndexParticle < 0)
	{
		cout<<"particles.cpp: error in calMaxWeight!"<<endl;
		return ;
	}
	// cout<<"particles.cpp: max weight: "<<maxW<<" with pose: "<<*m_pParticlePose[m_maxIndexParticle];
}

void CParticles::translate2RealPose(OrientedPoint2D& p, OrientedPoint2D& mp)
{
	p.x = mp.x - m_ori_pose->x ;
	p.y = m_ori_pose->y - mp.y;
	p.theta = mp.theta - M_PI/2.;
}

void CParticles::translate2MapPose(OrientedPoint2D& p ,OrientedPoint2D& mp)
{
	// mp.x = m_ori_pose->x + p.x;//m_pParticlePose[i].x;
	// mp.y = m_ori_pose->y + -1.*p.y;//-1.*m_pParticlePose[i].y;
	// mp.theta = p.theta + M_PI/2.;
	
	mp.x = m_ori_pose->x - p.y ;
	mp.y = m_ori_pose->y - p.x ; 
	mp.theta = p.theta + M_PI ;
}

bool CParticles::update(CVPmap* map, PMScan* ls)
{
	FM::Vec z(m_nParticles);
	OrientedPoint2D mp;
	vector<int> index(m_nParticles);
	vector<float> scores(m_nParticles);
	OrientedPoint2D rel_pose;
	for(int i=0;i<m_nParticles;i++)
	{
		/*mp.x = m_ori_pose->x + m_pParticlePose[i].x;
		mp.y = m_ori_pose->y + -1.*m_pParticlePose[i].y;
		mp.theta = m_pParticlePose[i].theta + M_PI/2.;*/
		translate2MapPose(*m_pParticlePose[i], mp);
		// z[i] = map->obsLikelyhood(ls, *m_pParticlePose[i]);
		scores[i] = z[i] = map->obsLikelyhood(ls, mp, rel_pose);
		*m_pParticlePose[i] = m_pParticlePose[i]->oplus(rel_pose);
		// translate2RealPose(*m_pParticlePose[i], mp);
		index[i] = i;
	}
	//cout<<"LIKELIHOOD: "<<z<<endl;
	// if(neff)
	m_PF->observe_likelihood(z);
	calMaxWeight();
	
	((threadLocalization*)0)->topK(index,scores,g_num_of_weight_particles);
	double meandis = meanDis(index);
	double goodness = weightMean(index,scores);
	// cout<<"particles.cpp: meandis: "<<meandis<<" goodness: "<<goodness<<endl;

	if(neff() < m_nParticles*g_resample_threshold)
	{
		// cout<<"particles: resampling occur!"<<endl;
		m_PF->update_resample();
	}
	else
	{
		// m_PF->mean();
	}

	// for initial scans, threshold is a little bit loose
	static int cnt = 0;
	if(++cnt <= 4)
	{
		if(goodness < 0.48) 
		{
			cout<<"particles.cpp: initial scan goodness: "<<goodness<<" ignored!"<<endl;
			return true;
		}else if(m_bUseMeanDis && meandis >= 0.08)
		{
			cout<<"particles.cpp: initial scan goodness: "<<meandis<<" ignored!"<<endl;
			return true;
		}
	}else
	{
		// cout<<"particles.cpp: Hi, I am here!"<<endl;
		if(goodness < g_goodness_of_localization)
		{
			cout<<"particles.cpp : goodness: "<<goodness<<" ignored!"<<endl;
			return true;
		}else if(m_bUseMeanDis && meandis >= g_meandis_of_localization)
		{
			cout<<"particles.cpp : meandis: "<<meandis<< "ignored!"<<endl;
			return true;
		}
	}
	return false;
}

void CParticles::getMean(OrientedPoint2D& p)
{
	// weightMean();
	translate2MapPose(*m_mean_pose,p);
}

OrientedPoint2D* CParticles::getMean2()
{
	return m_mean_pose;
}

void CParticles::getMax(OrientedPoint2D& p)
{
	if(m_maxIndexParticle >= 0)
	{
		// p = *m_pParticlePose[m_maxIndexParticle];
		translate2MapPose(*m_pParticlePose[m_maxIndexParticle],p);
	}else{
		cout<<"particles.cpp: no max particle is calculated!"<<endl;
	}
	// p.x = m_PF->x[0];
	// p.y = m_PF->x[1];
	// p.theta = m_PF->x[2];
}


