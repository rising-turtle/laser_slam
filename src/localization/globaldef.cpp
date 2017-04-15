#include "globaldef.h"
#include "PolarParameter.h"
#include "ZHPolar_Match.h"
#include "string.h"
#include <fstream>

const float g_pixResolution = 0.02; //0.0194; // 19.4 mm
const float g_free_threshold = 0.3; // threshold for free cell
const float g_occu_threshold = 0.6; // threshold for occu cell
const float g_unknown_value = 0.5; // unknown place

const float g_maxCorrDistance = 1.0;// 0.1; // max distance between correspondence
const unsigned int g_scan_decimation = 3; // decimation in likelihood calculation

const int g_num_of_global_particles = 10000;
const int g_num_of_particles = 60; // 50
const int g_num_of_weight_particles = 8; // 6
const double g_goodness_of_localization = 0.6;
const double g_meandis_of_localization = 0.05; 
const float g_resample_threshold = 0.5; // threshold 

const float g_threshold_resample = 0.5; // threadhold for resample
const float g_noisy_particle_dis = 6.25; // 2.5m
const float g_noisy_particle_ang = M_PI/2.; // 90 degree

const double rad10 = 0.1745329252;
const double rad30 = M_PI/6.;
const double g_sigma_x = 0.25; // 0.3 m
const double g_sigma_y = 0.25; // 0.3 m 
const double g_sigma_th = rad30*rad30;//rad30*rad30;

bool constructPSMfromRawSeed(char *line, PMScan& ls, CPolarMatch* m_pPSM)
{
	double timestamp; //must be double
	int num_points;
	int offset;
	char* ptr;
	string delim(" ,");

	timestamp = atof(strtok_r(line,delim.c_str(),&ptr));
	num_points = atoi(strtok_r(NULL,delim.c_str(),&ptr));
	offset = atoi(strtok_r(NULL,delim.c_str(),&ptr));

	// printf("CClientFrontend::constructPSMfromRawSeed: t=%.7f, pm_l_points=%d, offset=%d \n", timestamp,num_points,offset);

	if(m_pPSM->m_pParam->pm_l_points!=num_points){

		if(m_pPSM->m_pParam->pm_l_points== (num_points + 1))
		{}
		else
		{
			cout<<"num_points: "<<num_points<<" PARAM: "<<m_pPSM->m_pParam->pm_l_points<<endl;
			cout<<"not enough points in file!"<<endl;
			return false;
		}
	}
	ls.rx=ls.ry=ls.th=0; // its position must be 0,0,0
	ls.t = timestamp;
	ls.np = m_pPSM->m_pParam->pm_l_points;
	char* p;

	for(int i=0;i<num_points;i++)
	{
		p = strtok_r(NULL,delim.c_str(),&ptr);
		if(!p) return false;
		ls.r[i] = atof(p)*100.;
		// ls.r[i] = atof(strtok_r(NULL,delim.c_str(),&ptr))*100.0; // from [m] 2 [cm]
		ls.bad[i] = 0;
		if(ls.r[i] <= m_pPSM->m_pParam->pm_min_range)
		{
			ls.r[i] = 0; //m_pPSM->m_pParam->pm_max_range+1; //set it to a value larger than the max
			ls.bad[i] = 1;
		}
		else if(ls.r[i] >= m_pPSM->m_pParam->pm_max_range)
		{
			ls.r[i] = m_pPSM->m_pParam->pm_max_range+1;
			ls.bad[i] = 1;
		}
		ls.x[i] = ls.r[i] * m_pPSM->pm_co[i];
		ls.y[i] = ls.r[i] * m_pPSM->pm_si[i];
	}
	if(m_pPSM->m_pParam->pm_l_points==num_points + 1)
	{
		ls.r[num_points] =  m_pPSM->m_pParam->pm_max_range+1;
		ls.x[num_points] = 0;
		ls.y[num_points] = 0;
		ls.bad[num_points] = 1;
	}

	return true;
}

void cm2m(PMScan* ls)
{
	for(int i=0;i<ls->np;i++)
	{
		ls->r[i]*=0.01;
		ls->x[i]*=0.01;
		ls->y[i]*=0.01;
	}
}

void m2cm(PMScan* ls)
{
	for(int i=0;i<ls->np;i++)
	{
		ls->r[i]*=100;
		ls->x[i]*=100;
		ls->y[i]*=100;
	}
}
