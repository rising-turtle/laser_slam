#include "ZHCanonical_Matcher.h"
#include "ZHPolar_Match.h"
#include "PolarParameter.h"
#include "point.h"
#include <iostream>
#include <limits>
#include <fstream>
#include <string.h>
#include <algorithm>
#include <csm/csm_all.h>
#include <csm/algos.h>
#include <csm/logging.h>
#include <sys/time.h>

#include <Eigen/Core>

using namespace zhpsm; // for point.h

CCanonicalMatcher::CCanonicalMatcher(Base_PARAM* p):
input(new sm_params),
output(new sm_result),
m_pKeyframe(0),
pm_fi(0),
pm_si(0),
pm_co(0),
m_pParam(0)
{
	if(p!=NULL){
		// cout<<"before pm_init()!"<<endl;
		m_pParam = new Base_PARAM(*p);
		pm_init();
		// cout<<"I am here!"<<endl;
		}
	if(m_pParam == 0){
		// cout<<"what happened!?"<<endl;
	}
	// cout<<"Address of m_pParam: "<<m_pParam<<endl;
	sm_init(m_pParam);
	// sm_debug_write(true);

	// threshold init
	threshold_angular_ = 5.*(M_PI/180.); // 10 degree
	threshold_dist_sq_ = 0.02*0.02; //2 cm
}
CCanonicalMatcher::~CCanonicalMatcher(){
	for(int i=0;i<m_Scans.size();i++){
		ld_free(m_Scans[i]);
		m_Scans[i] =0;
	}
	delete input;
	delete output;
	delete m_pParam;
	delete []pm_fi;
	delete []pm_si;
	delete []pm_co;
}

void CCanonicalMatcher::setThreshold(float dist, float angle){
	threshold_angular_ = angle;
	threshold_dist_sq_ = dist*dist;
}

void CCanonicalMatcher::pm_init(){
	if(pm_fi == 0){
		pm_fi = new float[m_pParam->pm_l_points];
		pm_si = new float[m_pParam->pm_l_points];
		pm_co = new float[m_pParam->pm_l_points];
	}

	for(int i=0;i<m_pParam->pm_l_points;i++)
	{
		pm_fi[i] = ((float)i*m_pParam->pm_dfi + m_pParam
		->pm_fi_min);
		pm_si[i] = sinf(pm_fi[i]);
		pm_co[i] = cosf(pm_fi[i]);
	}
}

void CCanonicalMatcher::setCov(bool model){
	input->do_compute_covariance = model;
}

void CCanonicalMatcher::sm_init(Base_PARAM* p)
{
	if( p == 0){
		// default params
		input->min_reading = 0;
		input->max_reading = 1000;
	}
	else{
		// Param in PSM is [cm] while in CSM [m]
		input->min_reading = p->pm_min_range/100. ;
		input->max_reading = p->pm_max_range/100.;
	}
	// these parameters are from ROS scan_tools
	input->max_angular_correction_deg = 45; // Maximum angular displacement between scans
	input->max_linear_correction =  0.5; 	// Maximum translation between scans (m)

	input->max_iterations = 10; //50 ;// 1000; // Maximum ICP cycle iterations
	input->epsilon_xy = 0.001; //0.000001; 	// A threshold for stopping (m)
	input->epsilon_theta = 0.001; //0.000001;// A threshold for stopping (rad)

	input->max_correspondence_dist = 2;//0.3; // Maximum distance for a correspondence to be valid
	input->sigma = 0.04;//0.01;// 0.010; 	// Noise in the scan (m)

	input->use_corr_tricks = 1; // Use smart tricks for finding correspondences.
	input->restart = 1;//0;	// Restart: Restart if error is over threshold
	input->restart_threshold_mean_error = 0.01; // Restart: Threshold for restarting
	input->restart_dt = 0.01;//1.0; // Restart: displacement for restarting. (m)
	input->restart_dtheta = 0.0261799;//0.1;// Restart: displacement for restarting. (rad)
	input->clustering_threshold = 0.05;//0.25; // Max distance for staying in the same clustering
	input->orientation_neighbourhood = 10; //3;//20; // Number of neighbour rays used to estimate the orientation
	input->use_point_to_line_distance = 1; // If 0, it's vanilla ICP

	input->do_alpha_test = 0; // Discard correspondences based on the angles
	input->do_alpha_test_thresholdDeg = 20.0; // Discard correspondences based on the angles - threshold angle, in degrees
	
	input->outliers_maxPerc = 0.95;//0.90; // Percentage of correspondences to consider: if 0.9,
	// always discard the top 10% of correspondences with more error

	// Parameters describing a simple adaptive algorithm for discarding.
	//  1) Order the errors.
	//	2) Choose the percentile according to outliers_adaptive_order.
	//	   (if it is 0.7, get the 70% percentile)
	//	3) Define an adaptive threshold multiplying outliers_adaptive_mult
	//	   with the value of the error at the chosen percentile.
	//	4) Discard correspondences over the threshold.
	//	This is useful to be conservative; yet remove the biggest errors.
	input->outliers_adaptive_order = 0.7;
	input->outliers_adaptive_mult = 2.0;
	
	//If you already have a guess of the solution, you can compute the polar angle
	//	of the points of one scan in the new position. If the polar angle is not a monotone
	//	function of the readings index, it means that the surface is not visible in the 
	//	next position. If it is not visible, then we don't use it for matching.
	input->do_visibility_test = 0;

	// no two points in laser_sens can have the same corr.
	input->outliers_remove_doubles = 1;

	// If 1, computes the covariance of ICP using the method http://purl.org/censi/2006/icpcov
	input->do_compute_covariance = 0;
	// input->do_compute_covariance = 1;


	// Checks that find_correspondences_tricks gives the right answer
	input->debug_verify_tricks = 0;

	// If 1, the field 'true_alpha' (or 'alpha') in the first scan is used to compute the 
	// incidence beta, and the factor (1/cos^2(beta)) used to weight the correspondence.");
	input->use_ml_weights = 0;

	// If 1, the field 'readings_sigma' in the second scan is used to weight the 
	// correspondence by 1/sigma^2
	input->use_sigma_weights = 0;

}

void CCanonicalMatcher::median_filter ( PMScan *ls )
{
	const int HALF_WINDOW  = 2;//2 to left 2 to right
	const int WINDOW = 2*HALF_WINDOW+1;
	float   r[WINDOW];
	float   w;

	int i,j,k,l;

	for ( i=0;i<this->m_pParam->pm_l_points;i++ )
	{
		k=0;
		for ( j=i-HALF_WINDOW;j<=i+HALF_WINDOW;j++ )
		{
			l = ( ( j>=0 ) ?j:0 );
			r[k]=ls->r[ ( ( l < m_pParam->pm_l_points ) ?l: ( m_pParam->pm_l_points-1 ) ) ];
			k++;
		}
		//bubble sort r
		for ( j= ( WINDOW-1 );j>0;j-- )
			for ( k=0;k<j;k++ )
				if ( r[k]>r[k+1] ) // wrong order? - swap them
				{
					w=r[k];
					r[k]=r[k+1];
					r[k+1] = w;
				}
		ls->r[i] = r[HALF_WINDOW];//choose the middle point
	}
	// cout<<"finish YADEDEDE!"<<endl;
}


// translate PSM_Scan 2 LDP
void CCanonicalMatcher::fromPMScan2LDP(PMScan* pm_scan, LDP* ldp, Base_PARAM* param)
{
	unsigned int n = pm_scan->np;
	*ldp = ld_alloc_new(n);
	
	for(unsigned int i=0;i<n;i++){
		// calculate position in laser frame
		double r = pm_scan->r[i];
		if(r<= param->pm_min_range || r>= param->pm_max_range)
		{
			(*ldp)->valid[i] = 0;
			(*ldp)->readings[i] = -1; 
		}else{
			(*ldp)->valid[i] = 1;
			(*ldp)->readings[i] = r/100.0;
		}
		// theta
		(*ldp)->theta[i] = param->pm_fi_min + param->pm_dfi*i;
		(*ldp)->cluster[i] = -1;
	}
	(*ldp)->min_theta = (*ldp)->theta[0];
	(*ldp)->max_theta = (*ldp)->theta[n-1];

	(*ldp)->odometry[0] = 0.0;
	(*ldp)->odometry[1] = 0.0;
	(*ldp)->odometry[2] = 0.0;
	
	(*ldp)->estimate[0] = 0.0;
	(*ldp)->estimate[1] = 0.0;
	(*ldp)->estimate[2] = 0.0;
	
	(*ldp)->true_pose[0] = 0.0;
	(*ldp)->true_pose[1] = 0.0;
	(*ldp)->true_pose[2] = 0.0;
}

float CCanonicalMatcher::FMatch(PMScan* lsr, PMScan* lsa, bool bFirst)
{
	float ret;
	LDP ldpref,ldpcur;
	if(bFirst){
		median_filter(lsr);	
		bFirst = false;
	}
	median_filter(lsa);
	fromPMScan2LDP(lsr,&ldpref,m_pParam);
	fromPMScan2LDP(lsa,&ldpcur,m_pParam);
	ret = FMatch(ldpref,ldpcur);
	
	lsa->rx = ldpcur->true_pose[0]*100.; // x 
	lsa->ry = ldpcur->true_pose[1]*100.; // y
	lsa->th = ldpcur->true_pose[2];

	ld_free(ldpref);
	ld_free(ldpcur);
	return ret;
}

float CCanonicalMatcher::FMatch(LDP lsr, LDP lsa)
{
	float ret = 0;
	lsr->odometry[0] = 0.0;
	lsr->odometry[1] = 0.0;
	lsr->odometry[2] = 0.0;

	lsr->estimate[0] = 0.0;
	lsr->estimate[1] = 0.0;
	lsr->estimate[2] = 0.0;

	lsr->true_pose[0] = 0.0;
	lsr->true_pose[1] = 0.0;
	lsr->true_pose[2] = 0.0;

	input->laser_ref = lsr;
	input->laser_sens = lsa;

	// TODO: use Odo or velocity
	input->first_guess[0] = 0.0;
	input->first_guess[1] = 0.0;
	input->first_guess[2] = 0.0;

	sm_icp(input,output);
	if(output->valid) // succeed
	{
		for(int i=0;i<3;i++)
			lsa->true_pose[i] = output->x[i];
	}
	else{
		cout<<"icp_failed!"<<endl;
		ret = -1;
	}
	return ret;
}

float CCanonicalMatcher::FMatchKeyFrame2(PMScan* lsa, Eigen::Matrix3d& cor)
{
	float ret = FMatchKeyFrame(lsa);
	if(ret < 0 || ret == 2) // failed or first frame
		return ret;
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
		{
			cor(i,j) = gsl_matrix_get(output->cov_x_m,i,j);
		}
	return ret;
}

float CCanonicalMatcher::FMatchKeyFrame(PMScan* lsa){
	float ret;
	LDP ldpcur;
	median_filter(lsa);
	fromPMScan2LDP(lsa,&ldpcur,m_pParam);
	ret = FMatchKeyFrame(ldpcur);
	
	lsa->rx = ldpcur->true_pose[0]*100.; // x 
	lsa->ry = ldpcur->true_pose[1]*100.; // y
	lsa->th = ldpcur->true_pose[2];
	/*if(ret!=2) // keyframe is not assigned
		ld_free(ldpcur);*/
	return ret;
}

float CCanonicalMatcher::FMatchKeyFrame(LDP lsa)
{
	float ret = 0;
	if(m_pKeyframe == 0) {
		m_pKeyframe = lsa;
		return 2; // first laser data
	}
	m_pKeyframe->odometry[0] = 0.0;
	m_pKeyframe->odometry[1] = 0.0;
	m_pKeyframe->odometry[2] = 0.0;

	m_pKeyframe->estimate[0] = 0.0;
	m_pKeyframe->estimate[1] = 0.0;
	m_pKeyframe->estimate[2] = 0.0;

	m_pKeyframe->true_pose[0] = 0.0;
	m_pKeyframe->true_pose[1] = 0.0;
	m_pKeyframe->true_pose[2] = 0.0;

	input->laser_ref = m_pKeyframe;
	input->laser_sens = lsa;

	// TODO: use Odo or velocity
	input->first_guess[0] = 0.0;
	input->first_guess[1] = 0.0;
	input->first_guess[2] = 0.0;
	
	// struct timeval st,et;
	// gettimeofday(&st,0);
	sm_icp(input,output);
	// gettimeofday(&et,0);
	// cout<<"CSM icp cost: "<<(et.tv_sec-st.tv_sec)*1000+(et.tv_usec-st.tv_usec)/1000<<endl;

	if(output->valid) // succeed
	{
		for(int i=0;i<3;i++)
			lsa->true_pose[i] = output->x[i];
		// ld_free(m_pKeyframe);
		// m_pKeyframe = lsa;

		// whether to swap the key frame
		/*if(newKeyframeNeeded(lsa->true_pose)){
			// generate a keyframe
			ld_free(m_pKeyframe);
			m_pKeyframe = lsa;
			return 2;
		}*/
	}
	else{
		cout<<"icp_failed!"<<endl;
		ret = -1;
	}
	ld_free(m_pKeyframe);
	m_pKeyframe = lsa;
	return ret;
}
void CCanonicalMatcher::resetKeyFrame(PMScan* lsa)
{
	LDP ldpcur;
	fromPMScan2LDP(lsa,&ldpcur,m_pParam);
	ld_free(m_pKeyframe);
	m_pKeyframe = ldpcur;
}

bool CCanonicalMatcher::newKeyframeNeeded(double p[3])
{
	float x = p[0];
	float y = p[1];
	float th = p[2];
	if(th > threshold_angular_) return true; // angle threshold
	if(x*x + y*y > threshold_dist_sq_) return true; // dist threshold
	return false;
} 

void CCanonicalMatcher::runlogImproved(string file, int num)
{
	ifstream inf(file.c_str());
	if(!inf.is_open()){
		cout<<"failed to open file: "<<file<<endl;
		return ;
	}
	char line[4096];
	int cnt=0;
	PMScan* ls = new PMScan(m_pParam->pm_l_points);
	PMScan* la = new PMScan(m_pParam->pm_l_points);
	double timestamp;
	int num_points , offset;
	string delim(",");
	while(inf.getline(line,4096) && cnt<num){
		char* saveptr;
		cnt++;
		// construct PSM scan
		timestamp = atof(strtok_r(line,delim.c_str(),&saveptr));
		num_points = atoi(strtok_r(NULL,delim.c_str(),&saveptr));
		offset = atoi(strtok_r(NULL,delim.c_str(),&saveptr));
		for(int i=0;i<num_points;i++){
			ls->r[i] = atof(strtok_r(NULL,delim.c_str(),&saveptr))*100.0;
			ls->x[i] = ls->r[i]*pm_co[i];
			ls->y[i] = ls->r[i]*pm_si[i];
		}
		if(cnt==1){
			*la = *ls;
			continue;
		}
	}
}
bool CCanonicalMatcher::constructPSM(char* line, PMScan& scan, Base_PARAM* p)
{
	double timestamp;
	int num_points , offset;
	string delim(",");
	char* saveptr;
	// construct PSM scan
	timestamp = atof(strtok_r(line,delim.c_str(),&saveptr));
	num_points = atoi(strtok_r(NULL,delim.c_str(),&saveptr));
	if(num_points != p->pm_l_points){
		return false;		
	}
	scan.rx = scan.ry = scan.th = 0;
	offset = atoi(strtok_r(NULL,delim.c_str(),&saveptr));
	for(int i=0;i<num_points;i++){
		scan.r[i] = atof(strtok_r(NULL,delim.c_str(),&saveptr))*100.0;
		scan.x[i] = scan.r[i]*pm_co[i];
		scan.y[i] = scan.r[i]*pm_si[i];
		scan.bad[i] = 0;
		if(scan.r[i] < p->pm_min_range)
			scan.r[i] = p->pm_max_range + 1;
	}
	return true;
}

bool CCanonicalMatcher::constructPSM511(char * line, PMScan& scan, Base_PARAM* p){
	string delim(" ");
	char* saveptr;

	string timestamp = strtok_r(line, delim.c_str(),&saveptr);
	unsigned int num_points = atoi(strtok_r(NULL,delim.c_str(),&saveptr));
	if(num_points != p->pm_l_points){
		return false;
	}
	scan.rx=scan.ry=scan.th=0;
	scan.t = atof(timestamp.c_str());
	for(int i=0;i<num_points;i++){
		scan.r[i] = atof(strtok_r(NULL,delim.c_str(),&saveptr))*100.;
		scan.x[i] = scan.r[i]*pm_co[i];
		scan.y[i] = scan.r[i]*pm_si[i];
		scan.bad[i] = 0;
		if(scan.r[i]<p->pm_min_range)
			scan.r[i] = p->pm_max_range + 1;
	}
	return true;
}

void CCanonicalMatcher::testKeyFrameMatch(string ifile, int num)
{
	ifstream inf(ifile.c_str());
	if(!inf.is_open()){
		cout<<"failed to open file: "<<ifile<<endl;
		return;
	}
	// Test dataset from RawSeed 
	CPolarMatch* psm = new CPolarMatch("LMS151");
	m_pParam = new Base_PARAM(*psm->m_pParam);
	pm_init();
	sm_init(m_pParam);

	char line[8192];
	int cnt = 0;
	unsigned int N = num<0?1000000:num;
	OrientedPoint2D lpose_csm,pose_csm,rel_csm;
	
	ofstream fcsm("csm_rawseed_keyframe.log");//"/mnt/hgfs/SharedFold/log/csm/csm_rawseed_keyframe.log");

	PMScan la(m_pParam->pm_l_points);
	Eigen::Matrix3d cor;
	while(inf.getline(line,8192) && cnt<N){
		cnt++;
		// read file	
		if(!constructPSM(line,la,m_pParam))
			continue;
		// if(FMatchKeyFrame(&la)<0)
		if(FMatchKeyFrame2(&la, cor))
		{
			cout<<"CSM failed at: "<<cnt<<endl;
			continue;
		}
		for(int l=0;l<3;l++)
		{
			for(int k=0;k<3;k++)
			{
				cout<<cor(l,k)<<" ";
			}
			cout<<endl;
		}
		rel_csm = OrientedPoint2D(la.rx/100.0,la.ry/100.0,la.th);
		pose_csm = lpose_csm.oplus(rel_csm);
		
		// Thirdly, record their results
		fcsm<<pose_csm<<endl;
		cout<<"Match Result: "<<pose_csm.x<<" "<<pose_csm.y<<" "<<pose_csm.theta<<endl;

		// Finally, turn to next loop
		lpose_csm = pose_csm;
	}
fcsm.close();
// ld_free(lda);
delete psm;
}
void CCanonicalMatcher::compareCSM2PSM_RawSeed(string ifile, int num)
{
	ifstream inf(ifile.c_str());
	if(!inf.is_open()){
		cout<<"failed to open file: "<<ifile<<endl;
		return;
	}
	// Test dataset from RawSeed 

	// CPolarMatch* psm = new CPolarMatch("LMS211"); 
	CPolarMatch* psm = new CPolarMatch("LMS151");
	m_pParam = new Base_PARAM(*psm->m_pParam);
	pm_init();
	sm_init(m_pParam);

	char line[8192];
	int cnt = 0;
	unsigned int N = num<0?1000000:num;
	OrientedPoint2D lpose_csm,pose_csm,rel_csm;
	OrientedPoint2D lpose_psm,pose_psm,rel_psm;
	
	ofstream fcsm("csm_rawseed.log");
	ofstream fpsm("psm_rawseed.log");

	LDP lds;
	LDP lda;
	PMScan ls(m_pParam->pm_l_points);
	PMScan la(m_pParam->pm_l_points);
	float tmp_angle = 0;
	bool bfirst = true;
	while(inf.getline(line,8192) && cnt<N){
		cnt++;
		// read file	
		if(!constructPSM(line,la,m_pParam))
			continue;
		if(bfirst){
			bfirst = false;
			fpsm<<pose_psm<<endl;
			fcsm<<pose_csm<<endl;
			ls = la;
			continue;
		}else{
			// Firstly,  PSM 
			 static __thread bool pfirst = true;
			//static  bool pfirst = true;
			pair<float,float> err = psm->FMatch(&ls,&la,pfirst);
			if(pfirst) {
				fromPMScan2LDP(&ls,&lds,m_pParam);
				pfirst = false;
			}
			if(err.first<0 && err.second<0){
				cout<<"PSM failed at: "<<cnt<<endl;
				continue;
			}
			rel_psm = OrientedPoint2D(la.rx/100.0,la.ry/100.0,la.th);
			pose_psm = lpose_psm.oplus(rel_psm);
			ls = la;
			cout<<"POSE_PSM: "<<pose_psm.theta<<endl;
			cout<<"POSE_PSM: "<<pose_psm.theta<<endl;

			// Secondly, CSM 
			fromPMScan2LDP(&la,&lda,m_pParam);
			// cout<<"before icp, lda has valid: "<<count_equal(lda->valid,lda->nrays,1)<<endl;
			if(FMatch(lds,lda)<0){
				cout<<" CSM failed at: "<<cnt<<endl;
				continue;
			}
			// cout<<"after icp lda has valid: "<<count_equal(lda->valid,lda->nrays,1)<<endl;
			rel_csm = OrientedPoint2D(lda->true_pose[0],lda->true_pose[1],lda->true_pose[2]);
			pose_csm = lpose_csm.oplus(rel_csm);
			tmp_angle += rel_csm.theta;
			//cout<<"POSE_CSM: "<<tmp_angle<<endl;


			// Thirdly, record their results
			fcsm<<pose_csm<<endl;
			fpsm<<pose_psm<<endl;

			// Finally, turn to next loop
			lpose_csm = pose_csm;
			lpose_psm = pose_psm;
			ld_free(lds);
			lds = lda;
		}
	}
	fcsm.close();
	fpsm.close();
	ld_free(lds);
	// ld_free(lda);
	delete psm;
}

void CCanonicalMatcher::compareCSM2PSM_Lenovo(string ifile, int num)
{
	ifstream inf(ifile.c_str());
	if(!inf.is_open()){
		cout<<"failed to open file: "<<ifile<<endl;
		return;
	}

	// Test dataset from Lenovo office
	CPolarMatch* psm = new CPolarMatch("LMS151"); 
	psm->readCarmon(ifile,psm->m_pParam->pm_laser_name);
	m_pParam = new Base_PARAM(*psm->m_pParam);
	pm_init();
	sm_init(m_pParam);
	cout<<"obtain "<<psm->m_SickScans.size()<<" items"<<endl;

	int cnt = 0;
	unsigned int N = num<0?psm->m_SickScans.size()+1:num;
	OrientedPoint2D lpose_csm,pose_csm,rel_csm;
	OrientedPoint2D lpose_psm,pose_psm,rel_psm;
	
	ofstream fcsm("/mnt/hgfs/SharedFold/log/csm/csm_lenovo.log");
	ofstream fpsm("/mnt/hgfs/SharedFold/log/csm/psm_lenovo.log");

	LDP lds;
	LDP lda;
	PMScan* ls;
	PMScan* la;
	bool bfirst = true;
	
	while( cnt < psm->m_SickScans.size() && cnt<N){
		la = psm->m_SickScans[cnt];
		cnt++;
		if(bfirst){
			bfirst = false;
			fpsm<<pose_psm<<endl;
			fcsm<<pose_csm<<endl;
			ls = la;
			continue;
		}else{
			// Firstly,  PSM 
			 static __thread bool pfirst = true;
			//static bool pfirst = true;
			pair<float,float> err = psm->FMatch(ls,la,pfirst);
			if(pfirst) {
				fromPMScan2LDP(ls,&lds,m_pParam);
				pfirst = false;
			}
			if(err.first<0 && err.second<0){
				cout<<"PSM failed at: "<<cnt<<endl;
				continue;
			}
			rel_psm = OrientedPoint2D(la->rx/100.0,la->ry/100.0,la->th);
			pose_psm = lpose_psm.oplus(rel_psm);
			ls = la;
			cout<<"POSE_PSM: "<<pose_psm.theta<<endl;

			// Secondly, CSM 
			fromPMScan2LDP(la,&lda,m_pParam);
			// cout<<"before icp, lda has valid: "<<count_equal(lda->valid,lda->nrays,1)<<endl;
			if(FMatch(lds,lda)<0){
				cout<<" CSM failed at: "<<cnt<<endl;
				continue;
			}
			// cout<<"after icp lda has valid: "<<count_equal(lda->valid,lda->nrays,1)<<endl;
			rel_csm = OrientedPoint2D(lda->true_pose[0],lda->true_pose[1],lda->true_pose[2]);
			pose_csm = lpose_csm.oplus(rel_csm);
			
			// Thirdly, record their results
			fcsm<<pose_csm<<endl;
			fpsm<<pose_psm<<endl;

			// Finally, turn to next loop
			lpose_csm = pose_csm;
			lpose_psm = pose_psm;
			ld_free(lds);
			lds = lda;
		}
	}
	fcsm.close();
	fpsm.close();
	ld_free(lds);
	// ld_free(lda);
	delete psm;
}

void CCanonicalMatcher::compareCSM2PSM_511(string ifile, int num)
{
	ifstream inf(ifile.c_str());
	if(!inf.is_open()){
		cout<<"failed to open file: "<<ifile<<endl;
		return;
	}
	// Test dataset from RawSeed 
	CPolarMatch* psm = new CPolarMatch("LMS511"); 
	m_pParam = new Base_PARAM(*psm->m_pParam);
	pm_init();
	sm_init(m_pParam);

	char line[8192];
	int cnt = 0;
	unsigned int N = num<0?1000000:num;
	OrientedPoint2D lpose_csm,pose_csm,rel_csm;
	OrientedPoint2D lpose_psm,pose_psm,rel_psm;
	
	ofstream fcsm("/mnt/hgfs/SharedFold/log/csm/csm_511.log");
	ofstream fpsm("/mnt/hgfs/SharedFold/log/csm/psm_511.log");

	LDP lds;
	LDP lda;
	PMScan ls(m_pParam->pm_l_points);
	PMScan la(m_pParam->pm_l_points);
	bool bfirst = true;
	while(inf.getline(line,8192) && cnt<N){
		cnt++;
		// read file	
		if(!constructPSM511(line,la,m_pParam))
			continue;
		if(bfirst){
			bfirst = false;
			fpsm<<pose_psm<<endl;
			fcsm<<pose_csm<<endl;
			ls = la;
			continue;
		}else{
			// Firstly,  PSM 
			static __thread bool pfirst = true;
			//static bool pfirst = true;
			pair<float,float> err = psm->FMatch(&ls,&la,pfirst);
			if(pfirst) {
				fromPMScan2LDP(&ls,&lds,m_pParam);
				pfirst = false;
			}
			if(err.first<0 && err.second<0){
				cout<<"PSM failed at: "<<cnt<<endl;
				continue;
			}
			rel_psm = OrientedPoint2D(la.rx/100.0,la.ry/100.0,la.th);
			pose_psm = lpose_psm.oplus(rel_psm);
			ls = la;

			// Secondly, CSM 
			fromPMScan2LDP(&la,&lda,m_pParam);
			// cout<<"before icp, lda has valid: "<<count_equal(lda->valid,lda->nrays,1)<<endl;
			if(FMatch(lds,lda)<0){
				cout<<" CSM failed at: "<<cnt<<endl;
				continue;
			}
			// cout<<"after icp lda has valid: "<<count_equal(lda->valid,lda->nrays,1)<<endl;
			rel_csm = OrientedPoint2D(lda->true_pose[0],lda->true_pose[1],lda->true_pose[2]);
			pose_csm = lpose_csm.oplus(rel_csm);
			
			// Thirdly, record their results
			fcsm<<pose_csm<<endl;
			fpsm<<pose_psm<<endl;

			// Finally, turn to next loop
			lpose_csm = pose_csm;
			lpose_psm = pose_psm;
			ld_free(lds);
			lds = lda;
		}
	}
	fcsm.close();
	fpsm.close();
	ld_free(lds);
	// ld_free(lda);
	delete psm;
}
