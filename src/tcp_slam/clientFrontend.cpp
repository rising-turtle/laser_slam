#include <QThread>
#include <fstream>
#include <QMutexLocker>

#include "clientFrontend.h"
#include "clientFusion.h"
#include "ZHPolar_Match.h"
#include "ZHCanonical_Matcher.h"
#include "point.h"

// matrix UD etc.
#include "matSup.hpp"


// sick_reader
#include "CSICK.h"
#include "CObs2DScan.h"
#include <Eigen/Core>
// #include <sys/wait.h>

namespace{
	ofstream feout("fepose.log");
	ofstream upout("uppose.log");
	ofstream gtout("gtpose.log");
	ofstream ukf_cov("ukf_cov.log");
	ofstream sfout("scanframe.log");
	
	// For UDUFactor
	static int sx_dim = 3;
	static FM::SymMatrix COV(sx_dim,sx_dim);
	static FM::Matrix UD(sx_dim,sx_dim);
	static FM::Matrix G(sx_dim,sx_dim);

	static Vec q(sx_dim);


	void assign(Eigen::Matrix3d& from, FM::SymMatrix& to)
	{	
		for(int i=0;i<sx_dim;i++)
		for(int j=0;j<sx_dim;j++)
		{
			to(i,j) = from(i,j);
		}
	}
	void assign(double m[6], FM::SymMatrix& cov){
	cov(0,0) = m[0]; cov(0,1) = cov(1,0) = m[1];  cov(0,2) = cov(2,0) = m[2];
			 cov(1,1) = m[3]; 	      cov(1,2) = cov(2,1) = m[4]; 
			 		  	      cov(2,2) = m[5];
	}
}
using namespace zhpsm;

CClientFrontend::CClientFrontend(bool useCSM,QObject* parent):
						m_bCSM(useCSM),
						work_model(0),
						m_stop_thread(false),
						m_syn_num(0),
						m_pPSM(0),
						m_pCSM(0),
						m_pFusion(0),
						recv_t(0),
						x_dim(3),
						use_UKF(true)
{
	// for run
	bFirst_run = true;
	matrix = (double *)malloc(6 * sizeof(double));
	scnt = 0;
	bSetScnt = false;
	
	// for carmon reading
	bFirst_carmon = true;
	sent_pose = new OrientedPoint2D;
	
	// for UKF
	m_pUKF = new Unscented_scheme(x_dim);
	// Setup the initial state and covariance
	Vec x_init(x_dim);
	SymMatrix X_init(x_dim,x_dim);
	for(int i=0;i<x_dim;i++)
		x_init[i] = 0;
	X_init = ublas::zero_matrix<double>(x_dim,x_dim);
	m_pUKF->init_kalman(x_init,X_init);
	sick_observe = new SICK_observe;
	robot_predict = new Robot_predict;
	synGlobal_observe = new SynGlobal_observe;
	
	// for corridor matrix
	memset(corridor_m,0,sizeof(double)*6);
	corridor_m[0] = 0.5; // corridor covariance along x-axis
	corridor_m[3] = 0.5; // corridor covariance along y-axis

#ifdef USE_COV
	sent_cov.fill(0.);
	curr_cov.fill(0.);
	rel_cov.fill(0.);
	H_cov.fill(0.); H_cov(0,0) = 1; H_cov(1,1) = 1; H_cov(2,2) = 1;
	R_cov.fill(0.); R_cov(0,0) = 0.02; R_cov(1,1) = 0.02; R_cov(2,2) = 0.6;
	Q_cov.fill(0.); Q_cov(0,0) = 0.001*0.001; Q_cov(1,1) = 0.001*0.001; Q_cov(2,2) = 0;
	I_cov.fill(0.); I_cov(0,0) = 1; I_cov(1,1) = 1; I_cov(2,2) = 1;
#endif
}

CClientFrontend::~CClientFrontend()
{
	for(int i=0;i<m_traj.size();i++){
		delete m_traj[i];
		m_traj[i] = 0;
	}

 	if(m_pFusion) delete m_pFusion;
 	if(matrix) delete matrix;
	if(m_pUKF) delete m_pUKF;
	if(sick_observe) delete sick_observe;
	if(robot_predict) delete robot_predict;
}

// 0 RawSeed 1 Carmon 2 SICK 3 fusion
void CClientFrontend::setModel(int model){
	work_model = model;
}
void CClientFrontend::setFile(string file){
	m_file = file;
}
void CClientFrontend::setSICKIP(vector<string> ip, vector<unsigned int> port)
{
	sick_ip = ip;
	sick_port = port;
}
void CClientFrontend::recvTime(TTimeStamp t)
{
	QMutex lock(&mutex_recv_t);
	//printf("recvTime = %lld \n", t);
	recv_t = t;
}
void CClientFrontend::setFusion()
{
	// must init m_pFusion here before building the connections
	m_pFusion = new CClientFusion;
	m_pFusion->setSICKIP(sick_ip, sick_port);
	m_pFusion->setModel(work_model); // still require setModel since frontendSICK need to know this;
	
}

void CClientFrontend::runFrontEnd(){
	switch(work_model){
	case 0:
		cout<<"runRawSeed()!"<<endl;
		runRawSeed();
		break;
	case 1:
		cout<<"runCarmon()!"<<endl;
		runCarmon();
		break;
	case 2:
		cout<<"runSICK()!"<<endl;
		runSICK(sick_ip[0], sick_port[0]);
		break;
	case 3:
		cout<<"runFusion()!"<<endl;
		use_UKF = false; // frontend stop to use ukf
		m_pFusion->runFusion();
		break;
	default:
		cout<<"unknown work model!"<<endl;
		throw 1;
		break;
	}

	cout<<"ThreadLocal1: runFrontEnd() "<<(int)QThread::currentThreadId()<<" quit!" <<endl;
	finished();
	return ;
}

void CClientFrontend::fromRel2AbsPose(OrientedPoint2D& cor, OrientedPoint2D& rel, OrientedPoint2D& abs)
{
	float fcos = cos(cor.theta);
	float fsin = sin(cor.theta);
	abs.x = rel.x * fcos - rel.y * fsin;
	abs.y = rel.x * fsin + rel.y * fcos;
	abs.theta = rel.theta;
}

void CClientFrontend::receUpdatePose(int id, void* pose, double* cov)
{
	OrientedPoint2D* p = static_cast<OrientedPoint2D*>(pose);
	if(p==NULL) return ;
	if(id < 0 || id >= traj_index.size()){
		cout<<"error in CClientFrontend::receUpdatePose!"<<endl;
		return ;
	}
	cout<<"receUpdatePose from Server!"<<endl;
	{
		QMutexLocker locker(&m_mutex);
	
		OrientedPoint2D* last = m_traj[m_traj.size()-1];
		OrientedPoint2D* prev = m_traj[traj_index[id]];
		OrientedPoint2D trans = prev->ominus(*last);
		OrientedPoint2D changed = *prev + *p;
		*last = changed.oplus(trans);
		// OrientedPoint2D trans_abs;
		// fromRel2AbsPose(*prev, trans, trans_abs);		
		// *prev = *prev + *p;
		// cout<<"trans is: "<<trans<<endl;
		// cout<<"before trans: "<<*last<<endl;
		// *last = *last + *p;
		if(use_UKF){
			double cov_Global[6];
			memcpy(cov_Global,cov,sizeof(double)*6);
			cout<<"before UKF, cov: "<<m_pUKF->X(0,0)<<" "<<m_pUKF->X(1,1)<<" "<<m_pUKF->X(2,2)<<endl;
			
			// UD seperate
			assign(cov_Global,COV);
			FM::Float rcond = FM::UdUfactor(UD, COV);
			Bayesian_filter::Numerical_rcond rclimit;
			rclimit.check_PSD(rcond, "Init curr_cov not PSD");
			FM::UdUseperate(G, q, UD);

			synGlobal_observe->Zv[0] = q(0); // *100.;  // cov_Global[0]*100.;
			synGlobal_observe->Zv[1] = q(1); // *100.;  // cov_Global[3]*100.;
			synGlobal_observe->Zv[2] = q(2);       //rad2deg(cov_Global[5]);
			
			/*
			synGlobal_observe->Zv[0] = cov_Global[0];
			synGlobal_observe->Zv[1] = cov_Global[3];
			synGlobal_observe->Zv[2] = rad2deg(cov_Global[5]);
			*/

			Vec z(3);
			((CClientFusion*)0)->cvtOPD2X(*last,z);
			m_pUKF->observe(*synGlobal_observe,z);
			cout<<"after UKF, cov: "<<m_pUKF->X(0,0)<<" "<<m_pUKF->X(1,1)<<" "<<m_pUKF->X(2,2)<<endl;
	
			// update the observation covariance
			/*curr_cov(0,0) = m_pUKF->X(0,0)/100.;
			curr_cov(1,1) = m_pUKF->X(1,1)/100.;*/
			
			curr_cov(0,0) = m_pUKF->X(0,0);
			curr_cov(1,1) = m_pUKF->X(1,1);
			curr_cov(2,2) = m_pUKF->X(2,2); //deg2rad(m_pUKF->X(2,2));
			curr_cov(1,0) = curr_cov(0,1) = curr_cov(2,0) = curr_cov(0,2) = curr_cov(2,1) = curr_cov(1,2) = 0;
		}
		cout<<"after trans: "<<*last<<endl;
		upout<<id<<" "<<*p<<endl;
		m_syn_num ++;
	}
}

void CClientFrontend::getEigen(Eigen::Matrix3d& cov, double m[6])
{
	m[0] = cov(0,0); m[1] = cov(0,1); m[2] = cov(0,2);
			 m[3] = cov(1,1); m[4] = cov(1,2); 
			 		  m[5] = cov(2,2);
}
void CClientFrontend::addMatrix(double m[6], double a[6])
{
	for(int i=0;i<6;i++)
		m[i]+=a[i];
}

void CClientFrontend::addEigen(Eigen::Matrix3d& cov, double a[6]){
	cov(0,0) += a[0]; cov(1,1) += a[3]; cov(2,2) += a[5];
}


void CClientFrontend::run(PMScan& ls){

	static ofstream fout("tcp_out.txt");
	TTimeStamp ts = getCurrentTime();

	OrientedPoint2D prev_pose;
	OrientedPoint2D rel_pose;
	OrientedPoint2D curr_pose;
	OrientedPoint2D gt_pose(ls.rx,ls.ry,ls.th);
	int syn_num = 0;

	static int nFailedCnt = 0;
	// use CSM or PSM
	if(m_bCSM){
	scnt++;
	#ifdef USE_COV
		float ret = m_pCSM->FMatchKeyFrame2(&ls,rel_cov);
	#else
		float ret = m_pCSM->FMatchKeyFrame(&ls);
	#endif
		if(ret < 0 )
		{
			cout<<"CClientFrontend::runSICK: CSM failed : "<<++nFailedCnt<<endl;
			return ;
		}
	}else{ // PSM

	}
	if(bFirst_run){
		bFirst_run = false;
		m_traj.push_back(new OrientedPoint2D(curr_pose));
	#ifdef USE_COV
		m_pCSM->setCov(true);
		getEigen(curr_cov,matrix);
		sendFrameInfoCov(curr_pose.x,curr_pose.y,curr_pose.theta,ls.r,ls.np,syn_num,matrix);
	#else
		m_pCSM->setCov(false);
		sendFrameInfo(curr_pose.x,curr_pose.y,curr_pose.theta,ls.r,ls.np,syn_num);
	#endif
		// send2CBScan(&ls.r[0],ls.np,curr_pose.x,curr_pose.y,curr_pose.theta);
		// send2CBPose(curr_pose.x,curr_pose.y,curr_pose.theta);

		traj_index.push_back(m_traj.size()-1);
		return ;
	}

	rel_pose = OrientedPoint2D(ls.rx/100.,ls.ry/100.,ls.th);
#ifdef USE_COV
	curr_cov = curr_cov + rel_cov; // + Q_cov;
	/*if(use_UKF){
		curr_cov(0,0) = m_pUKF->X(0,0)/100.;
		curr_cov(1,1) = m_pUKF->X(1,1)/100.;
		curr_cov(2,2) = deg2rad(m_pUKF->X(2,2));
	}*/
	// curr_cov = curr_cov + rel_cov; // + Q_cov;
	// Eigen::Matrix3d tmpM = curr_cov + rel_cov + Q_cov;
	// K_cov = curr_cov*tmpM.inverse();
	// curr_cov = (I_cov-K_cov)*curr_cov;

	getEigen(curr_cov,matrix);
#else
	//matrix
#endif
	// after updating the covariance
	if(!validMove(rel_pose)){
		return ;
	}

	if(work_model!=3)
	{
		/*if(notMove(rel_pose)) {
			cout<<"pose: "<<scnt<<" not move!"<<endl;
			not_move<<"pose: "<<scnt<<" not move!"<<endl;
			return ;
		}*/

		{
			QMutexLocker locker(&m_mutex);
			prev_pose = *m_traj[m_traj.size()-1];
			syn_num = m_syn_num;
			// record trjectory
			curr_pose = prev_pose.oplus(rel_pose);
			if(use_UKF){
				Vec z(3);
				((CClientFusion*)0)->cvtOPD2X(curr_pose,z);
			#ifdef USE_COV
				// UD seperate
				assign(curr_cov, COV);
				FM::Float rcond = FM::UdUfactor(UD, COV);
				Bayesian_filter::Numerical_rcond rclimit;
				rclimit.check_PSD(rcond, "Init curr_cov not PSD");
				FM::UdUseperate(G, q, UD);

				sick_observe->Zv[0] =  q(0);// curr_cov(0,0)*100;
				sick_observe->Zv[1] =  q(1);// curr_cov(1,1)*100;
				sick_observe->Zv[2] =  q(2); // rad2deg(curr_cov(2,2));
				
				/*
				sick_observe->Zv[0] =  curr_cov(0,0);
				sick_observe->Zv[1] =  curr_cov(1,1);
				sick_observe->Zv[2] =  rad2deg(curr_cov(2,2));
				*/
				
			#endif
				m_pUKF->predict(*robot_predict);
				((CClientFusion*)0)->cvtX2OPD(m_pUKF->x, prev_pose);
				m_pUKF->observe(*sick_observe,z);
				((CClientFusion*)0)->cvtX2OPD(m_pUKF->x, curr_pose);
				ukf_cov<<m_pUKF->X(0,0)/100.<<" "<<m_pUKF->X(1,1)/100.<<" "<<deg2rad((double)m_pUKF->X(2,2))<<endl;
			}
			m_traj.push_back(new OrientedPoint2D(curr_pose));
		}
		// cout<<m_traj.size()<<" "<<curr_pose.x<<" "<<curr_pose.y<<" "<<curr_pose.theta<<endl;
		// m_pCSM->resetKeyFrame(&ls);
		// feout<<curr_pose.x<<" "<<curr_pose.y<<" "<<curr_pose.theta<<endl;
		// gtout<<gt_pose.x<<" "<<gt_pose.y<<" "<<gt_pose.theta<<endl;
		// send to sever
		// cout<<"sendFrameInfo x: "<<curr_pose.x<<" y: "<<curr_pose.y<<" th: "<<curr_pose.theta<<endl;
		// sendFrameInfo(curr_pose.x,curr_pose.y,curr_pose.theta,ls.r,ls.np);

		rel_pose = sent_pose->ominus(curr_pose);

		if((!smallMove(rel_pose)) || (bSetScnt && scnt>=5))
		{
			if( scnt >=5 ){
				m_pPSM->pm_segment_scan(&ls);
				// if the robot is not moving
				if(!m_pPSM->pm_is_corridor(&ls)){
					scnt = 0;
					return ;
				}
				else{ // this scan is taken in the corridor
					addMatrix(matrix, corridor_m);
					addEigen(curr_cov, corridor_m);
					cout<<"clientFrontend: in corridor, covariance: "<<matrix[0]<<" "<<matrix[3]<<" "<<matrix[5]<<endl;
				}
			}

			#ifdef USE_COV
				sendFrameInfoCov(curr_pose.x,curr_pose.y,curr_pose.theta,ls.r,ls.np,syn_num,matrix);
			#else
				sendFrameInfo(curr_pose.x,curr_pose.y,curr_pose.theta,ls.r,ls.np,syn_num);
			#endif
			
			*sent_pose = curr_pose;
			// m_traj.push_back(new OrientedPoint2D(curr_pose));
			traj_index.push_back(m_traj.size()-1);
			// cout<<" "<<curr_pose.x<<" "<<curr_pose.y<<" "<<curr_pose.theta<<endl;
			feout<<curr_pose.x<<" "<<curr_pose.y<<" "<<curr_pose.theta<<endl;
			sfout<<scnt<<endl;
			// gtout<<gt_pose.x<<" "<<gt_pose.y<<" "<<gt_pose.theta<<endl;
			// m_pCSM->resetKeyFrame(&ls);
			bSetScnt = true;
			scnt = 0;
			usleep(10000);

			TTimeStamp te = getCurrentTime();
			fout<<timeDifference(ts, te)<<" "<<curr_pose.x<<" "<<curr_pose.y<<" "<<curr_pose.theta<<endl;
		}
	}
	else
	{
		// Fusion mode

		if((!smallMove(rel_pose)) || (bSetScnt && scnt>=5))
		{
			if( scnt >=5 ){
				m_pPSM->pm_segment_scan(&ls);
				// if the robot is not moving
				if(!m_pPSM->pm_is_corridor(&ls)){
					scnt = 0;
					return ;
				}
				else{ // this scan is taken in the corridor
					addMatrix(matrix, corridor_m);
					addEigen(curr_cov, corridor_m);
					cout<<"clientFrontend: in corridor, covariance: "<<matrix[0]<<" "<<matrix[3]<<" "<<matrix[5]<<endl;
				}
			}

			TTimeStamp t = getCurrentTime();
			sendRelFrameInfo(rel_pose.x,rel_pose.y,rel_pose.theta,ls.r,ls.np,t, syn_num,matrix);
			//cout<<"ClientFrontend::run "<<" "<<rel_pose.x<<" "<<rel_pose.y<<" "<<rel_pose.theta<<" "<<ls.np<<" "<<ls.t<<" "<<t<<endl;

			bSetScnt = true;
			scnt = 0;
			/*
			while(1)
			{
				QMutex lock(&mutex_recv_t);
				if(recv_t != t)
				{
					QThread::yieldCurrentThread();
					continue;
				}
				else
					break;
			}
			*/
		}
	}
}

void CClientFrontend::runSICK(string ip, unsigned int port){
	cout<<"CClientFrontend::runSICK() "<<ip<<" "<<port<<endl;

	// single SICK situation

	m_pPSM = new CPolarMatch("LMS151");
	m_pCSM = new CCanonicalMatcher(m_pPSM->m_pParam);
	PMScan ls(m_pPSM->m_pParam->pm_l_points);

	CSICK laser(ip, port);
	CObs2DScan outObs;
	bool isOutObs, hardwareError;
	if(!laser.turnOn()){
		cout<<"CClientFrontend::runSICK: failed to turnOn SICK!"<<endl;
		return ;
	}
	cout<<"CClientFrontend::runSICK: succeed to turnOn SICK!"<<endl;
	laser.doProcessSimple(isOutObs, outObs, hardwareError);

	while(!m_stop_thread){
		laser.doProcessSimple(isOutObs, outObs, hardwareError);
		if(hardwareError)
		{
			cout<<"CClientFrontend::runSICK: "<<"Error with SICK reading! But continue!"<<endl;
			continue;
		}
		// TODO:

		if(!constructPSMFromSICK(outObs, m_pPSM, ls))
		{
			cout<<"CClientFrontend::runSICK: "<<"Error with constructPSMfromSICK! But continue!"<<endl;
			continue;
		}
		run(ls);
	}
	cout<<"CClientFrontend::runSICK: !!!!!! SLAM LOCAL EXIT!"<<endl;
	return ;
}

void CClientFrontend::runfile(){
	PMScan ls(m_pPSM->m_pParam->pm_l_points);
	ifstream inf(m_file.c_str());
	if(!inf.is_open()){
		cout<<"in runCarmon() failed to open file: "<<m_file<<endl;
		return ;
	}
	char line[8192];
	int cnt = 0;
	while(!inf.eof() && inf.getline(line,8192) && !m_stop_thread)
	{
		// if(!constructPSMfromCarmon(line,ls,m_pPSM))
		if(!constructPSMfromRawSeed(line,ls,m_pPSM))
		{
			cout<<"clientFrontend frame "<<	++cnt<<" invalid!"<<endl;
			continue;
		}
		cnt++;
		// if(cnt>800) break;
		run(ls);
	}
	cout<<"quit runfile()!"<<endl;
}

// run our own Carmon-style LMS151 data
void CClientFrontend::runCarmon(){
	cout<<"ThreadLocal1: runCarmon() "<<(int)QThread::currentThreadId()<<endl;
	m_pPSM = new CPolarMatch("LMS151");
	// m_pPSM = new CPolarMatch("LMS211");
	// m_pPSM = new CPolarMatch("LMS511");
	m_pCSM = new CCanonicalMatcher(m_pPSM->m_pParam);
	runfile();
	return ;
}

void CClientFrontend::runRawSeed()
{
	cout<<"ThreadLocal1: runRawseed() "<<(int)QThread::currentThreadId()<<endl;
	m_pPSM = new CPolarMatch("LMS211");
	m_pCSM = new CCanonicalMatcher(m_pPSM->m_pParam);
	runfile();
	return ;
}

bool CClientFrontend::constructPSMfromCarmon(char* line, PMScan& ls, CPolarMatch* m_pPSM){
	ls.rx=ls.ry=ls.th=0;
	char* last;
	string delim(" ");
	double start, fov, resolution, maxRange, accuracy;
	int laserType, remissionMode, num_points;
	string tag = strtok_r(line,delim.c_str(),&last);
	if(strcmp(tag.c_str(),"ROBOTLASER1")){
		return false;
	}
	laserType = atoi(strtok_r(NULL,delim.c_str(),&last));
	// laserType = atoi(strtok_r(line,delim.c_str(),&last));
	start = atof(strtok_r(NULL,delim.c_str(),&last));
	fov = atof(strtok_r(NULL,delim.c_str(),&last));
	resolution = atof(strtok_r(NULL,delim.c_str(),&last));
	maxRange = atof(strtok_r(NULL,delim.c_str(),&last));
	accuracy = atof(strtok_r(NULL,delim.c_str(),&last));
	remissionMode = atoi(strtok_r(NULL,delim.c_str(),&last));
	num_points = atoi(strtok_r(NULL,delim.c_str(),&last));
	if(num_points != m_pPSM->m_pParam->pm_l_points && \
		num_points != m_pPSM->m_pParam->pm_l_points-1)
		return false;
	// set parameters
	if(bFirst_carmon){
		bFirst_carmon = false;
		m_pPSM->m_pParam->pm_fi_min = start;
		m_pPSM->m_pParam->pm_fi_max = start + fov*PM_D2R;
		m_pPSM->pm_init();
		if(maxRange*100 != m_pPSM->m_pParam->pm_max_range)
			m_pPSM->m_pParam->pm_max_range = maxRange*100;
	}
	// read bearings
	for(int i=0;i<num_points;i++){
		ls.r[i] = atof(strtok_r(NULL,delim.c_str(),&last))*100.0; // from [m] 2 [cm]
		ls.x[i] = ls.r[i] * m_pPSM->pm_co[i];
		ls.y[i] = ls.r[i] * m_pPSM->pm_si[i];
		ls.bad[i] = 0;
		if(ls.r[i]<m_pPSM->m_pParam->pm_min_range){
			ls.r[i]=m_pPSM->m_pParam->pm_max_range+1; //set it to a value larger than the max
			ls.bad[i] = 1;
		}
	}
	if(num_points == m_pPSM->m_pParam->pm_l_points-1){
		ls.r[num_points] = m_pPSM->m_pParam->pm_max_range+1;
		ls.x[num_points] = 0;
		ls.y[num_points] = 0;
		ls.bad[num_points] = 1;
	}
	// remission data
	strtok_r(NULL,delim.c_str(),&last);
	// laser pose
	ls.rx = atof(strtok_r(NULL,delim.c_str(),&last));
	ls.ry = atof(strtok_r(NULL,delim.c_str(),&last));
	ls.th = atof(strtok_r(NULL,delim.c_str(),&last));

	return true;
}

bool CClientFrontend::constructPSMFromSICK(const CObs2DScan obsScan, CPolarMatch* psm, PMScan& ls){

	if(obsScan.scan.size()!=psm->m_pParam->pm_l_points)
		return false;

	ls.rx=ls.ry=ls.th=0; // its position must be 0,0,0
	ls.t = obsScan.timestamp;

	for(int i=0;i<psm->m_pParam->pm_l_points;i++){
		ls.r[i] = obsScan.scan[i]*100.0; // PSM using cm
		ls.x[i] = ls.r[i] * psm->pm_co[i];
		ls.y[i] = ls.r[i] * psm->pm_si[i];
		ls.bad[i] = 0;
		if(ls.r[i] < PM_MIN_RANGE || ls.r[i] < 0 || ls.r[i] > psm->m_pParam->pm_max_range){
			ls.r[i] = psm->m_pParam->pm_max_range+ 1;
			ls.bad[i] = 1;
		}
	}
	return true;
}


bool CClientFrontend::constructPSMfromRawSeed(char *line, PMScan& ls, CPolarMatch* m_pPSM)
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

		if(m_pPSM->m_pParam->pm_l_points==num_points + 1)
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
	
	char* p;

	for(int i=0;i<num_points;i++){
		p = strtok_r(NULL,delim.c_str(),&ptr);
		if(!p) return false;
		ls.r[i] = atof(p)*100.;
		// ls.r[i] = atof(strtok_r(NULL,delim.c_str(),&ptr))*100.0; // from [m] 2 [cm]
		ls.x[i] = ls.r[i] * m_pPSM->pm_co[i];
		ls.y[i] = ls.r[i] * m_pPSM->pm_si[i];
		ls.bad[i] = 0;
		if(ls.r[i] <= m_pPSM->m_pParam->pm_min_range){
			ls.r[i] = m_pPSM->m_pParam->pm_max_range+1; //set it to a value larger than the max
			ls.bad[i] = 1;
		}
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

bool CClientFrontend::notMove(OrientedPoint2D& pose)
{
#define MOTION_THRE 0.0001 
#define ANGLE_THRE 0.0001 
	if(fabs(pose.x)+ fabs(pose.y) >= MOTION_THRE)
		return false;
	if(fabs(pose.theta) >= ANGLE_THRE)
		return false;
	return true;
}
bool CClientFrontend::smallMove(OrientedPoint2D& pose)
{
// 0.05*0.05 & 5'
#define SMALL_MOTION 0.02*0.02  
#define SMALL_ANGLE 2*PM_D2R
// #define SMALL_MOTION 0.05*0.05  
// #define SMALL_ANGLE 5*PM_D2R

	if(pose.x*pose.x + pose.y*pose.y > SMALL_MOTION)
		return false;
	if(pose.theta > SMALL_ANGLE)
		return false;
	return true;
}

bool CClientFrontend::validMove(OrientedPoint2D& pose)
{
// max speed / pfs
// 10 m/s / 50 =  0.2
#define BIG_MOTION 0.2*0.2
#define BIG_ANGLE 10*PM_D2R
	if(pose.x*pose.x + pose.y*pose.y >= BIG_MOTION)
		return false;
	if(fabs(pose.theta) >= BIG_ANGLE)
		return false;
	return true;
}

void CClientFrontend::stop(){
	m_stop_thread = true;
}

