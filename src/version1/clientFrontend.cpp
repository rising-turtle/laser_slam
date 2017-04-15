#include <QThread>
#include <fstream>
#include <QMutexLocker>

#include "clientFrontend.h"
#include "ZHPolar_Match.h"
#include "ZHCanonical_Matcher.h"
#include "point.h"
#include "clientFusion.h"

// matrix UD etc.
#include "matSup.hpp"

// sick_reader
#include "CSICK.h"
#include "CObs2DScan.h"

#include <sys/time.h>

using namespace zhpsm;

namespace{

	// For UDUFactor
	static  int sx_dim = 3;
	static  FM::SymMatrix COV(sx_dim,sx_dim);
	static  FM::Matrix UD(sx_dim,sx_dim);
	static  FM::Matrix G(sx_dim,sx_dim);

	static  Vec q(sx_dim);

	void assign(Eigen::Matrix3d& from, FM::SymMatrix& to)
	{
		for(int i=0;i<sx_dim;i++)
		for(int j=0;j<sx_dim;j++)
		{
			to(i,j) = from(i,j);
		}
	}
	inline void assign(double m[6], FM::SymMatrix& cov){
	cov(0,0) = m[0]; cov(0,1) = cov(1,0) = m[1];  cov(0,2) = cov(2,0) = m[2];
			 cov(1,1) = m[3]; 	      cov(1,2) = cov(2,1) = m[4];
			 		  	      cov(2,2) = m[5];
	}

//	ofstream time_log("time.log");
}


CClientFrontend::CClientFrontend(bool useCSM,QObject* parent):
	m_bCSM(useCSM),
	work_model(0),
	m_stop_thread(false),
	m_bSent2Localization(false),
	m_syn_num(0),
	m_pPSM(0),
	m_pCSM(0),
	m_pFusion(0),
	x_dim(3),
	absOdo_cur(new OrientedPoint2D),
	absOdo_last(new OrientedPoint2D),
	absBn_cur(new OrientedPoint2D),
	use_UKF(1) //1

{

	_SCAN_READY = false;
	m_scan = new CObs2DScan;
	m_scan->scan.resize(541);

	// for run
	bFirst_run = 1;
	matrix = (double*)malloc(6*sizeof(double));
	scnt = 0;
	nmovecnt = 0;
	bSetScnt = 0;
	sent_pose = new OrientedPoint2D;
	
	// for UKF
	m_pUKF = new Unscented_scheme(x_dim);
	// Set up the initial state and covariance
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
	corridor_m[0] = 0.5; 
	corridor_m[3] = 0.5;

	curr_cov.fill(0.);
	curr_cov(0,0) = curr_cov(1,1) = curr_cov(2,2) = 1;
	
	rel_cov.fill(0.);
	// rel_cov(0,0) = rel_cov(1,1) = rel_cov(2,2) = 1.;
}

CClientFrontend::~CClientFrontend()
{
	for(int i=0;i<m_traj.size();i++){
		delete m_traj[i];
		m_traj[i] = 0;
	}

 	if(m_pFusion) delete m_pFusion;
	if(sent_pose) delete sent_pose;
	if(matrix) free(matrix);
	if(m_pUKF) delete m_pUKF;
	if(sick_observe) delete sick_observe;
	if(robot_predict) delete robot_predict;
	if(synGlobal_observe) delete synGlobal_observe;
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
void CClientFrontend::setFusion()
{
	// must init m_pFusion here before building the connections
	m_pFusion = new CClientFusion;
	m_pFusion->setSICKIP(sick_ip, sick_port);
	m_pFusion->setModel(work_model); // still require setModel since frontendSICK need to know this;
	// initialization
	if(!m_pFusion->initFusion())
	{
		cout<<"CClientFusion "<<" initial failed!"<<endl;
		m_pFusion->finished();
		return ;
	}

	
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
		cout<<"runSICK2()!"<<endl;
		//runSICK(sick_ip[0], sick_port[0]);
		runSICK2();
		break;
	case 3:
		cout<<"runFusionInfo()!"<<endl;
		use_UKF = false;
		m_pFusion->runFusionInfo(); //uif for fusion; odo first
		//m_pFusion->runFusionPara(); //uif for fusion; parallel
		break;
	case 4:
		cout<<"runODO"<<endl;
		m_pFusion->runODO();
		break;
	case 5:
		cout<<"runBN"<<endl;
		m_pFusion->runBN();
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
/*
void CClientFrontend::receUpdatePose(int id, void* pose)
{
	OrientedPoint2D* p = static_cast<OrientedPoint2D*>(pose);
	if(p==NULL) return ;
	if(id < 0 || id >= traj_index.size()){
		cout<<"error in CClientFrontend::receUpdatePose!"<<endl;
		return ;
	}
	cout<<"receUpdatePose from Server: "<<*p<<endl;
	if(m_stop_thread)
		return ;
	{
		QMutexLocker locker(&m_mutex);
		OrientedPoint2D* last = m_traj[m_traj.size()-1];
		OrientedPoint2D* prev = m_traj[traj_index[id]];
		OrientedPoint2D trans = prev->ominus(*last);
		OrientedPoint2D trans_abs;
		fromRel2AbsPose(*prev, trans, trans_abs);		
		*prev = *prev + *p;
		// cout<<"trans is: "<<trans<<endl;
		cout<<"before trans: "<<*last<<endl;
		*last = *last + *p;
		cout<<"after trans: "<<*last<<endl;
		m_syn_num ++;
	}
}
*/

void CClientFrontend::receUpdatePose2(int id, void* pose, bool bIgnored)
{
	OrientedPoint2D* p = static_cast<OrientedPoint2D*>(pose);
	if(p==NULL) return ;
	if(id < 0 || id >= m_traj.size()){
		cout<<"error in CClientFrontend::receUpdatePose!"<<endl;
		return ;
	}
	//cout<<"CClientFusion::receUpdatePose2 " <<id<<" "<<*p<<endl;
	{
		if(!bIgnored)
		{
			QMutexLocker locker(&m_mutex);
			OrientedPoint2D* last = m_traj[m_traj.size()-1];
			OrientedPoint2D* prev = m_traj[id];
			// OrientedPoint2D* prev = m_traj[id+1];

			OrientedPoint2D trans = prev->ominus(*last);
			OrientedPoint2D changed = prev->ominus(*p);
			// cout<<"receUpdatePose from Localization: trans: "<<changed<<endl;
			// cout<<"id: "<<id<<" total: "<<m_traj.size()<<endl;
			*last = p->oplus(trans);//changed.oplus(trans);
		}
		m_bSent2Localization = false;
	}
}

void CClientFrontend::receUpdatePose(int id, void* pose, double* cov)
{
	OrientedPoint2D* p = static_cast<OrientedPoint2D*>(pose);
	if(p==NULL) return ;
	if(id < 0 || id >= traj_index.size()){
		cout<<"error in CClientFrontend::receUpdatePose!"<<endl;
		return ;
	}
	// cout<<"receUpdatePose from Backend!"<<endl;
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

		double cov_Global[6];
		memcpy(cov_Global,cov,sizeof(double)*6);

		if(use_UKF)
		{
			// cout<<"before UKF, cov: "<<m_pUKF->X(0,0)<<" "<<m_pUKF->X(1,1)<<" "<<m_pUKF->X(2,2)<<endl;
			// UD seperate
			assign(cov_Global,COV);
			FM::Float rcond = FM::UdUfactor(UD, COV);
			Bayesian_filter::Numerical_rcond rclimit;
			rclimit.check_PSD(rcond, "Init curr_cov not PSD");
			FM::UdUseperate(G, q, UD);

			synGlobal_observe->Zv[0] = q(0); // *100.;  // cov_Global[0]*100.;
			synGlobal_observe->Zv[1] = q(1); // *100.;  // cov_Global[3]*100.;
			synGlobal_observe->Zv[2] = q(2);       //rad2deg(cov_Global[5]);

		/*	synGlobal_observe->Zv[0] = cov_Global[0]*100.;
			synGlobal_observe->Zv[1] = cov_Global[3]*100.;
			synGlobal_observe->Zv[2] = rad2deg(cov_Global[5]);*/
			Vec z(3);
			((CClientFusion*)0)->cvtOPD2X(*last,z);
			m_pUKF->observe(*synGlobal_observe,z);
			// cout<<"after UKF, cov: "<<m_pUKF->X(0,0)<<" "<<m_pUKF->X(1,1)<<" "<<m_pUKF->X(2,2)<<endl;
	
			// update the observation covariance
			curr_cov(0,0) = m_pUKF->X(0,0);
			curr_cov(1,1) = m_pUKF->X(1,1);
			curr_cov(2,2) = m_pUKF->X(2,2); // deg2rad(m_pUKF->X(2,2));
		}else{
			curr_cov(0,0) = cov_Global[0];
			curr_cov(1,1) = cov_Global[3];
			curr_cov(2,2) = cov_Global[5];
		}
		curr_cov(1,0) = curr_cov(0,1) = curr_cov(2,0) = curr_cov(0,2) = curr_cov(2,1) = curr_cov(1,2) = 0;
		// cout<<"after trans: "<<*last<<endl;
		// upout<<id<<" "<<*p<<endl;
		m_syn_num ++;
	}
}

inline void CClientFrontend::getEigen(Eigen::Matrix3d& cov, double m[6])
{
	m[0] = cov(0,0); m[1] = cov(0,1); m[2] = cov(0,2);
			 m[3] = cov(1,1); m[4] = cov(1,2); 
			 		  m[5] = cov(2,2);
}

inline void CClientFrontend::addMatrix(double m[6], double a[6])
{
	for(int i=0;i<6;i++)
		m[i]+=a[i];
}

inline void CClientFrontend::addMatrix(double m[6], double a[6], int scale)
{
	for(int i=0;i<6;i++)
		m[i]+=scale*a[i];
}

inline void CClientFrontend::addEigen(Eigen::Matrix3d& cov, double a[6]){
	cov(0,0) += a[0]; cov(1,1) += a[3]; cov(2,2) += a[5];
}


void CClientFrontend::run(PMScan& ls){
	OrientedPoint2D prev_pose;
	OrientedPoint2D curr_pose;
	OrientedPoint2D rel_pose;
	
	// static OrientedPoint2D sent_pose;
	// static bool bfirst = true;

	int syn_num = 0;
	bool bSent2Localization = false;
	m_bGoodMatch = true;
	static bool firstSyn = true;
	// copy the original scans
	//vector<float> tmpls(ls.np);
	//tmpls.insert(tmpls.begin(),ls.r,ls.r+ls.np);
	// use CSM or PSM
	
	// static TTimeStamp ts;
	// static TTimeStamp es;
	
	// ts = getCurrentTime();

	if(m_bCSM){
		scnt++;
		// float ret = m_pCSM->FMatchKeyFrame2(&ls, rel_cov);
		float ret = m_pCSM->FMatchKeyFrame(&ls);
		if(ret < 0 )
		{
			cout<<"CClientFrontend::runSICK: CSM failed! But continue!"<<endl;
			// scan match failed
			m_bGoodMatch = false;
			return ;
		}
	}else{ // PSM

	}
	
	// ts = getCurrentTime();

	if(bFirst_run)
	{
		bFirst_run = false;
		m_traj.push_back(new OrientedPoint2D(curr_pose));
		// m_pCSM->setCov(true); 
		getEigen(curr_cov,matrix);

		sendScanFrameCov((void*)(&ls), syn_num, matrix); // send to Backend
		
		sendFirstFrame((void*)(&ls)); // send to localization to syn coordinates
		//cout<<"sendFirstFrame: "<<ls.t<<" "<<ls.rx<<" "<<ls.ry<<" "<<ls.th<<endl;
		// send to SLAM interface
		send2CBScan(&ls.r[0],ls.np,curr_pose.x,curr_pose.y,curr_pose.theta);
		sendFrameInfo(curr_pose.x,curr_pose.y,curr_pose.theta,ls.r,ls.np); // send to display
		traj_index.push_back(m_traj.size()-1);
		
		// es = getCurrentTime();
		// time_log<<timeDifference(ts,es)*1000<<" ";
		/*
		cout<<"FIRST SCAN: ";
		for(int i=0;i<541;i++)
			cout<<" "<<ls.r[i];
		cout<<endl;
		*/

		if(work_model==3)
		{
			//fusion
			sendFrameInfo(curr_pose.x,curr_pose.y,curr_pose.theta,ls.r,ls.np); // send to display
			//sendRelFrameInfo(0.,0.,0.,ls.r,ls.np,ls.t, matrix,0, true);
			sendRelFrameInfo2(0.,0.,0., (void*)(&ls), 0, m_bGoodMatch);
		}

		return ;
	}

	// result from CSM
	rel_pose = OrientedPoint2D(ls.rx/100.,ls.ry/100.,ls.th);
	// increase the PLICP covariance
	curr_cov = curr_cov + rel_cov;
	getEigen(curr_cov, matrix);

	// filter large trans between successive scans

	rel_pose.theta = normAngle(rel_pose.theta, -M_PI);
	if(!validMove(rel_pose)) 
	{
		cout<<"CLIENTFRONTEND: TOO FAST ! SICK CAN NOT HANDLE"<<endl;
		return ;
	}

	if(work_model!=3)
	{
		//if(notMove(rel_pose)) {return ;}
		{
			QMutexLocker locker(&m_mutex);
			prev_pose = *m_traj[m_traj.size()-1];
			syn_num = m_syn_num;
			bSent2Localization = m_bSent2Localization;
	
			// record trjectory
			curr_pose = prev_pose.oplus(rel_pose);
			if(use_UKF){
				Vec z(3);
				((CClientFusion*)0)->cvtOPD2X(curr_pose,z);
				// UD seperate
				assign(curr_cov, COV);
				FM::Float rcond = FM::UdUfactor(UD, COV);
				Bayesian_filter::Numerical_rcond rclimit;
				rclimit.check_PSD(rcond, "Init curr_cov not PSD");
				FM::UdUseperate(G, q, UD);

				sick_observe->Zv[0] =  q(0);// curr_cov(0,0)*100;
				sick_observe->Zv[1] =  q(1);// curr_cov(1,1)*100;
				sick_observe->Zv[2] =  q(2); // rad2deg(curr_cov(2,2));

			/*	sick_observe->Zv[0] = curr_cov(0,0)*100;
				sick_observe->Zv[1] = curr_cov(1,1)*100;
				sick_observe->Zv[2] = rad2deg(curr_cov(2,2));   */

				m_pUKF->predict(*robot_predict);
				((CClientFusion*)0)->cvtX2OPD(m_pUKF->x, prev_pose);
				m_pUKF->observe(*sick_observe,z);
				((CClientFusion*)0)->cvtX2OPD(m_pUKF->x, curr_pose);
			}
			m_traj.push_back(new OrientedPoint2D(curr_pose));
		}
		
		// fout<<curr_pose.x<<" "<<curr_pose.y<<" "<<curr_pose.theta<<endl;
		// send to sever
		 //cout<<"sendFrameInfo "<<m_traj.size()-1<<" "<<ls.t<<" "<< "x: "<<curr_pose.x<<" y: "<<curr_pose.y<<" th: "<<curr_pose.theta<<endl;
		//sendFrameInfo(curr_pose.x,curr_pose.y,curr_pose.theta,ls.r,ls.np);
		
		rel_pose = sent_pose->ominus(curr_pose);
		curr_pose.theta = normAngle(curr_pose.theta, -M_PI);
		rel_pose.theta = normAngle(rel_pose.theta, -M_PI);

		if((!smallMove(rel_pose)) || (bSetScnt && scnt>=5))
		{
			if(scnt>=5){
				m_pPSM->pm_segment_scan(&ls);
				if(!m_pPSM->pm_is_corridor(&ls)){
					scnt = 0;
					return ;
				}
				else{	// this scan is taken in corridor
					addMatrix(matrix, corridor_m);
					addEigen(curr_cov, corridor_m);
					cout<<"clientFrontend: in corridor, covariance: "<<matrix[0]<<" "<<matrix[3]<<" "<<matrix[5]<<endl;
				}
			}

			ls.rx = curr_pose.x * 100.;
			ls.ry = curr_pose.y * 100.;
			ls.th = curr_pose.theta;

			// sendScanFrameCov((void*)(&ls),syn_num,matrix); // send to Backend
			if(!bSent2Localization)
			{
				sendScanFrame((void*)(&ls),m_traj.size()-1); // send to Localization
				m_bSent2Localization = true; 
			}
			// send to SLAM interface
			send2CBScan((&ls.r[0]),ls.np,curr_pose.x,curr_pose.y,curr_pose.theta);
			sendFrameInfo(curr_pose.x,curr_pose.y,curr_pose.theta,ls.r,ls.np); // send to display
			*sent_pose = curr_pose;
			// m_traj.push_back(new OrientedPoint2D(curr_pose));
			traj_index.push_back(m_traj.size()-1);
			bSetScnt = true;
			scnt = 0;
			//usleep(50000);

			// cout<<"sendFrameInfo: "<<m_traj.size()-1<<" "<<ls.t<< " "<<ls.rx<<" "<<ls.ry<<" "<<ls.th<<endl;
			// m_pCSM->resetKeyFrame(&ls);
		}else if(bSetScnt)
		{

			send2CBScan(&ls.r[0], ls.np, curr_pose.x, curr_pose.y, curr_pose.theta);
		}
		//??????????????????
		//send2CBScan(&ls.r[0], ls.np, curr_pose.x, curr_pose.y, curr_pose.theta);
	}
	else
	{

		// Fusion mode

		getEigen(rel_cov, matrix);
		prev_pose = *m_traj[m_traj.size()-1];
		curr_pose = prev_pose.oplus(rel_pose);
		curr_pose.theta = normAngle(curr_pose.theta, -M_PI);

		m_traj.push_back(new OrientedPoint2D(curr_pose));

		OrientedPoint2D big_rel_pose =sent_pose->ominus(curr_pose) ;
		big_rel_pose.theta = normAngle(big_rel_pose.theta, -M_PI);
		if((!smallMove(big_rel_pose)) || (bSetScnt && scnt>=5))
		{
			if(scnt>=5){
				m_pPSM->pm_segment_scan(&ls);
				if(!m_pPSM->pm_is_corridor(&ls)){
					scnt = 0;
					return ;
				}
				else{	// this scan is taken in corridor
					addMatrix(matrix, corridor_m);
				}
			}

			*sent_pose = curr_pose;
			traj_index.push_back(m_traj.size()-1);
			bSetScnt = true;
			scnt = 0;

			//cout<<"ClientFrontend::curr_pose: "<<m_traj.size()-1<<" "<<ls.t<<" "<<curr_pose.x*100.<<" "<<curr_pose.y*100.<<" "<<curr_pose.theta<<endl;

			//sendRelFrameInfo(big_rel_pose.x,big_rel_pose.y,big_rel_pose.theta,ls.r,ls.np,ls.t, matrix,scnt, m_bGoodMatch);
			//cout<<"AAAAA sendRelFrameInfo2  "<<m_traj.size()-1<<" "<<rel_pose.x<<" "<<rel_pose.y<<" "<<rel_pose.theta<<" "<<scnt<<" "<<m_bGoodMatch<<endl;
			sendRelFrameInfo2(big_rel_pose.x,big_rel_pose.y,big_rel_pose.theta, (void*)(&ls), scnt, m_bGoodMatch);
		}


		//cout<<"sendRelFrameInfo2 "<<m_traj.size()-1<<" "<<rel_pose.x<<" "<<rel_pose.y<<" "<<rel_pose.theta<<" "<<scnt<<" "<<m_bGoodMatch<<endl;
		//sendRelFrameInfo2(rel_pose.x,rel_pose.y,rel_pose.theta, (void*)(&ls), scnt, m_bGoodMatch);

		// sendFrameInfo(curr_pose.x,curr_pose.y,curr_pose.theta,ls.r,ls.np); // send to display

		//usleep(10000);
	}
	// es = getCurrentTime();
	// time_log<<timeDifference(ts,es)*1000<<" ";
}

/*runSICK receive sick frames by itself*/
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
		this->sickError();
		cout<<"CClientFrontend::runSICK: failed to turnOn SICK!"<<endl;
		this->sickCNKError();
		return ;
	}
	cout<<"CClientFrontend::runSICK: succeed to turnOn SICK!"<<endl;
	laser.doProcessSimple(isOutObs, outObs, hardwareError);

	//ofstream fout("runSICK_scan.log");
	while(!m_stop_thread){
		//TTimeStamp ts = getCurrentTime();
		laser.doProcessSimple(isOutObs, outObs, hardwareError);
		if(hardwareError)
		{
			//cout<<"CClientFrontend::runSICK: "<<"Error with SICK reading! But continue!"<<endl;
			continue;
		}
		// TODO:
		// cout<<"before constructPSMFromSICK!"<<endl;

		if(!constructPSMFromSICK(outObs, m_pPSM, ls))
		{
			//cout<<"CClientFrontend::runSICK: "<<"Error with constructPSMfromSICK! But continue!"<<endl;
			continue;
		}

		/*
		fout<<ls.t<<", 541, 0, ";
		for(int i=0;i<541;i++)
		{
			fout<<ls.r[i]<<", ";
		}
		fout<<endl;
		*/

		send2CBScanOnly(ls.r,ls.np);

		// cout<<"before run(ls)"<<endl;
		//TTimeStamp tr = getCurrentTime();
		run(ls);
		// cout<<"after run(ls)"<<endl;
		// TTimeStamp te = getCurrentTime();
	    // cout<<"Time: "<<timeDifference(ts, te)*1000<<"  "<<timeDifference(ts, tr)*1000<<endl;
	}

	cout<<"CClientFrontend::runSICK: !!!!!! SLAM LOCAL EXIT!"<<endl;
	return ;
}

/*runSICK2 receive sick frame from outside by signals*/
void CClientFrontend::runSICK2(){
	cout<<"CClientFrontend::runSICK2() "<<endl;


	// single SICK situation
	m_pPSM = new CPolarMatch("LMS151");
	m_pCSM = new CCanonicalMatcher(m_pPSM->m_pParam);
	PMScan ls(m_pPSM->m_pParam->pm_l_points);
	m_scan->scan.resize(m_pPSM->m_pParam->pm_l_points);

	//ofstream outf("scan_frontend.log");
	//TTimeStamp t_last = 0.;
	while(!m_stop_thread){

		QMutexLocker lock(&m_mutex_scan);
		if(_SCAN_READY)
		{

			/*
			outf<<m_scan->timestamp<<", 541, 0, ";
			for(int i=0;i<541;i++)
			{
				outf<<m_scan->scan[i]<<", ";
			}
			outf<<endl;
			*/


			//cout<<"CClientFrontend::runSICK2()"<<endl;
			if(!constructPSMFromSICK(*m_scan, m_pPSM, ls))
			{
				continue;
			}
			send2CBScanOnly(ls.r,ls.np);
			run(ls);

			//outf<<timeDifference(m_scan->timestamp, t_last)<<endl;
			/*
			outf<<m_scan->timestamp<<", 541, 0, ";
			for(int i=0;i<541;i++)
			{
				outf<<m_scan->scan[i]<<", ";
			}
			outf<<endl;
			*/
			//t_last = m_scan->timestamp;

			_SCAN_READY = false;
		}
		else
		{
			QThread::yieldCurrentThread();
			continue;
		}
	}

	cout<<"CClientFrontend::runSICK2: !!!!!! SLAM LOCAL EXIT!"<<endl;
	return ;
}

void CClientFrontend::receScan(float* scan, uint64_t t){

	//cout<<"CClientFrontend::receScan: "<<m_scan->scan.size()<<" "<<t<<endl;
	QMutexLocker lock(&m_mutex_scan);
	if(m_scan->scan.size()>0)
	{
		memcpy(&(m_scan->scan[0]), scan, m_scan->scan.size()*sizeof(float));
		m_scan->timestamp = t;
		_SCAN_READY = true;
	}

}

void CClientFrontend::runfile(){
	PMScan ls(m_pPSM->m_pParam->pm_l_points);
	ifstream inf(m_file.c_str());
	if(!inf.is_open()){
		cout<<"in runCarmon() failed to open file: "<<m_file<<endl;
		return ;
	}
	// record time consuming
	//ofstream fout("time.log");
	struct timeval st,et;

	char line[8192];
	// TTimeStamp ts, es;
	int step = 0;
	while(!inf.eof() && inf.getline(line,8192) && !m_stop_thread)
	{
		// if(!constructPSMfromCarmon(line,ls,m_pPSM))
		// ts = getCurrentTime();
		if(!constructPSMfromRawSeed(line,ls,m_pPSM))
		{
			continue;
		}
		send2CBScanOnly(ls.r,ls.np);

		//gettimeofday(&st,0);
		run(ls);
		// es = getCurrentTime();
		// cout<<"Time: "<<timeDifference(ts,es)*1000<<" "<<endl;
		// time_log<<timeDifference(ts,es)*1000<<" "<<endl;
		step = 0;
		//gettimeofday(&et,0);
		//fout<<((et.tv_sec - st.tv_sec)*1000 + (et.tv_usec - st.tv_usec)/1000)<<endl;
	}
}

// run our own Carmon-style LMS151 data
void CClientFrontend::runCarmon(){
	cout<<"ThreadLocal1: runCarmon() "<<(int)QThread::currentThreadId()<<endl;
	m_pPSM = new CPolarMatch("LMS151");
	m_pCSM = new CCanonicalMatcher(m_pPSM->m_pParam);
	runfile();
	return ;
}

void CClientFrontend::runRawSeed()
{
	cout<<"ThreadLocal1: runRawseed() "<<(int)QThread::currentThreadId()<<endl;
	m_pPSM = new CPolarMatch("LMS151");
	m_pCSM = new CCanonicalMatcher(m_pPSM->m_pParam);
	runfile();
	return ;
}

bool CClientFrontend::constructPSMfromCarmon(char* line, PMScan& ls, CPolarMatch* m_pPSM){
	ls.rx=ls.ry=ls.th=0;
	char** last;
	string delim(" ");
	double start, fov, resolution, maxRange, accuracy;
	int laserType, remissionMode, num_points;
	// static __thread bool bFirst = true;
	static bool bFirst = true;

	laserType = atoi(strtok_r(line,delim.c_str(),last));
	start = atof(strtok_r(NULL,delim.c_str(),last));
	fov = atof(strtok_r(NULL,delim.c_str(),last));
	resolution = atof(strtok_r(NULL,delim.c_str(),last));
	maxRange = atof(strtok_r(NULL,delim.c_str(),last));
	accuracy = atof(strtok_r(NULL,delim.c_str(),last));
	remissionMode = atoi(strtok_r(NULL,delim.c_str(),last));
	num_points = atoi(strtok_r(NULL,delim.c_str(),last));
	if(num_points != m_pPSM->m_pParam->pm_l_points)
		return false;
	// set parameters
	if(bFirst){
		bFirst = false;
		m_pPSM->m_pParam->pm_fi_min = start;
		m_pPSM->m_pParam->pm_fi_max = start + fov*PM_D2R;
		m_pPSM->pm_init();
		if(maxRange*100 != m_pPSM->m_pParam->pm_max_range)
			m_pPSM->m_pParam->pm_max_range = maxRange*100;
	}
	// read bearings
	for(int i=0;i<num_points;i++){
		ls.r[i] = atof(strtok_r(NULL,delim.c_str(),last))*100.0; // from [m] 2 [cm]
		ls.x[i] = ls.r[i] * m_pPSM->pm_co[i];
		ls.y[i] = ls.r[i] * m_pPSM->pm_si[i];
		ls.bad[i] = 0;
		if(ls.r[i]<= m_pPSM->m_pParam->pm_min_range){
			// ls.r[i]=m_pPSM->m_pParam->pm_max_range+1; //set it to a value larger than the max
			ls.r[i] = 0;
			ls.bad[i] = 1;
		}else if(ls.r[i] >= m_pPSM->m_pParam->pm_max_range)
		{	
			ls.bad[i] = 1;
		}
	}
	return true;
}

inline bool CClientFrontend::constructPSMFromSICK(const CObs2DScan obsScan, CPolarMatch* psm, PMScan& ls){

	if(obsScan.scan.size()!=psm->m_pParam->pm_l_points)
		return false;

	ls.rx=ls.ry=ls.th=0; // its position must be 0,0,0
	ls.t = obsScan.timestamp;

	for(int i=0;i<psm->m_pParam->pm_l_points;i++){
		ls.r[i] = obsScan.scan[i]*100.0; // PSM using cm
		ls.x[i] = ls.r[i] * psm->pm_co[i];
		ls.y[i] = ls.r[i] * psm->pm_si[i];
		ls.bad[i] = 0;
		if(ls.r[i] <= PM_MIN_RANGE){
			ls.r[i] = 0;
			ls.bad[i] = 1;
		}else if(ls.r[i] >= psm->m_pParam->pm_max_range){
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

	// cout<<timestamp<<" "<<m_pPSM->m_pParam->pm_l_points<<" "<<num_points<<" "<<offset<<endl;

	if(m_pPSM->m_pParam->pm_l_points!=num_points){
		cout<<"not enough points in file!"<<endl;
		return false;
	}
	ls.rx=ls.ry=ls.th=0; // its position must be 0,0,0
	ls.t = timestamp;
	
	char *p;

	for(int i=0;i<num_points;i++){
		p = strtok_r(NULL,delim.c_str(),&ptr);
		if(!p) return false;

		// ls.r[i] = atof(strtok_r(NULL,delim.c_str(),&ptr))*100.0; // from [m] 2 [cm]
		ls.r[i] = atof(p)*100.0;
		ls.x[i] = ls.r[i] * m_pPSM->pm_co[i];
		ls.y[i] = ls.r[i] * m_pPSM->pm_si[i];
		ls.bad[i] = 0;
		if(ls.r[i]<= m_pPSM->m_pParam->pm_min_range){
			// ls.r[i]=m_pPSM->m_pParam->pm_max_range+1; //set it to a value larger than the max
			ls.r[i] = 0;
			ls.bad[i] = 1;
		}
	}
	if(m_pPSM->m_pParam->pm_l_points == num_points+1){
		ls.r[num_points] = m_pPSM->m_pParam->pm_max_range+1;
		ls.x[num_points] = ls.y[num_points] = 0;
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
inline bool CClientFrontend::smallMove(OrientedPoint2D& pose)
{
#define SMALL_MOTION 0.05*0.05
#define SMALL_ANGLE 2*PM_D2R
// #define SMALL_MOTION 0.1*0.1
// #define SMALL_ANGLE 10*PM_D2R
	if(pose.x*pose.x + pose.y*pose.y > SMALL_MOTION)
		return false;
	if(pose.theta > SMALL_ANGLE)
		return false;
	return true;
}
bool CClientFrontend::validMove(OrientedPoint2D& pose)
{
	// max speed / pfs
	// 10 m/s /50 = 0.2
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


void CClientFrontend::recvOdo(float x, float y, float theta, float t)
{
	absOdo_cur->x = x / 1000.;
	absOdo_cur->y = y / 1000.;
	absOdo_cur->theta = normAngle(zhpsm::deg2rad(theta-90), -M_PI);
	odo_t = t;

	sendOdo(absOdo_cur->x,absOdo_cur->y,absOdo_cur->theta,odo_t);
}


void CClientFrontend::recvBn(float x, float y, float theta)
{
	absBn_cur->x = x;
	absBn_cur->y = y;
	absBn_cur->theta = normAngle(zhpsm::deg2rad(theta-90), -M_PI);
	//bn_t = t;

	sendBn(absBn_cur->x, absBn_cur->y, absBn_cur->theta);
}

