/*
 * clientFusion.cpp
 *
 *  Created on: Jan 11, 2013
 *      Author: liu
 */
#include <QThread>
#include <fstream>

#include "clientFrontend.h"
#include "ZHPolar_Match.h"
#include "ZHCanonical_Matcher.h"
#include "point.h"
#include "localv1.h"

// sick_reader
#include "CSICK.h"
#include "CObs2DScan.h"
#include "frontendSICK.h"
#include "clientFusion.h"
//#include "ParseXML.h"
#include "C_C.h"

//mode
#define _FUSION_MODE 1 //0: file; 1: sick online
bool _EXTERN_SLAM_PARAM = 1; //0: use default; 1: use extern para from config file


//BN VALID RANGE
#define _BN_MAX_XY 100

#define _MAX_COV_XY 0.25
#define _MAX_COV_THETA 0.04
#define _MAX_COV 9999
#define _MIN_COV 0.9999

float m_fSLAMParams[15];


//using namespace zhpsm;


CClientFusion::CClientFusion():
		x_dim(3),
		oriInRobot_odo(new zhpsm::OrientedPoint2D),
		oriInRobot_minorSICK(new zhpsm::OrientedPoint2D),
		oriInRobot_mainSICK(new zhpsm::OrientedPoint2D),
		mainSICK(0),
		minorSICK(0),
		scan_mainSick(0),
		scan_minorSick(0),
		relp_mainSick(new zhpsm::OrientedPoint2D),
		relp_minorSick(new zhpsm::OrientedPoint2D),
		absPose_mainSick(new zhpsm::OrientedPoint2D),
		absPose_minorSick(new zhpsm::OrientedPoint2D),
		absPose_bn(new zhpsm::OrientedPoint2D),
		absPose_global(new zhpsm::OrientedPoint2D),
		absPose_localize(new zhpsm::OrientedPoint2D),
		absPose_odo_cur(new zhpsm::OrientedPoint2D),
		absPose_odo_last(new zhpsm::OrientedPoint2D),
		relPose_odo(new zhpsm::OrientedPoint2D),
		opd_x_p(new zhpsm::OrientedPoint2D),
		opd_x_u(new zhpsm::OrientedPoint2D),
		mainSick_xp(new zhpsm::OrientedPoint2D),
		mainSick_xu(new zhpsm::OrientedPoint2D),
		minorSick_xp(new zhpsm::OrientedPoint2D),
		minorSick_xu(new zhpsm::OrientedPoint2D),
		odo_xp(new zhpsm::OrientedPoint2D),
		odo_xu(new zhpsm::OrientedPoint2D),
		_MAIN_SICK_READY(false),
		_MINOR_SICK_READY(false),
		_ODO_READY(false),
		_BN_READY(false),
		_SYN_GLOBAL_READY(false),
		_MAIN_SICK_UPDATE(false),
		_MINOR_SICK_UPDATE(false),
		_ODO_UPDATE(false),
		_GPS_UPDATE(false),
		_BN_UPDATE(false),
		_SYN_GLOBAL_UPDATE(false),
		_SYN_LOCALIZE_READY(false),
		_ODO_FIRST(true),
		_LOCALIZE_FIRST(true),
		mainSick_valid(true),
		minorSick_valid(true),
		m_bSent2Localization(false),
		m_syn_num(0)
{

	_USE_MAIN_SICK = true;
	_USE_MINOR_SICK = true;
	_USE_ODO = true;
	_USE_BN = true;

	scnt = 0;
	nmovecnt = 0;
	nmovecnt_main = 0;
	nmovecnt_minor = 0;

	odo_t_cur = 0;
	odo_t_last = 0;

	// Initial covariance
	m_cov = new double[6];
	m_cov_inc = new double[6];
	for(int i=0;i<6;i++)
	{
		m_cov[i] = 0.;
		m_cov_inc[i] = 0.;
	}

	cov_mainSick = new double[6];
	cov_minorSick = new double[6];
	cov_Global = new double[6];

	scan_mainSick = new float[541];
	scan_minorSick = new float[541];

	// Initial filter
	my_filter = new Unscented_scheme(x_dim); // 3 is the dimension of the state
	// Setup the initial state and covariance
	Vec x_init (x_dim);
	SymMatrix X_init (x_dim, x_dim);

	for(int i=0; i<x_dim; i++)
		x_init[i] = 0.;		// Start at 10 with no uncertainty

	x_init[2] = 0;//it should be 0 (difference), since the coordinate of the robot is the same as the world coordinate.
	X_init = ublas::zero_matrix<double>(x_dim, x_dim);

	my_filter->init_kalman (x_init, X_init);
	cout << "Initial  " << my_filter->x << my_filter->X << endl;
}

CClientFusion::~CClientFusion()
{


	if(my_filter) delete my_filter;
	if(mainSICK) delete mainSICK;
	if(minorSICK) delete minorSICK;
	if(oriInRobot_odo) delete oriInRobot_odo;
	if(oriInRobot_mainSICK) delete oriInRobot_mainSICK;
	if(oriInRobot_minorSICK) delete oriInRobot_minorSICK;

	if(m_cov) delete m_cov;
	if(m_cov_inc) delete m_cov_inc;
	if(cov_mainSick) delete cov_mainSick;
	if(cov_minorSick) delete cov_minorSick;
	if(cov_Global) delete cov_Global;
	if(scan_mainSick) delete scan_mainSick;
	if(scan_minorSick) delete scan_minorSick;
	if(my_filter) delete my_filter;

	if(relp_mainSick) delete relp_mainSick;
	if(relp_minorSick) delete relp_minorSick;
	if(absPose_bn) delete absPose_bn;
	if(absPose_odo_cur) delete absPose_odo_cur;
	if(absPose_odo_last) delete absPose_odo_last;
	if(relPose_odo) delete relPose_odo;
	if(opd_x_p) delete opd_x_p;
	if(opd_x_u) delete opd_x_u;

}

void CClientFusion::setGlobalParam( bool use_extern_param)
{
	if(use_extern_param)
	{
		float mainSick_weight = m_fSLAMParams[0];
		float minorSick_weight = m_fSLAMParams[1];
		float bn_weight = m_fSLAMParams[2];
		float odo_weight = m_fSLAMParams[3];
		float synGlobal_weight = m_fSLAMParams[8];
		float predict_weight = m_fSLAMParams[9];
		float synLocalize_weight = m_fSLAMParams[10];

		//cout<<"WEIGHT: "<<" "<<synGlobal_weight<<" "<<predict_weight<<endl;

		// original (x,y,theta)

		/*
				my_filter->x[0] = m_fSLAMParams[4]/1000;
				my_filter->x[1] = m_fSLAMParams[5]/1000;
				my_filter->x[2] = m_fSLAMParams[6];
		 */

		// min and max prob
		if(mainSick_weight == 0)
		{
			mainSick_weight = - _MAX_COV;
			_USE_MAIN_SICK = false;
		}
		else if(mainSick_weight == 1)
		{mainSick_weight =  _MIN_COV;}

		if(minorSick_weight == 0)
		{
			minorSick_weight = - _MAX_COV;
			_USE_MINOR_SICK = false;
		}
		else if(minorSick_weight == 1)
		{minorSick_weight =  _MIN_COV;}

		if(bn_weight == 0)
		{
			bn_weight = - _MAX_COV;
			_USE_BN = false;
		}
		else if(bn_weight == 1)
		{bn_weight =  _MIN_COV;}

		if(odo_weight == 0)
		{
			odo_weight = - _MAX_COV;
			_USE_ODO = false;
		}
		else if(odo_weight == 1)
		{odo_weight = _MIN_COV;}

		if(synGlobal_weight == 0)
		{synGlobal_weight = - _MAX_COV;}
		else if(synGlobal_weight == 1)
		{synGlobal_weight = _MIN_COV;}

		if(predict_weight == 0)
		{predict_weight = - _MAX_COV;}
		else if(predict_weight == 1)
		{predict_weight = _MIN_COV;}

		if(synLocalize_weight == 0)
		{synLocalize_weight = - _MAX_COV;}
		else if(synLocalize_weight == 1)
		{synLocalize_weight = _MIN_COV;}

		sick_observe_main.Zv[0] = (1- mainSick_weight) * _MAX_COV_XY;
		sick_observe_main.Zv[1] = (1- mainSick_weight) * _MAX_COV_XY;
		sick_observe_main.Zv[2] = (1- mainSick_weight) * _MAX_COV_THETA;

		sick_observe_minor.Zv[0] = (1- minorSick_weight) * _MAX_COV_XY;
		sick_observe_minor.Zv[1] = (1- minorSick_weight) * _MAX_COV_XY;
		sick_observe_minor.Zv[2] = (1- minorSick_weight) * _MAX_COV_THETA;

		bn_observe.Zv[0] = (1- bn_weight) * _MAX_COV_XY;
		bn_observe.Zv[1] = (1- bn_weight) * _MAX_COV_XY;
		bn_observe.Zv[2] = (1- bn_weight) * _MAX_COV_THETA;

		odo_observe.Zv[0] = (1- odo_weight) * _MAX_COV_XY;
		odo_observe.Zv[1] = (1- odo_weight) * _MAX_COV_XY;
		odo_observe.Zv[2] = (1- odo_weight) * _MAX_COV_THETA;

		synGlobal_observe.Zv[0] = (1- synGlobal_weight) * _MAX_COV_XY;
		synGlobal_observe.Zv[1] = (1- synGlobal_weight) * _MAX_COV_XY;
		synGlobal_observe.Zv[2] = (1- synGlobal_weight) * _MAX_COV_THETA;

		synLocalize_observe.Zv[0] = (1- synLocalize_weight) * _MAX_COV_XY;
		synLocalize_observe.Zv[1] = (1- synLocalize_weight) * _MAX_COV_XY;
		synLocalize_observe.Zv[2] = (1- synLocalize_weight) * _MAX_COV_THETA;

		robot_predict.q[0] = (1- predict_weight) * _MAX_COV_XY;
		robot_predict.q[1] = (1- predict_weight) * _MAX_COV_XY;
		robot_predict.q[2] = (1- predict_weight) * _MAX_COV_THETA;


		/*
				printf("weight: %f %f %f %f\n;", mainSick_weight, minorSick_weight, bn_weight, odo_weight);
				for(int i=0;i<3;i++)
				{
					printf("cov: %f %f %f %f\n;", sick_observe_main.Zv[i], sick_observe_minor.Zv[i], bn_observe.Zv[i], odo_observe.Zv[i]);
				}
		 */

	}
}

bool CClientFusion::initFusion()
{

	if(sick_ip.size()<2 || sick_ip.size()!=sick_port.size())
	{
		cout<<"CClientFusion::initFusion() "<<" No SICK IP found!"<<endl;
		return false;
	}

	//load configured value for cov
	setGlobalParam(_EXTERN_SLAM_PARAM);

	mainSICK = new CFrontendSICK;
	mainSICK->setModel(work_model);
	mainSICK->setSickIP(sick_ip[0], sick_port[0]);
	mainSICK->setSickName("MainSick");
	//mainSICK->setOriInRobot(*oriInRobot_mainSICK);
	mainSICK->moveToThread(&mThreadMainSICK);

	minorSICK = new CFrontendSICK;
	minorSICK->setModel(work_model);
	minorSICK->setSickIP(sick_ip[1], sick_port[1]);
	minorSICK->setSickName("MinorSick");
	//minorSICK->setOriInRobot(*oriInRobot_minorSICK);
	minorSICK->moveToThread(&mThreadMinorSICK);

	if(!_FUSION_MODE)
	{
		//testing using ourdata in rawseed format
		//mainSICK->setFile("/home/administrator/Work/lms151/20130108_t2.txt");
		mainSICK->setFile("/home/administrator/Desktop/zh_mainctrl/timett.log");
		minorSICK->setFile("/home/administrator/Work/lms151/20130108_tt1.txt");
		connect(&mThreadMainSICK,SIGNAL(started()),mainSICK,SLOT(runRS()),Qt::DirectConnection); // for debuging using rawseed data
		connect(&mThreadMinorSICK,SIGNAL(started()),minorSICK,SLOT(runRS()),Qt::DirectConnection);
	}
	else
	{
		connect(&mThreadMainSICK,SIGNAL(started()),mainSICK,SLOT(runSick()),Qt::DirectConnection);
		connect(&mThreadMinorSICK,SIGNAL(started()),minorSICK,SLOT(runSick()),Qt::DirectConnection);
	}

	connect(mainSICK,SIGNAL(finished()),&mThreadMainSICK,SLOT(quit()),Qt::DirectConnection);
	connect(minorSICK,SIGNAL(finished()),&mThreadMinorSICK,SLOT(quit()),Qt::DirectConnection);
	connect(&mThreadMainSICK,SIGNAL(finished()),this,SLOT(stop()),Qt::DirectConnection);
	connect(&mThreadMinorSICK,SIGNAL(finished()),this,SLOT(stop()),Qt::DirectConnection);


	/*
	connect(mainSICK,SIGNAL(sendRelFrameInfo(double,double,double,float*,int,TTimeStamp, double*,int,bool)),
			this,SLOT(recvRelMainSICK(double,double,double,float*,int,TTimeStamp, double*,int,bool)),Qt::DirectConnection);
	connect(minorSICK,SIGNAL(sendRelFrameInfo(double,double,double,float*,int,TTimeStamp, double*,int,bool)),
			this,SLOT(recvRelMinorSICK(double,double,double,float*,int,TTimeStamp, double*,int,bool)),Qt::DirectConnection);
	 */

	connect(mainSICK,SIGNAL(sendRelFrameInfo2(double,double,double,void*,int,bool)),
			this,SLOT(recvRelMainSICK2(double,double,double,void*,int,bool)),Qt::DirectConnection);
	connect(minorSICK,SIGNAL(sendRelFrameInfo2(double,double,double,void*,int,bool)),
			this,SLOT(recvRelMinorSICK2(double,double,double,void*,int,bool)),Qt::DirectConnection);


	if(work_model == 3)
	{
		mThreadMainSICK.start();
		//mThreadMinorSICK.start();
	}

	return true;
}

// Run Sensor Fusion using UIF
void CClientFusion::runFusionInfo(){

	cout<<"CClientFusion::runFusionInfo()"<<endl;


	zhpsm::OrientedPoint2D rel_pose;
	zhpsm::OrientedPoint2D sent_pose;
	bool first = true;

	cvtX2OPD(my_filter->x, *mainSick_xp);
	cvtX2OPD(my_filter->x, *minorSick_xp);
	cvtX2OPD(my_filter->x, *odo_xp);

	Vec z(3);

	ofstream outf("runFusionInfo.log");
	outf<<"COV: "<<robot_predict.q<<" "<<sick_observe_main.Zv<<" "<<odo_observe.Zv<<" "<<bn_observe.Zv<<" "<<synGlobal_observe.Zv<<endl;

	while(!m_stop_thread)
	{

		float* scanForShow = 0;
		int npForShow = 0;

		QMutexLocker lock_filter(&mutex_filter);

		if(_ODO_READY)// )//  _ODO_READY)//  _MAIN_SICK_READY  &&&& &&//_SYN_LOCALIZE_READY &&  || _MINOR_SICK_READY || _ODO_READY || _BN_READY )

		{

			//prediction
			my_filter->predict(robot_predict);

			if(_MAIN_SICK_READY && mainSick_valid)
			{
				scanForShow = scan_mainSick;
				npForShow = np_mainSick;

				//calc info contribution
				cvtOPD2X(*mainSick_xp,z);
				outf<<"MAIN SICK REL: "<<z<<endl;
				outf<<"MAIN SICK ABS: "<<*absPose_mainSick<<endl;
				my_filter->observeFusion(sick_observe_main,z);

				_MAIN_SICK_READY = false;

			}
			mainSick_valid = true; // refresh every loop

			if(_ODO_READY )
			{
				//calc info contribution
				cvtOPD2X(*odo_xp,z);
				outf<<"ODO REL: "<<z<<endl;
				outf<<"ODO ABS: "<<*absPose_odo_cur<<endl;
				my_filter->observeFusion(odo_observe,z);
				_ODO_READY = false;

			}

			if(_BN_READY)
			{
				//calc info contribution
				cvtOPD2X(*absPose_bn,z);
				outf<<"BN ABS: "<<z<<endl;
				my_filter->observeFusion(bn_observe,z);
				_BN_READY = false;
			}

			/*
			if(_SYN_GLOBAL_READY)
			{
				cvtOPD2X(*absPose_global,z);
				outf<<"GLOBAL ABS: "<<z<<endl;
				my_filter->observeFusion(synGlobal_observe,z);
				_SYN_GLOBAL_READY = false;
			}
			*/

			if(_SYN_LOCALIZE_READY)
			{
				cvtOPD2X(*absPose_localize,z);
				outf<<"LOCALIZE ABS: "<<z<<endl;
				my_filter->observeFusion(synLocalize_observe,z);
				_SYN_LOCALIZE_READY = false;
			}

			//Final Fusion
			my_filter->updateFusion();

			outf<<"FUSION: "<<my_filter->x<<" "<<my_filter->X<<endl;

			cvtX2OPD(my_filter->x, *odo_xp);
			cvtX2OPD(my_filter->x, *mainSick_xp);
			cvtX2OPD(my_filter->x, *minorSick_xp);
			cvtX2OPD(my_filter->x, *opd_x_u);

			//send frame to server
			if(npForShow>0)// && nmovecnt_main==0 && m_pFrame_mainSick)
			{
								//send to backend
				//sendScanFrameCov((void*)(m_pFrame_mainSick), m_syn_num, m_cov_inc); // send to Backend
				//send to server for show
				sendFrameInfo(opd_x_u->x,opd_x_u->y,opd_x_u->theta, scanForShow, npForShow);
				// record fused trajectory
				m_traj.push_back(new zhpsm::OrientedPoint2D(*opd_x_u));
				traj_index.push_back(m_traj.size()-1);
				//send to RS server for show
				send2CBScan(scanForShow, npForShow, opd_x_u->x,opd_x_u->y,opd_x_u->theta);
				//cout<<"CClientFusion::runFusionInfo pose: "<<opd_x_u->x<<" "<<opd_x_u->y<<" "<<opd_x_u->theta<<endl;
			}

		}
		else
		{
			QThread::yieldCurrentThread();
			continue;
		}

	}

	finished();
}

// Run Sensor Fusion using UIF
void CClientFusion::runFusionPara(){

	cout<<"CClientFusion::runFusionPara()"<<endl;

	Vec z(3);

	//

	zhpsm::OrientedPoint2D rel_pose;
	zhpsm::OrientedPoint2D sent_pose;
	bool first = true;

	cvtX2OPD(my_filter->x, *mainSick_xp);
	cvtX2OPD(my_filter->x, *minorSick_xp);
	cvtX2OPD(my_filter->x, *odo_xp);

	ofstream outf("runFusionPara.log");
	outf<<"COV: "<<robot_predict.q<<" "<<sick_observe_main.Zv<<" "<<odo_observe.Zv<<" "<<bn_observe.Zv<<" "<<synGlobal_observe.Zv<<endl;

	while(!m_stop_thread)
	{

		float* scanForShow = 0;
		int npForShow = 0;

		QMutexLocker lock_filter(&mutex_filter);
		if(_MAIN_SICK_READY)//_MAIN_SICK_READY)///* _ODO_READY  || _MAIN_SICK_READY  || _MINOR_SICK_READY || _BN_READY || _SYN_GLOBAL_READY*/_SYN_LOCALIZE_READY)
		{

			//prediction
			my_filter->predict(robot_predict);


			if(_MAIN_SICK_READY && mainSick_valid)
			{
				scanForShow = scan_mainSick;
				npForShow = np_mainSick;

				//calc info contribution
				cvtOPD2X(*mainSick_xp,z);
				outf<<"MAIN SICK REL: "<<z<<endl;
				outf<<"MAIN SICK ABS: "<<*absPose_mainSick<<endl;
				my_filter->observeFusion(sick_observe_main,z);

				_MAIN_SICK_READY = false;
				_MAIN_SICK_UPDATE = true;

			}
			mainSick_valid = true; // refresh every loop

			if(_MINOR_SICK_READY && minorSick_valid)
			{
				/*
				scanForShow = scan_minorSick;
				npForShow = np_minorSick;
				 */

				//calc info contribution
				cvtOPD2X(*minorSick_xp,z);
				outf<<"MINOR SICK REL: "<<z<<endl;
				outf<<"MINOR SICK ABS: "<<*absPose_minorSick<<endl;
				my_filter->observeFusion(sick_observe_minor,z);

				_MINOR_SICK_READY = false;
				_MINOR_SICK_UPDATE = true;

			}
			minorSick_valid = true; // refresh every loop

			if(_ODO_READY )
			{
				//calc info contribution
				cvtOPD2X(*odo_xp,z);
				outf<<"ODO REL: "<<z<<endl;
				outf<<"ODO ABS: "<<*absPose_odo_cur<<endl;
				my_filter->observeFusion(odo_observe,z);
				_ODO_READY = false;
				_ODO_UPDATE = true;

			}

			if(_BN_READY)
			{
				//calc info contribution
				cvtOPD2X(*absPose_bn,z);
				outf<<"BN ABS: "<<z<<endl;
				my_filter->observeFusion(bn_observe,z);
				_BN_READY = false;
				_BN_UPDATE = true;
			}

			/*

			if(_SYN_GLOBAL_READY)
			{
				cvtOPD2X(*absPose_global,z);
				outf<<"GLOBAL ABS: "<<z<<endl;
				my_filter->observeFusion(synGlobal_observe,z);
				_SYN_GLOBAL_READY = false;
				_SYN_GLOBAL_UPDATE = true;
			}
			 */



			if(_SYN_LOCALIZE_READY)
			{
				cvtOPD2X(*absPose_localize,z);
				outf<<"LOCALIZE ABS: "<<z<<endl;
				my_filter->observeFusion(synLocalize_observe,z);
				_SYN_LOCALIZE_READY = false;
			}


			//Final Fusion
			my_filter->updateFusion();

			outf<<"FUSION: "<<my_filter->x<<" "<<my_filter->X<<endl;

			if(_ODO_UPDATE)
			{
				cvtX2OPD(my_filter->x, *odo_xp);
				_ODO_UPDATE = false;
			}

			if(_MAIN_SICK_UPDATE)
			{
				cvtX2OPD(my_filter->x, *mainSick_xp);
				_MAIN_SICK_UPDATE = false;
			}

			if(_MINOR_SICK_UPDATE)
			{
				cvtX2OPD(my_filter->x, *minorSick_xp);
				_MINOR_SICK_UPDATE = false;
			}

			cvtX2OPD(my_filter->x, *opd_x_u);

			//send frame to server
			if(npForShow>0)// && nmovecnt_main==0 && m_pFrame_mainSick)
			{

				//send to backend
				//sendScanFrameCov((void*)(m_pFrame_mainSick), m_syn_num, m_cov_inc); // send to Backend
				//send to server for show
				sendFrameInfo(opd_x_u->x,opd_x_u->y,opd_x_u->theta, scanForShow, npForShow);
				// record fused trajectory
				m_traj.push_back(new zhpsm::OrientedPoint2D(*opd_x_u));
				traj_index.push_back(m_traj.size()-1);


			}

			//send to RS server for show
			//cout<<"CClientFusion::runFusionPara pose: "<<opd_x_u->x<<" "<<opd_x_u->y<<" "<<opd_x_u->theta<<endl;
			if(npForShow>0)
				send2CBScan(scanForShow, npForShow, opd_x_u->x,opd_x_u->y,opd_x_u->theta);

		}
		else
		{
			QThread::yieldCurrentThread();
			continue;
		}

	}

	finished();
}



void CClientFusion::runODO()
{

	//cout<<"runOdo!!!!!!!!!!!!!!!!!!!!!!!"<<endl;

	while(!m_stop_thread)
	{

		QMutexLocker lock(&mutex_filter);
		if( _ODO_READY )
		{
			//cout<<"sendOdo!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
			//send2CBScan(0, 0, absPose_odo_cur->x,absPose_odo_cur->y,absPose_odo_cur->theta);
			sendODO(absPose_odo_cur->x,absPose_odo_cur->y,absPose_odo_cur->theta,odo_t_last);
			_ODO_READY = false;
		}
		else
		{
			QThread::yieldCurrentThread();
			continue;
		}
	}

}

void CClientFusion::runBN()
{


	while(!m_stop_thread)
	{
		QMutexLocker lock(&mutex_filter);
		if( _BN_READY )
		{
			sendBN(absPose_bn->x, absPose_bn->y, absPose_bn->theta);
//send2CBScan(0, 0, absPose_bn->x,absPose_bn->y,absPose_bn->theta);
			_BN_READY = false;
		}else
		{
			QThread::yieldCurrentThread();
			continue;
		}
	}

}

void CClientFusion::updateBNNode(){

	if(_BN_READY)
	{
		//cout<<"CClientFusion::updateBNNode()"<<endl;
		Vec z(3);
		cvtOPD2X(*absPose_bn, z);

		my_filter->observe (bn_observe, z);
		my_filter->update();

		_BN_READY = false;
	}
}

void CClientFusion::updateOdoNode(){

	if(_ODO_READY)
	{
		//cout<<"CClientFusion::updateOdoNode()"<<endl;
		//relpToRobotFrame(*oriInRobot_odo, *relPose_odo);
		//*opd_x_u = opd_x_p->oplus(*relPose_odo);
		Vec z(3);
		cvtOPD2X(*odo_xp, z);
		//cvtOPD2X(*opd_x_u, z);


		my_filter->observe (odo_observe, z);
		my_filter->update();
		cvtX2OPD(my_filter->x, *odo_xp);

		/*
		relPose_odo->x = 0.;
		relPose_odo->y = 0.;
		relPose_odo->theta = 0.;
		 */
		_ODO_READY = false;
	}

}

void CClientFusion::updateMainSICKNode(){


	if(_MAIN_SICK_READY)
	{
		cout<<"CClientFusion::updateMainSICKNode()"<<endl;

		//relpToRobotFrame(*oriInRobot_mainSICK, *relp_mainSick);
		//*opd_x_u = opd_x_p->oplus(*relp_mainSick);

		Vec z(3);
		cvtOPD2X(*mainSick_xp, z);
		//cvtOPD2X(*opd_x_u, z);


		my_filter->observe (sick_observe_main, z);
		my_filter->update();

		cvtX2OPD(my_filter->x, *mainSick_xp);

		_MAIN_SICK_READY = false;
	}


}

void CClientFusion::updateMinorSICKNode(){


	//QMutexLocker lock_minor(&mutex_minorSICK);
	if(_MINOR_SICK_READY)
	{
		//cout<<"CClientFusion::updateMinorSICKNode()"<<endl;

		//relpToRobotFrame(*oriInRobot_minorSICK, *relp_minorSick);
		//*opd_x_u = opd_x_p->oplus(*relp_minorSick);
		Vec z(3);
		cvtOPD2X(*minorSick_xp, z);
		//cvtOPD2X(*opd_x_u, z);


		//cout<<" "<<sick_observe_minor.Zv[0]<<" "<<sick_observe_minor.Zv[1]<<" "<<sick_observe_minor.Zv[2]<<endl;
		my_filter->observe (sick_observe_minor, z);
		my_filter->update();

		cvtX2OPD(my_filter->x, *minorSick_xp);

		/*
		relp_minorSick->x = 0.;
		relp_minorSick->y = 0.;
		relp_minorSick->theta = 0.;
		 */
		_MINOR_SICK_READY = false;
	}

}


/*Receive Relative Pose from Main SICK Module*/
void CClientFusion::recvRelMainSICK(double x,double y,double theta,float* scan, int np, TTimeStamp t, double* cov, int nmovecnt,bool bGoodMatch){
	//cout<<"CClientFusion::recvRelMainSICK" <<x<<" "<<y<<" "<<theta<<" "<<nmovecnt<<endl;
	{
		//QMutexLocker lock(&mutex_mainSICK);
		QMutexLocker lock(&mutex_filter);

		if(np != 541)
		{
			printf("ERROR!!!!!!!!!!!!!!!!!!!!! \n");
			return;
		}

		float *pscan = scan;
		memcpy(&scan_mainSick[0],pscan,np*sizeof(float));
		memcpy(cov_mainSick,cov,6*sizeof(double));

		//accumulative relative pose
		zhpsm::OrientedPoint2D p;
		p.x = x;
		p.y = y;
		p.theta = theta;
		relpToRobotFrame(*oriInRobot_mainSICK, p);
		*mainSick_xp = mainSick_xp->oplus(p);
		*absPose_mainSick = absPose_mainSick->oplus(p);
		//*relp_mainSick = relp_mainSick->oplus(p);

		//cout<<"CClientFusion::recvRelMainSICK ACC: " <<mainSick_xp->x<<" "<<mainSick_xp->y<<" "<<mainSick_xp->theta<<endl;

		np_mainSick = np;
		t_mainSick = t;
		nmovecnt_main = nmovecnt;

		//if one scan is invalid, then this false state only will be changed after updating once;

		if(mainSick_valid && !bGoodMatch)
		{
			mainSick_valid = false;
		}


		_MAIN_SICK_READY = true;
		//cout<<"MAIN READy"<<endl;
	}
}

void CClientFusion::recvRelMainSICK2(double x,double y,double theta,void* p_ls,int nmovecnt,bool bGoodMatch)
{

	//cout<<"CClientFusion::recvRelMainSICK2 " <<x<<" "<<y<<" "<<theta<<" "<<nmovecnt<<endl;
	QMutexLocker lock(&mutex_filter);

	PMScan* ls = static_cast<PMScan*>(p_ls);
	m_pFrame_mainSick = new PMScan(*ls);

	if(m_pFrame_mainSick->np != 541)
	{
		printf("ERROR!!!!!!!!!!!!!!!!!!!!! \n");
		return;
	}

	float *pscan = m_pFrame_mainSick->r;
	memcpy(&scan_mainSick[0],pscan,m_pFrame_mainSick->np*sizeof(float));

	//accumulative relative pose
	zhpsm::OrientedPoint2D p;
	p.x = x;
	p.y = y;
	p.theta = theta;
	relpToRobotFrame(*oriInRobot_mainSICK, p);

	*mainSick_xp = mainSick_xp->oplus(p);
	*absPose_mainSick = absPose_mainSick->oplus(p);
	//*relp_mainSick = relp_mainSick->oplus(p);

	//cout<<"CClientFusion::recvRelMainSICK ACC: " <<mainSick_xp->x<<" "<<mainSick_xp->y<<" "<<mainSick_xp->theta<<endl;

	np_mainSick = m_pFrame_mainSick->np;
	t_mainSick = m_pFrame_mainSick->t;
	nmovecnt_main = nmovecnt;

	//if one scan is invalid, then this false state only will be changed after updating once;

	if(mainSick_valid && !bGoodMatch)
	{
		mainSick_valid = false;
	}
	_MAIN_SICK_READY = true;


	//send to localization

	bool bSent2Localization = m_bSent2Localization;
	m_pFrame_mainSick->rx = mainSick_xp->x * 100.;
	m_pFrame_mainSick->ry = mainSick_xp->y * 100.;
	m_pFrame_mainSick->th = mainSick_xp->theta;
	if(_LOCALIZE_FIRST)
	{
		_LOCALIZE_FIRST = false;
		//cout<<"_LOCALIZE_FIRST "<<m_pFrame_mainSick->t<<" "<<m_pFrame_mainSick->rx<<" "<<m_pFrame_mainSick->ry<<" "<<m_pFrame_mainSick->th<<endl;
		sendFirstFrame((void*)(m_pFrame_mainSick)); // send to localization to syn coordinates

		/*cout<<"FIRST SCAN: ";
		for(int i=0;i<541;i++)
			cout<<" "<<m_pFrame_mainSick->r[i];
		cout<<endl;
		 */
	}
	else if(!bSent2Localization)
	{

		//cout<<"bSent2Localization: "<< m_traj.size()-1<<" "<<m_pFrame_mainSick->t<<" "<<m_pFrame_mainSick->rx<<" "<<m_pFrame_mainSick->ry<<" "<<m_pFrame_mainSick->th<<endl;

		/*cout<<"SCAN: ";
		for(int i=0;i<541;i++)
			cout<<" "<<m_pFrame_mainSick->r[i];
		cout<<endl;
		 */

		sendScanFrame((void*)(m_pFrame_mainSick),m_traj.size()-1); // send to Localization
		m_bSent2Localization = true;
		//usleep(2000);
	}
	delete m_pFrame_mainSick;

	//cout<<"MAIN READy"<<endl;

}

void CClientFusion::recvRelMinorSICK2(double x,double y,double theta,void* p_ls,int nmovecnt,bool bGoodMatch)
{

	//cout<<"CClientFusion::recvRelMinorSICK2 " <<x<<" "<<y<<" "<<theta<<" "<<nmovecnt<<endl;
	QMutexLocker lock(&mutex_filter);

	PMScan* ls = static_cast<PMScan*>(p_ls);
	m_pFrame_minorSick = new PMScan(*ls);

	if(m_pFrame_minorSick->np != 541)
	{
		printf("ERROR!!!!!!!!!!!!!!!!!!!!! \n");
		return;
	}

	float *pscan = m_pFrame_minorSick->r;
	memcpy(&scan_minorSick[0],pscan,m_pFrame_minorSick->np*sizeof(float));

	//accumulative relative pose
	zhpsm::OrientedPoint2D p;
	p.x = x;
	p.y = y;
	p.theta = theta;
	relpToRobotFrame(*oriInRobot_minorSICK, p);
	*minorSick_xp = minorSick_xp->oplus(p);
	*absPose_minorSick = absPose_minorSick->oplus(p);
	//*relp_minorSick = relp_minorSick->oplus(p);

	//cout<<"CClientFusion::recvRelMinorSICK ACC: " <<minorSick_xp->x<<" "<<minorSick_xp->y<<" "<<minorSick_xp->theta<<endl;

	np_minorSick = m_pFrame_minorSick->np;
	t_minorSick = m_pFrame_minorSick->t;
	nmovecnt_minor = nmovecnt;

	//if one scan is invalid, then this false state only will be changed after updating once;

	if(minorSick_valid && !bGoodMatch)
	{
		minorSick_valid = false;
	}


	_MINOR_SICK_READY = true;
	//cout<<"MINOR READy"<<endl;

}




/*Receive Relative Pose from Main SICK Module*/
void CClientFusion::recvRelMinorSICK(double x,double y,double theta,float* scan, int np, TTimeStamp t, double* cov, int nmovecnt,bool bGoodMatch){
	//cout<<"CClientFusion::recvRelMinorSICK" <<x<<" "<<y<<" "<<theta<<endl;
	{
		//QMutexLocker lock_minor(&mutex_minorSICK);
		QMutexLocker lock(&mutex_filter);

		if(np != 541)
		{
			printf("ERROR!!!!!!!!!!!!!!!!!!!!! \n");
			return;
		}

		float *pscan = scan;
		memcpy(&scan_minorSick[0],pscan,np*sizeof(float));
		memcpy(cov_minorSick,cov,6*sizeof(double));

		//accumulative relative pose
		zhpsm::OrientedPoint2D p;
		p.x = x;
		p.y = y;
		p.theta = theta;
		//*relp_minorSick = relp_minorSick->oplus(p);
		relpToRobotFrame(*oriInRobot_minorSICK, p);
		*minorSick_xp = minorSick_xp->oplus(p);
		*absPose_minorSick = absPose_minorSick->oplus(p);

		np_minorSick = np;
		t_minorSick = t;
		nmovecnt_minor = nmovecnt;

		if(minorSick_valid && !bGoodMatch)
		{
			minorSick_valid = false;
		}

		_MINOR_SICK_READY = true;

		//cout<<"MINOR READy"<<endl;

	}
}

/*receive abs pose from BN. unit is (cm, cm, rad)*/
//Will update BN immediately
void CClientFusion::recvBN(float x,float y,float theta)
{
	//cout<<"CClientFusion::recvBN" <<x<<" "<<y<<" "<<theta<<endl;
	{
		QMutexLocker lock(&mutex_filter);
		absPose_bn->x = x / 100.; //cm to m
		absPose_bn->y = y / 100.;
		absPose_bn->theta = zhpsm::normAngle(zhpsm::deg2rad(theta-90.), -M_PI);//;

		if(fabs(absPose_bn->x)>_BN_MAX_XY || fabs(absPose_bn->y)>_BN_MAX_XY)
		{}
		else
		{
			_BN_READY = true;
			sendBN(absPose_bn->x, absPose_bn->y, absPose_bn->theta);
		}
	}
}


/*receive abs pose from odo. unit is (mm, mm, deg)*/
//Will update Odo immediately
void CClientFusion::recvODO(float x,float y,float theta,float t)
{
	//cout<<"CClientFusion::recvODO" <<x<<" "<<y<<" "<<theta<<endl;
	{
		QMutexLocker lock(&mutex_filter);
		if(_ODO_FIRST)
		{
			absPose_odo_cur->x = x / 1000.; //mm to m
			absPose_odo_cur->y = y / 1000.;
			absPose_odo_cur->theta = zhpsm::normAngle(zhpsm::deg2rad(theta - 90.), -M_PI);

			*absPose_odo_last = *absPose_odo_cur;

			odo_t_last = t;

			_ODO_READY = false;
			_ODO_FIRST = false;

			sendODO(absPose_odo_cur->x,absPose_odo_cur->y,absPose_odo_cur->theta,odo_t_last);
			return;
		}

		if(t == odo_t_last)
		{
			_ODO_READY = false;
			return ;
		}

		absPose_odo_cur->x = x / 1000.; //mm to m
		absPose_odo_cur->y = y / 1000.;
		absPose_odo_cur->theta = zhpsm::normAngle(zhpsm::deg2rad(theta - 90.), -M_PI);

		zhpsm::OrientedPoint2D p = absPose_odo_last->ominus(*absPose_odo_cur);

		//printf("Odo Rel: %f %f %f \n", p.x, p.y, p.theta);

		if(p.x<10 || p.y<10 || fabs(p.theta)<M_PI_2)//this->validMove(p))
		{
			relpToRobotFrame(*oriInRobot_odo, p);
			*relPose_odo = relPose_odo->oplus(p);

			*odo_xp = odo_xp->oplus(p);
			*absPose_odo_last = *absPose_odo_cur;
			odo_t_last = t;
			_ODO_READY = true;

			//cout<<"ODO READy"<<endl;
		}
		else
		{
			_ODO_READY = false;
		}

		sendODO(absPose_odo_cur->x,absPose_odo_cur->y,absPose_odo_cur->theta,odo_t_last);


	}
}


void CClientFusion::recvLocalizedPose(int id, void* pose, bool bIgnored)
{
	if(bIgnored) return ;
	//cout<<"CClientFusion::recvLocalizedPose " <<id<<endl;
	QMutexLocker lock(&mutex_filter);
	zhpsm::OrientedPoint2D* p = static_cast<zhpsm::OrientedPoint2D*>(pose);
	if(p==NULL) return ;
	if(id < 0 || id >= m_traj.size()){
		cout<<"error in CClientFusion::recvLocalizedPose!"<<endl;
		return ;
	}
	//cout<<"CClientFusion::recvLocalizedPose " <<id<<" "<<*p<<endl;
	{
		zhpsm::OrientedPoint2D* last = m_traj[m_traj.size()-1];
		zhpsm::OrientedPoint2D* prev = m_traj[id];
		zhpsm::OrientedPoint2D trans = prev->ominus(*last);
		zhpsm::OrientedPoint2D changed = prev->ominus(*p);
		//cout<<"recvLocalizedPose from Localization: trans: "<<changed<<endl;
		//cout<<"id: "<<id<<" total: "<<m_traj.size()<<endl;

		if(abs(changed.x) > 1 || fabs(changed.y)>1 || fabs(changed.theta)>0.2)
		{
			m_bSent2Localization = false;
			_SYN_LOCALIZE_READY = false;

		}
		else
		{
			*last = p->oplus(trans);//changed.oplus(trans);
			*absPose_localize = *last;
			//*absPose_mainSick = *last; // ??

			m_bSent2Localization = false;
			_SYN_LOCALIZE_READY = true;

/*
			Vec z(3);
			cvtOPD2X(*absPose_localize,z);
			my_filter->observeFusion(synLocalize_observe,z);
			_SYN_LOCALIZE_READY = false;
			my_filter->updateFusion();
*/
			sendLD(absPose_localize->x, absPose_localize->y, absPose_localize->theta);

		}


	}
}

/*Receive Relative Pose from Main SICK Module*/
void CClientFusion::recvRelGlobalOptimized(int id, void* pose, double* cov){
	//cout<<"CClientFusion::recvRelGlobalOptimized " <<id<<endl;

	QMutexLocker lock(&mutex_filter);
	zhpsm::OrientedPoint2D* p = static_cast<zhpsm::OrientedPoint2D*>(pose);
	memcpy(cov_Global,cov,6*sizeof(double));
	//cout<<"CClientFusion::recvRelGlobalOptimized cov: "<<cov_Global[0]<<" "<<cov_Global[3]<<" "<<cov_Global[5]<<endl;
	if(p==NULL) return ;
	if(id < 0 || id >= traj_index.size()){
		cout<<"error in CClientFrontend::recvRelGlobalOptimized!"<<endl;
		return ;
	}
	{

		zhpsm::OrientedPoint2D* last = m_traj[m_traj.size()-1];
		zhpsm::OrientedPoint2D* prev = m_traj[traj_index[id]];
		zhpsm::OrientedPoint2D trans = prev->ominus(*last);
		zhpsm::OrientedPoint2D changed = *prev + *p;
		*last = changed.oplus(trans);
		*absPose_global = *last;
		_SYN_GLOBAL_READY = true;
		m_syn_num ++;
	}


}



/* trans the relative pose to the robot frame;*/
void CClientFusion::relpToRobotFrame(const zhpsm::OrientedPoint2D oriInRobot, zhpsm::OrientedPoint2D &relp)
{
	double x = relp.x;
	double y = relp.y;
	double ctheta = cos(oriInRobot.theta), stheta = sin(oriInRobot.theta);
	relp.x = x * ctheta - y * stheta;
	relp.y = x * stheta + y * ctheta;
}


/*Convert zhpsm::OrientedPoint2D to FM::Vec format.
 * The angle will be normalized to [-pi, pi] first*/
void CClientFusion::cvtOPD2X(zhpsm::OrientedPoint2D& opd, FM::Vec & x)
{
	if(x.size()!=3)
	{
		cout<<"ERROR: cvtOPD2X(); x dimission must be 3!"<<endl;
		return ;
	}
	opd.theta = zhpsm::normAngle(opd.theta, -M_PI);

	/*x[0] = opd.x * 100; //metre to cm
	x[1] = opd.y * 100;
	x[2] = rad2deg(opd.theta);// * rad2deg; //rad to degree
	 */
	x[0] = opd.x;
	x[1] = opd.y;
	x[2] = opd.theta;
}

// unit and format conversion
void CClientFusion::cvtX2OPD(FM::Vec& x, zhpsm::OrientedPoint2D& opd)
{
	if(x.size()!=3)
	{
		cout<<"ERROR: cvtX2OPD(); x dimission must be 3!"<<endl;
		return ;
	}

	/*opd.x = x[0] / 100; // cm to m
	opd.y = x[1] / 100;
	opd.theta = zhpsm::normAngle(zhpsm::deg2rad(x[2]), -M_PI);// * D2R; // degree to rad

	x[2] = rad2deg(opd.theta);*/
	opd.x = x[0];
	opd.y = x[1];
	opd.theta = zhpsm::normAngle(x[2], -M_PI);
	x[2] = opd.theta;
}

/*
int CClientFusion::LoadConfig(char *pcConfigPath)
{
	//ParseXML CParseXML;
	char cIP[4],cHostIP[4];
	int i,j,nPort,nHostPort;
	m_CParseXML.ParseXMLRun(pcConfigPath);


	j=0;
	for(i=22;i<m_CParseXML.m_vctData.size();i++)
	{
		printf("Param:  %s  \n",m_CParseXML.m_vctData[i].c_str());
		m_fSLAMParams[j++]=atof(m_CParseXML.m_vctData[i].c_str());
	}


	return 0;
}
 */



