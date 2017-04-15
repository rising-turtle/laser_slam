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

// sick_reader
#include "CSICK.h"
#include "CObs2DScan.h"
#include "frontendSICK.h"
#include "frontendOdo.h"
#include "clientFusion.h"

using namespace zhpsm;


CClientFusion::CClientFusion():
x_dim(3),
n_steps(0),
oriInRobot_odo(new OrientedPoint2D),
oriInRobot_minorSICK(new OrientedPoint2D),
oriInRobot_mainSICK(new OrientedPoint2D),
mainSICK(0),
minorSICK(0),
scan_mainSick(0),
scan_minorSick(0),
cov_mainSick(0),
cov_minorSick(0),
relp_mainSick(new OrientedPoint2D),
relp_minorSick(new OrientedPoint2D),
relp_odo(new OrientedPoint2D),
opd_x_p(new OrientedPoint2D),
opd_x_u(new OrientedPoint2D),
_MAIN_SICK_READY(false),
_MINOR_SICK_READY(false),
_ODO_READY(false),
syn_mainSick(0)
{
	// Initial covariance
	cov_mainSick = new double[6];
	cov_Global = new double[6];

	// Initial filter
	my_filter = new Unscented_scheme(x_dim); // 3 is the dimension of the state
	// Setup the initial state and covariance
	Vec x_init (x_dim);
	SymMatrix X_init (x_dim, x_dim);

	for(int i=0; i<x_dim; i++)
		x_init[i] = 0.;		// Start at 10 with no uncertainty
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
	if(relp_mainSick) delete relp_mainSick;
	if(relp_minorSick) delete relp_minorSick;
	if(relp_odo) delete relp_odo;

}

bool CClientFusion::initFusion()
{
	cout<<sick_ip.size()<<endl;
	if(sick_ip.size()<2 || sick_ip.size()!=sick_port.size())
	{
		cout<<"CClientFusion::initFusion() "<<" No SICK IP found!"<<endl;
		return false;
	}

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

	odo = new CFrontendOdo;
	odo->moveToThread(&mThreadOdo);


	bool test = false;
	if(test)
	{
		//testing


		{
			//lenovo, 151
			mainSICK->setFile("/media/ShareRegion/Datasets/LMS151_20130125/exp1.txt");
			// mainSICK->setFile("/mnt/hgfs/SharedFold/dataset/lenovo/LMS151/20130108_t2.txt");
		//	mainSICK->setFile("/mnt/hgfs/SharedFold/dataset/lenovo/LMS151/20130219.txt");

			connect(&mThreadMainSICK,SIGNAL(started()),mainSICK,SLOT(runCM()),Qt::DirectConnection); // for debuging using rawseed data
			connect(&mThreadMinorSICK,SIGNAL(started()),minorSICK,SLOT(runCM()),Qt::DirectConnection);
			connect(&mThreadOdo,SIGNAL(started()),odo,SLOT(runRawseedOdo()),Qt::DirectConnection);
		
		}

		/*
		{

			//intel, but 211
			mainSICK->setFile("/media/ShareRegion/Datasets/intel/intel_rawseed.log");
			odo->setFile("/media/ShareRegion/Datasets/intel/intel_pose.log");

			connect(&mThreadMainSICK,SIGNAL(started()),mainSICK,SLOT(runRS()),Qt::DirectConnection); // for debuging using rawseed data
			connect(&mThreadMinorSICK,SIGNAL(started()),minorSICK,SLOT(runRS()),Qt::DirectConnection);
			connect(&mThreadOdo,SIGNAL(started()),odo,SLOT(runRawseedOdo()),Qt::DirectConnection);


		}
		*/


		/*
		{
			//ourdata in rawseed format, but 151
			mainSICK->setFile("/media/ShareRegion/Datasets/LMS151_20130108/t1.txt");
			minorSICK->setFile("/media/ShareRegion/Datasets/LMS151_20130108/tt1.txt");
			connect(&mThreadMainSICK,SIGNAL(started()),mainSICK,SLOT(runCM()),Qt::DirectConnection); // for debuging using rawseed data
			connect(&mThreadMinorSICK,SIGNAL(started()),minorSICK,SLOT(runCM()),Qt::DirectConnection);
		}
		*/


		/*
		{
			//rawseed, but 211
			mainSICK->setFile("/media/ShareRegion/Datasets/Rawseeds/Bicocca_2009-02-25b/Bicocca_2009-02-25b-SICK_FRONT.csv");
			minorSICK->setFile("/media/ShareRegion/Datasets/Rawseeds/Bicocca_2009-02-25b/Bicocca_2009-02-25b-SICK_REAR.csv");
			odo->setFile("/media/ShareRegion/Datasets/Rawseeds/Bicocca_2009-02-25b/Bicocca_2009-02-25b-ODOMETRY_XYT.csv");

			oriInRobot_mainSICK->x = 0.08;
			oriInRobot_mainSICK->y = 0.;
			oriInRobot_mainSICK->theta = 0.;

			oriInRobot_minorSICK->x = -0.463;
			oriInRobot_minorSICK->y = 0.001;
			oriInRobot_minorSICK->theta = M_PI;

			oriInRobot_odo->x = 0.;
			oriInRobot_odo->y = 0.;
			oriInRobot_odo->theta = 0.;


			connect(&mThreadMainSICK,SIGNAL(started()),mainSICK,SLOT(runRS()),Qt::DirectConnection); // for debuging using rawseed data
			connect(&mThreadMinorSICK,SIGNAL(started()),minorSICK,SLOT(runRS()),Qt::DirectConnection);
			connect(&mThreadOdo,SIGNAL(started()),odo,SLOT(runRawseedOdo()),Qt::DirectConnection);
		}
		*/




	}
	else
	{
		connect(&mThreadMainSICK,SIGNAL(started()),mainSICK,SLOT(runSick()),Qt::DirectConnection);
		connect(&mThreadMinorSICK,SIGNAL(started()),minorSICK,SLOT(runSick()),Qt::DirectConnection);
	}

	connect(mainSICK,SIGNAL(finished()),&mThreadMainSICK,SLOT(quit()),Qt::DirectConnection);
	connect(minorSICK,SIGNAL(finished()),&mThreadMinorSICK,SLOT(quit()),Qt::DirectConnection);
	connect(odo,SIGNAL(finished()),&mThreadOdo,SLOT(quit()),Qt::DirectConnection);

	connect(&mThreadMainSICK,SIGNAL(finished()),this,SLOT(stop()),Qt::DirectConnection);
	connect(&mThreadMinorSICK,SIGNAL(finished()),this,SLOT(stop()),Qt::DirectConnection);
	connect(&mThreadOdo,SIGNAL(finished()),this,SLOT(stop()),Qt::DirectConnection);

	connect(mainSICK,SIGNAL(sendRelFrameInfo(double,double,double,float*,int,TTimeStamp, int, double*)),
			this,SLOT(recvRelMainSICK(double,double,double,float*,int,TTimeStamp, int, double*)),Qt::DirectConnection);
	connect(minorSICK,SIGNAL(sendRelFrameInfo(double,double,double,float*,int,TTimeStamp, int, double*)),
			this,SLOT(recvRelMinorSICK(double,double,double,float*,int,TTimeStamp, int, double*)),Qt::DirectConnection);
	connect(odo,SIGNAL(sendRelOdoNode(double,double,double,TTimeStamp)),
			this,SLOT(recvRelOdo(double,double,double,TTimeStamp)),Qt::DirectConnection);

	//synchronization
	connect(this,SIGNAL(sendMainTime(TTimeStamp)),mainSICK,SLOT(recvTime(TTimeStamp)),Qt::DirectConnection);
	connect(this,SIGNAL(sendMinorTime(TTimeStamp)),minorSICK,SLOT(recvTime(TTimeStamp)),Qt::DirectConnection);
	connect(this,SIGNAL(sendOdoTime(TTimeStamp)),odo,SLOT(getTimeRecvOdo(TTimeStamp)),Qt::DirectConnection);

	mThreadMainSICK.start();
	//mThreadMinorSICK.start();
	//mThreadOdo.start();


	return true;
}

// Run Sensor Fusion
void CClientFusion::runFusion(){
	// initialization
	if(!initFusion())
	{
		cout<<"CClientFusion::runFusion()"<<" initial failed!"<<endl;
		finished();
		return ;
	}

	float* scanForShow = 0;
	int npForShow = 0;
	
	OrientedPoint2D rel_pose;
	OrientedPoint2D sent_pose;
	bool first = true;

	// record sent trajectory
	ofstream traj("fuse_traj.log");

	//while loop
	while(!m_stop_thread)
	{

		QMutexLocker lock_filter(&mutex_filter);
		QMutexLocker lock_main(&mutex_mainSICK);
		//QMutexLocker lock_odo(&mutex_odo);
		//QMutexLocker lock_minor(&mutex_minorSICK);

		if(_MAIN_SICK_READY)// && _ODO_READY)// && _MAIN_SICK_READY)// && _ODO_READY)// && scan_minorSick)
		{

			//printf("main time = %lld, minor time = %lld \n", t_mainSick, t_minorSick);

			scanForShow = scan_mainSick;
			npForShow = np_mainSick;
			//scanForShow = scan_minorSick;
			//npForShow = np_minorSick;
			//prediction
			my_filter->predict(robot_predict);
			my_filter->update();		// Update the filter, so state and covariance are available
			cvtX2OPD(my_filter->x, *opd_x_p); //cm to m

			//update
			updateMainSICKNode();
			//updateMinorSICKNode();
			//updateOdoNode();

			//send frame to server
			cvtX2OPD(my_filter->x, *opd_x_u);
			//sendFrameInfo(opd_x_u->x,opd_x_u->y,opd_x_u->theta, scanForShow, npForShow, syn_mainSick);
			if(cov_mainSick)
			{
				cov_mainSick[0] = my_filter->X(0,0)/100.;
				cov_mainSick[3] = my_filter->X(1,1)/100.;
				cov_mainSick[5] = deg2rad((double)my_filter->X(2,2));
			}
			n_steps++;
			//sendFrameInfo(opd_x_u->x,opd_x_u->y,opd_x_u->theta, scanForShow, npForShow, syn_mainSick);

			// whether to send this frame
			rel_pose = sent_pose.ominus(*opd_x_u);
			if(first || !mainSICK->smallMove(rel_pose)){
				sendFrameInfoCov(opd_x_u->x,opd_x_u->y,opd_x_u->theta, scanForShow, npForShow, syn_mainSick, cov_mainSick);
				sent_pose = *opd_x_u;
				// record fused trajectory
				traj<<sent_pose<<endl; 
				first = false;
			}
			// cout<<"CClientFusion::runFusion "<<opd_x_u->x<<" "<<opd_x_u->y<<" "<<opd_x_u->theta<<endl;
		}
		else
		{
			QThread::yieldCurrentThread();
			continue;
		}

	}

	finished();
}

void CClientFusion::updateMainSICKNode(){

	//cout<<"CClientFusion::updateMainSICKNode()"<<endl;

	relpToRobotFrame(*oriInRobot_mainSICK, *relp_mainSick);

	*opd_x_u = opd_x_p->oplus(*relp_mainSick);
	Vec z(3);
	cvtOPD2X(*opd_x_u, z);


	if(cov_mainSick)
	{
		sick_observe_main.Zv[0] = cov_mainSick[0]*100;
		sick_observe_main.Zv[1] = cov_mainSick[3]*100;
		sick_observe_main.Zv[2] = rad2deg(cov_mainSick[5]);
	}

	my_filter->observe (sick_observe_main, z);
	my_filter->update();

	_MAIN_SICK_READY = false;
	scan_mainSick = 0;
	sendMainTime(t_mainSick);


}

void CClientFusion::updateMinorSICKNode(){

	cout<<"CClientFusion::updateMinorSICKNode()"<<endl;

	relpToRobotFrame(*oriInRobot_minorSICK, *relp_minorSick);

	*opd_x_u = opd_x_p->oplus(*relp_minorSick);
	Vec z(3);
	cvtOPD2X(*opd_x_u, z);

	if(cov_minorSick)
	{
		sick_observe_minor.Zv[0] = cov_minorSick[0]*100;
		sick_observe_minor.Zv[1] = cov_minorSick[3]*100;
		sick_observe_minor.Zv[2] = rad2deg(cov_minorSick[5]);
	}

	my_filter->observe (sick_observe_minor, z);
	my_filter->update();

	scan_minorSick = 0;
	_MINOR_SICK_READY = false;
	sendMinorTime(t_minorSick);

}


void CClientFusion::updateOdoNode(){

	cout<<"CClientFusion::updateOdoNode()"<<endl;

	relpToRobotFrame(*oriInRobot_odo, *relp_odo);

	*opd_x_u = opd_x_p->oplus(*relp_odo);
	Vec z(3);
	cvtOPD2X(*opd_x_u, z);

	my_filter->observe (odo_observe, z);
	my_filter->update();

	_ODO_READY = false;
	sendOdoTime(t_odo);

}

/*Receive Relative Pose from Main SICK Module*/
void CClientFusion::recvRelMainSICK(double x,double y,double theta,float* scan, int np, TTimeStamp t, int syn_num, double* cov){
	// printf("CClientFusion::recvRelMainSICK: timestamp %lld \n",t);
	// cout<<"CClientFusion::recvRelMainSICK:"<<x<<" "<<y<<" "<<theta<<endl;

	QMutexLocker lock(&mutex_mainSICK);
	relp_mainSick->x = x;
	relp_mainSick->y = y;
	relp_mainSick->theta = theta;
	scan_mainSick = scan;
	np_mainSick = np;
	t_mainSick = t;
	// syn_mainSick = syn_num;
	// cov_mainSick = cov;
	memcpy(cov_mainSick,cov,6*sizeof(double));
	_MAIN_SICK_READY = true;
}

/*Receive Relative Pose from Main SICK Module*/
void CClientFusion::recvRelMinorSICK(double x,double y,double theta,float* scan, int np, TTimeStamp t, int syn_num, double* cov){
	printf("CClientFusion::recvRelMinorSICK: timestamp %lld \n",t);
	cout<<"CClientFusion::recvRelMinorSICK" <<x<<" "<<y<<" "<<theta<<endl;
	QMutexLocker lock(&mutex_minorSICK);
	relp_minorSick->x = x;
	relp_minorSick->y = y;
	relp_minorSick->theta = theta;
	scan_minorSick = scan;
	np_minorSick = np;
	t_minorSick = t;
	syn_minorSick = syn_num;
	cov_minorSick = cov;
	_MINOR_SICK_READY = true;
}

void CClientFusion::recvRelOdo(double x,double y,double theta, TTimeStamp t){
	printf("CClientFusion::recvRelOdo: timestamp %lld \n",t);
	cout<<"CClientFusion::recvRelOdo" <<x<<" "<<y<<" "<<theta<<endl;
	QMutexLocker lock(&mutex_odo);
	relp_odo->x = x;
	relp_odo->y = y;
	relp_odo->theta = theta;
	t_odo = t;
	_ODO_READY = true;
}


/*Receive Relative Pose from Main SICK Module*/
void CClientFusion::recvRelGlobalOptimized(int id, void* pose, double* cov){
	cout<<"CClientFusion::recvRelGlobalOptimized " <<id<<endl;

	memcpy(cov_Global,cov,6*sizeof(double));
	cout<<"Fusion cov: "<<cov_Global[0]<<" "<<cov_Global[3]<<" "<<cov_Global[5]<<endl; 


	QMutexLocker lock(&mutex_filter);
	OrientedPoint2D* p = static_cast<OrientedPoint2D*>(pose);
	if(p==NULL) return ;
	if(id < 0 || id >= n_steps){
		cout<<"error in CClientFrontend::recvRelGlobalOptimized!"<<endl;
		return ;
	}

	//update filt
	if(1){
		synGlobal_observe.Zv[0] = cov_Global[0]*100;
		synGlobal_observe.Zv[1] = cov_Global[3]*100;
		synGlobal_observe.Zv[2] = rad2deg(cov_Global[5]);
	}
	*opd_x_u = *opd_x_p + *p;
	Vec z(3);
	cvtOPD2X(*opd_x_u, z);

	my_filter->observe (synGlobal_observe, z);
	my_filter->update();

	syn_mainSick++;

}


/* trans the relative pose to the robot frame;*/
void CClientFusion::relpToRobotFrame(const OrientedPoint2D oriInRobot, OrientedPoint2D &relp)
{
	double x = relp.x;
	double y = relp.y;
	double ctheta = cos(oriInRobot.theta), stheta = sin(oriInRobot.theta);
	relp.x = x * ctheta - y * stheta;
	relp.y = x * stheta + y * ctheta;
}

/*Convert OrientedPoint2D to FM::Vec format.
 * The angle will be normalized to [-pi, pi] first*/
void CClientFusion::cvtOPD2X(OrientedPoint2D& opd, FM::Vec & x)
{
	if(x.size()!=3)
	{
		cout<<"ERROR: cvtOPD2X(); x dimission must be 3!"<<endl;
		return ;
	}
	opd.theta = normAngle(opd.theta, -M_PI);

	x[0] = opd.x * 100; //metre to cm
	x[1] = opd.y * 100;
	x[2] = rad2deg(opd.theta);// * rad2deg; //rad to degree
}

// unit and format conversion
void CClientFusion::cvtX2OPD(FM::Vec& x, OrientedPoint2D& opd)
{
	if(x.size()!=3)
	{
		cout<<"ERROR: cvtX2OPD(); x dimission must be 3!"<<endl;
		return ;
	}

	opd.x = x[0] / 100; // cm to m
	opd.y = x[1] / 100;
	opd.theta = normAngle(deg2rad(x[2]), -M_PI);// * D2R; // degree to rad

	x[2] = rad2deg(opd.theta);
}
