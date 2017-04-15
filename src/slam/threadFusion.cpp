/*
 * threadFusion.cpp
 *
 *  Created on: Dec 18, 2012
 *      Author: liu
 */



#include "threadFusion.h"
#include "FlirterNode.h"
#include "ZHPolar_Match.h"
#include "ZHCanonical_Matcher.h"
#include "MapNode.h"



ThreadFusion::ThreadFusion():
my_filter(0),
fusedSICKNode(0),
mainSICKNode(0),
minorSICKNode(0),
odoNode(0),
t_filter_current(0),
t_mainSICKNode_last(0),
t_minorSICKNode_last(0),
t_odoNode_last(0),
m_stop_thread(false),
m_pause_thread(false),
opd_x_p(new OrientedPoint2D),
opd_x_u(new OrientedPoint2D),
oriInRobot_odo(new OrientedPoint2D),
oriInRobot_minorSICK(new OrientedPoint2D),
oriInRobot_mainSICK(new OrientedPoint2D),
synGlobal_pose(0)
{
	// Use an 'Unscented' filter scheme with one state

	my_filter = new Unscented_scheme(3); // 3 is the dimension of the state

	// Setup the initial state and covariance
	Vec x_init (3);
	SymMatrix X_init (3, 3);
	x_init[0] = 0.;		// Start at 10 with no uncertainty
	x_init[1] = 0.;
	x_init[2] = 0.;
	X_init = ublas::zero_matrix<double>(3,3);
	my_filter->init_kalman (x_init, X_init);
	cout << "Initial  " << my_filter->x << my_filter->X << endl;
}
ThreadFusion::~ThreadFusion(){
	if(fusedSICKNode!=0) delete fusedSICKNode;
	if(mainSICKNode!=0) delete mainSICKNode;
	if(minorSICKNode!=0) delete minorSICKNode;
	if(odoNode!=0) delete odoNode;
	if(my_filter!=0) delete my_filter;
	if(opd_x_p) delete opd_x_p;
	if(opd_x_u) delete opd_x_u;
	if(oriInRobot_odo) delete oriInRobot_odo;
	if(oriInRobot_minorSICK) delete oriInRobot_minorSICK;
	if(oriInRobot_mainSICK) delete oriInRobot_mainSICK;
}

// unit and format conversion
void ThreadFusion::cvtOPD2X(const OrientedPoint2D opd, FM::Vec & x)
{
	if(x.size()!=3)
	{
		cout<<"ERROR: cvtOPD2X(); x dimission must be 3!"<<endl;
		return ;
	}
	x[0] = opd.x * 100; //metre to cm
	x[1] = opd.y * 100;
	x[2] = opd.theta * R2D; //rad to degree
}

// unit and format conversion
void ThreadFusion::cvtX2OPD(const FM::Vec x, OrientedPoint2D& opd)
{
	if(x.size()!=3)
	{
		cout<<"ERROR: cvtX2OPD(); x dimission must be 3!"<<endl;
		return ;
	}
	opd.x = x[0] / 100; // cm to m
	opd.y = x[1] / 100;
	opd.theta = x[2] * D2R; // degree to rad
}
void ThreadFusion::prepareFusedNode_online(){

	cout<<"ThreadFusion::prepareFusedNode_online - ID: "<<(int)QThread::currentThreadId()<<endl;


	while(!m_stop_thread){

		updateSynGlobalNode();

		QMutexLocker locker_1(&mutex_mainSICK);
		{
			QMutexLocker locker_2(&mutex_minorSICK);
			QMutexLocker locker_3(&mutex_odo);
			if(mainSICKNode == NULL )//||  minorSICKNode == NULL) //|| odoNode == NULL )
			{
				QThread::yieldCurrentThread();
				continue;
			}
		}

		//predict
		my_filter->predict (robot_predict);
		my_filter->update();		// Update the filter, so state and covariance are available
		cvtX2OPD(my_filter->x,* opd_x_p);

		cout<<"ThreadFusion::prepareFusedNode - Prediction: "<<opd_x_p->x<<" "<<opd_x_p->y<<" "<<opd_x_p->theta<<endl;

		//update
		updateMainSICKNode();
		//updateMinorSICKNode();
		//updateOdoNode();

		//Finished and send the fused node

		cvtX2OPD(my_filter->x, *opd_x_u);
		mainSICKNode->m_pose = *opd_x_u;
		mainSICKNode->m_relpose = opd_x_p->ominus(*opd_x_u); //need be sure??

		//insert to the graph
		if(m_graph.size()<=0)
		{
			m_graph.insert(make_pair(0,mainSICKNode));
		}
		else {
			m_graph.insert(make_pair(m_graph.size(),mainSICKNode));
		}


		// TODO: send pcurNode to display this frame
		vector<float> obs_x;
		vector<float> obs_y;

		//translate2GlobalFrame(mainSICKNode->m_pScan->r,obs_x,obs_y,mainSICKNode->m_pose.x,mainSICKNode->m_pose.y,mainSICKNode->m_pose.theta);
		cout<<mainSICKNode->m_pFMatch->m_pParam->pm_l_points<<" "<<mainSICKNode->m_pScan->np<<" "<<mainSICKNode->m_pFMatch2->m_pParam->pm_l_points<<endl;
		translate2GlobalFrame(mainSICKNode,obs_x,obs_y);
		sendObservation(&obs_x[0],&obs_y[0],(int)obs_x.size(),&(mainSICKNode->m_pose.x),&(mainSICKNode->m_pose.y),&(mainSICKNode->m_pose.theta));
		paintReady();

		// send the node at the last step, not before processing done!!!

		TTimeStamp mainSICKNode_t = mainSICKNode->m_timestamp;
		sendTimeMainSICK(mainSICKNode_t);
		sendFusedNode(mainSICKNode);
		mainSICKNode = NULL;
	}
	finished();
}

void ThreadFusion::prepareFusedNode_rawseed(){

	cout<<"ThreadFusion::prepareFusedNode_rawseed - ID: "<<(int)QThread::currentThreadId()<<endl;

	// Setup the initial state and covariance
	while(!m_stop_thread){

		updateSynGlobalNode();

		QMutexLocker locker_1(&mutex_mainSICK);
		{
			QMutexLocker locker_2(&mutex_minorSICK);
			QMutexLocker locker_3(&mutex_odo);
			if(mainSICKNode == NULL || minorSICKNode == NULL )  //|| odoNode == NULL ) //)//
			{
				QThread::yieldCurrentThread();
				continue;
			}
		}

		//predict
		my_filter->predict (robot_predict);
		my_filter->update();		// Update the filter, so state and covariance are available
		cvtX2OPD(my_filter->x,* opd_x_p);

		cout<<"ThreadFusion::prepareFusedNode - Prediction: "<<opd_x_p->x<<" "<<opd_x_p->y<<" "<<opd_x_p->theta<<endl;

		//update
		updateMainSICKNode();
		updateMinorSICKNode();
		//updateOdoNode();

		//Finished and send the fused node

		cvtX2OPD(my_filter->x, *opd_x_u);
		mainSICKNode->m_pose = *opd_x_u;
		mainSICKNode->m_relpose = opd_x_p->ominus(*opd_x_u); //need be sure??

		//insert to the graph
		if(m_graph.size()<=0)
		{
			m_graph.insert(make_pair(0,mainSICKNode));
		}
		else {
			m_graph.insert(make_pair(m_graph.size(),mainSICKNode));
		}

		// TODO: send pcurNode to display this frame
		vector<float> obs_x;
		vector<float> obs_y;

		//translate2GlobalFrame(mainSICKNode->m_pScan->r,obs_x,obs_y,mainSICKNode->m_pose.x,mainSICKNode->m_pose.y,mainSICKNode->m_pose.theta);
		cout<<mainSICKNode->m_pFMatch->m_pParam->pm_l_points<<" "<<mainSICKNode->m_pScan->np<<" "<<mainSICKNode->m_pFMatch2->m_pParam->pm_l_points<<endl;

		translate2GlobalFrame(mainSICKNode,obs_x,obs_y);
		sendObservation(&obs_x[0],&obs_y[0],(int)obs_x.size(),&(mainSICKNode->m_pose.x),&(mainSICKNode->m_pose.y),&(mainSICKNode->m_pose.theta));
		paintReady();

		// send the node at the last step, not before processing done!!!

		TTimeStamp mainSICKNode_t = mainSICKNode->m_timestamp;
		sendTimeMainSICK(mainSICKNode_t);
		sendFusedNode(mainSICKNode);
		mainSICKNode = NULL;
	}
	finished();
}

void ThreadFusion::updateMainSICKNode()
{

	if(mainSICKNode != NULL && !m_pause_thread)
	{
		cout<<"XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"<<endl;
		cout<<"ThreadFusion::updateMainSICKNode"<<endl;

		double t_diff = timeDifference(t_filter_current, mainSICKNode->m_timestamp);
		cout<<"updateMainSICKNode - Time Difference: "<<t_diff<<endl;

		//do fusion
		//update timestamp
		t_filter_current = mainSICKNode->m_timestamp;
		//update measurement
		OrientedPoint2D relp;
		double ctheta = cos(oriInRobot_mainSICK->theta), stheta = sin(oriInRobot_mainSICK->theta);
		relp.x = mainSICKNode->m_relpose.x * ctheta - mainSICKNode->m_relpose.y * stheta;
		relp.y = mainSICKNode->m_relpose.x * stheta + mainSICKNode->m_relpose.y * ctheta;
		relp.theta = mainSICKNode->m_relpose.theta;
		*opd_x_u = opd_x_p->oplus(relp);//mainSICKNode->m_relpose);
		Vec z_2(3);
		cvtOPD2X(*opd_x_u, z_2);
		pi2pi(my_filter->x[2]);
		pi2pi(z_2[2]);
		my_filter->observe (sick_observe_main, z_2);
		my_filter->update();

		cout<<"updateMainSICKNode - Measurement MainSICK (cm) relp: "<<relp.x*100<<" "<<relp.y*100<<" "<<relp.theta*R2D<<endl;
		cout<<"updateMainSICKNode - Measurement MainSICK (cm) relpose: "<<mainSICKNode->m_relpose.x*100<<" "<<mainSICKNode->m_relpose.y*100<<" "<<mainSICKNode->m_relpose.theta*R2D<<endl;
		cout<<"updateMainSICKNode - Measurement MainSICK (cm) pose: "<<mainSICKNode->m_pose.x*100<<" "<<mainSICKNode->m_pose.y*100<<" "<<mainSICKNode->m_pose.theta*R2D<<endl;
		cout<<"updateMainSICKNode - Update FILTER (cm): "<<my_filter->x<<" "<<my_filter->X<<endl;

	}

}

void ThreadFusion::updateMinorSICKNode()
{

	QMutexLocker locker(&mutex_minorSICK);

	if(minorSICKNode != NULL)
	{
		cout<<"XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"<<endl;
		cout<<"ThreadFusion::updateMinorSICKNode"<<endl;

		double t_diff = timeDifference(t_filter_current, minorSICKNode->m_timestamp);
		cout<<"minorSICKNode - Time Difference: "<<t_diff<<endl;

		//update measurement
		OrientedPoint2D relp;
		double ctheta = cos(oriInRobot_minorSICK->theta), stheta = sin(oriInRobot_minorSICK->theta);
		relp.x = minorSICKNode->m_relpose.x * ctheta - minorSICKNode->m_relpose.y * stheta;
		relp.y = minorSICKNode->m_relpose.x * stheta + minorSICKNode->m_relpose.y * ctheta;
		relp.theta = mainSICKNode->m_relpose.theta;

		*opd_x_u = opd_x_p->oplus(relp);//minorSICKNode->m_relpose);
		Vec z_2(3);
		cvtOPD2X(*opd_x_u, z_2);
		pi2pi(my_filter->x[2]);
		pi2pi(z_2[2]);
		my_filter->observe (sick_observe_minor, z_2);
		my_filter->update();

		cout<<"updateMinorSICKNode - Measurement MinorSICK (cm) relp: "<<relp.x*100<<" "<<relp.y*100<<" "<<relp.theta*R2D<<endl;
		cout<<"updateMinorSICKNode - Measurement MinorSICK (cm) relpose: "<<minorSICKNode->m_relpose.x*100<<" "<<minorSICKNode->m_relpose.y*100<<" "<<minorSICKNode->m_relpose.theta*R2D<<endl;
		cout<<"updateMinorSICKNode - Measurement MinorSICK (cm) pose: "<<minorSICKNode->m_pose.x*100<<" "<<minorSICKNode->m_pose.y*100<<" "<<minorSICKNode->m_pose.theta*R2D<<endl;
		cout<<"updateMinorSICKNode - Update FILTER (cm): "<<my_filter->x<<" "<<my_filter->X<<endl;

		sendTimeMinorSICK(minorSICKNode->m_timestamp);
		minorSICKNode = NULL;

	}
}

void ThreadFusion::updateOdoNode()
{

	QMutexLocker locker(&mutex_odo);
	if(odoNode != NULL)
	{
		cout<<"XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"<<endl;
		cout<<"ThreadFusion::updateOdoNode"<<endl;

		double t_diff = timeDifference(t_filter_current, odoNode->m_timestamp);
		cout<<"updateOdoNode - Time Difference: "<<t_diff<<endl;

		if(fabs(t_diff)<0.02)
		{
			//update measurement
			OrientedPoint2D relp;
			double ctheta = cos(oriInRobot_odo->theta), stheta = sin(oriInRobot_odo->theta);
			relp.x = odoNode->m_relpose->x * ctheta - odoNode->m_relpose->y * stheta;
			relp.y = odoNode->m_relpose->x * stheta + odoNode->m_relpose->y * ctheta;
			relp.theta = odoNode->m_relpose->theta;

			*opd_x_u = opd_x_p->oplus(relp);//*(odoNode->m_relpose));
			Vec z_2(3);
			cvtOPD2X(*opd_x_u, z_2);
			pi2pi(my_filter->x[2]);
			pi2pi(z_2[2]);
			my_filter->observe (odo_observe, z_2);
			my_filter->update();

			cout<<"updateOdoNode - Measurement Odometry (cm) relp: "<<relp.x*100<<" "<<relp.y*100<<" "<<relp.theta*R2D<<endl;
			cout<<"updateOdoNode - Measurement Odometry relpose: "<<odoNode->m_timestamp<<" "<<odoNode->m_relpose->x*100.<<" "<<odoNode->m_relpose->y*100.<<" "<<odoNode->m_relpose->theta*R2D<<endl;
			cout<<"updateOdoNode - Measurement Odometry pose: "<<odoNode->m_timestamp<<" "<<odoNode->m_pose->x*100.<<" "<<odoNode->m_pose->y*100.<<" "<<odoNode->m_pose->theta*R2D<<endl;
			cout<<"updateOdoNode - Update FILTER: "<<my_filter->x<<" "<<my_filter->X<<endl;
			sendTimeOdo(odoNode->m_timestamp);
			odoNode = NULL;
		}

	}
}

void ThreadFusion::updateSynGlobalNode()
{

	QMutexLocker locker(&mutex_synGlobal);
	if(synGlobal_pose != NULL)
	{
		cout<<"XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"<<endl;
		cout<<"ThreadFusion::updateSynGlobalNode"<<endl;

		map<int, CFliterNode*>::iterator it = m_graph.find(synGlobal_pose_ID);
		if(it == m_graph.end()){
			cerr<<"error in synFromGlobal!"<<endl;
			return ;
		}
		// only update the last node is enough
		// it->second->m_pose = *new_pose;
		CFliterNode* last = m_graph.rbegin()->second;
		//last->m_pose  = last->m_pose.oplus(it->second->m_pose.ominus(*new_pose));

		OrientedPoint2D opd_syn  = last->m_pose.oplus(it->second->m_pose.ominus(*synGlobal_pose));

		//check the angle returned from global
		Vec z(3);
		cvtOPD2X(opd_syn, z);
		cout<<"updateSynGlobalNode - SynGlobal Prediction: "<<my_filter->x<<" "<<my_filter->X<<endl;
		pi2pi(z[2]);
		my_filter->observe (synGlobal_observe, z);
		my_filter->update();

		cout<<"updateSynGlobalNode - SynGlobal Measurement (cm): "<<opd_syn.x*100<<" "<<opd_syn.y*100<<" "<<opd_syn.theta*R2D<<endl;
		cout<<"updateSynGlobalNode - SynGlobal Update (cm): "<<my_filter->x<<" "<<my_filter->X<<endl;

		cvtX2OPD(my_filter->x, last->m_pose);
		cleanObservation((double)last->m_pose.x,(double)last->m_pose.y);

		synGlobal_pose = NULL;
	}

}

void ThreadFusion::recvMainSICKNode(void* param)
{
	QMutexLocker locker(&mutex_mainSICK);
	mainSICKNode = static_cast<CFliterNode*>(param);

	if(mainSICKNode == NULL){
		cout<<"Error in recvMainSICKNode!"<<endl;
		return;
	}

	cout<<"//////////////////////////"<<endl;
	cout<<"ThreadFusion::recvMainSICKNode - ID: "<<(int)QThread::currentThreadId()<<endl;
	cout<<"ThreadFusion::recvMainSICKNode - Timestamp: "<<mainSICKNode->m_timestamp<<endl;
}

void ThreadFusion::recvMinorSICKNode(void* param)
{

	QMutexLocker locker(&mutex_minorSICK);

	//cout<<"recvMinorSICKNode: received a node! "<<(int)QThread::currentThreadId()<<endl;
	minorSICKNode = static_cast<CFliterNode*>(param);
	if(minorSICKNode == NULL){
		cout<<"Error in recvMinorSICKNode!"<<endl;
		return;
	}


	cout<<"//////////////////////////"<<endl;
	cout<<"ThreadFusion::recvMinorSICKNode - ID: "<<(int)QThread::currentThreadId()<<endl;
	cout<<"ThreadFusion::recvMinorSICKNode - Timestamp: "<<minorSICKNode->m_timestamp<<endl;
}

void ThreadFusion::recvOdoNode(void* param)
{

	QMutexLocker locker(&mutex_odo);
	odoNode = static_cast<COdoNode*>(param);
	if(odoNode == NULL){
		cout<<"Error in recvMinorSICKNode!"<<endl;
		return;
	}

	cout<<"//////////////////////////"<<endl;
	cout<<"ThreadFusion::recvOdoNode - ID: "<<(int)QThread::currentThreadId()<<endl;
	cout<<"ThreadFusion::recvOdoNode - Timestamp: "<<odoNode->m_timestamp<<endl;

}

void ThreadFusion::stopThreadFusion(){
	// cout<<"thread: "<<(int)QThread::currentThreadId()<<" stop threadLocal1"<<endl;
	m_stop_thread = true;
	finished();
}


void ThreadFusion::synFromGlobal(int id, void* pose)
{


	QMutexLocker locker(&mutex_synGlobal);
	synGlobal_pose = static_cast<OrientedPoint2D*> (pose);
	if(synGlobal_pose == NULL){
		cout<<"ThreadFusion::synFromGlobal - error in synFromGlobal!"<<endl;
		return ;
	}
	else
	{
		synGlobal_pose_ID = id;
	}

	cout<<"ThreadFusion::synFromGlobal - graph.size(): "<<m_graph.size()<<", id: "<<id<<endl;

}

//void ThreadFusion::translate2GlobalFrame(float* bearing, vector<float>& fx,vector<float>& fy, double rx, double ry, double th)
void ThreadFusion::translate2GlobalFrame(void* param, vector<float>& fx,vector<float>& fy)
{
	CFliterNode* fliterNode = static_cast<CFliterNode*>(param);
	const float bad_range = -100000;
	int num_points = fliterNode->m_pFMatch->m_pParam->pm_l_points;
	int max_range = fliterNode->m_pFMatch->m_pParam->pm_max_range;
	float rx = fliterNode->m_pose.x;
	float ry = fliterNode->m_pose.y;
	float th = fliterNode->m_pose.theta;

	if(fx.size()!= num_points){
		fx.resize(num_points);
		fy.resize(num_points);
	}

	if(num_points!=fliterNode->m_pScan->np){
		cout<<"ERROR: num_points: "<<num_points<<" but r.size()"<<fliterNode->m_pScan->np;
	}

	float x,y,tx,ty,nIdx,nIdy;
	float fcos = cosf(th);
	float fsin = sinf(th);
	for(int i=0;i<num_points;i++){
		if(fliterNode->m_pScan->r[i] > max_range){ // this is bad range
			fx[i]=fy[i]=bad_range;
			continue;
		}
		x=fliterNode->m_pScan->x[i]/100.;
		y=fliterNode->m_pScan->y[i]/100.;
		tx=fcos*x-fsin*y+rx;
		ty=fsin*x+fcos*y+ry;

		fx[i] = tx;
		fy[i] = ty;
	}

}

void ThreadFusion::translate2RobotFrame(const OrientedPoint2D* oriInRobot, OrientedPoint2D* newPose)
{
	float fcos = cosf(oriInRobot->theta);
	float fsin = sinf(oriInRobot->theta);

	double tx, ty, ttheta;
	tx = fcos*newPose->x - fsin*newPose->y + oriInRobot->x;
	ty = fsin*newPose->x + fcos*newPose->y + oriInRobot->y;
	ttheta = newPose->theta + oriInRobot->theta;

	newPose->x = tx;
	newPose->y = ty;
	newPose->theta = ttheta;

}

//theta in [-M_PI, M_PI] in degrees
void ThreadFusion::pi2pi(float& theta){

	theta = fmod(theta, M_2PI*R2D);

	if(theta > M_PI*R2D)
	{
		theta = theta - M_2PI*R2D;
	}
	else if(theta < -M_PI*R2D)
	{
		theta = theta + M_2PI*R2D;
	}
}

void ThreadFusion::pi2pi(double& theta){

	theta = fmod(theta, M_2PI*R2D);

	if(theta > M_PI*R2D)
	{
		theta = theta - M_2PI*R2D;
	}
	else if(theta < -M_PI*R2D)
	{
		theta = theta + M_2PI*R2D;
	}
}


