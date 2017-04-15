/*
 * clientFusion.h
 *
 *  Created on: Jan 11, 2013
 *      Author: liu
 */

#ifndef CLIENTFUSION_H_
#define CLIENTFUSION_H_

#include <QObject>
#include <QMutex>
#include <QThread>
#include <string>
#include <vector>

#include "clientFrontend.h"
#include "unsFlt.hpp"
#include <boost/numeric/ublas/io.hpp>
#include "config.hpp"

class CFrontendSICK;
class CFrontendOdo;

class CClientFusion : public CClientFrontend
{
	Q_OBJECT
public:
	CClientFusion();
	~CClientFusion();

public Q_SLOTS:
	void recvRelMainSICK(double,double,double,float*,int,TTimeStamp,int,double*);
	void recvRelMinorSICK(double,double,double,float*,int,TTimeStamp,int,double*);
	void recvRelGlobalOptimized(int id, void* pose, double*);
	void recvRelOdo(double , double , double , TTimeStamp);
	//void recvRelGPS();
Q_SIGNALS:
	void sendMainTime(TTimeStamp);
	void sendMinorTime(TTimeStamp);
	void sendOdoTime(TTimeStamp);

public:
	bool initFusion();
	void runFusion(); //model 3
	void updateSynGlobalNode();
	void updateMainSICKNode();
	void updateMinorSICKNode();
	void updateGlobalOptimizedNode();
	void updateOdoNode();

	void relpToRobotFrame(const zhpsm::OrientedPoint2D, zhpsm::OrientedPoint2D &);
	void cvtOPD2X(zhpsm::OrientedPoint2D& opd, FM::Vec & x);
	void cvtX2OPD(FM::Vec& x, zhpsm::OrientedPoint2D& opd);

	CFrontendOdo* odo;
	CFrontendSICK* mainSICK;
	CFrontendSICK* minorSICK;
	QThread mThreadOdo;
	QThread mThreadMinorSICK;
	QThread mThreadMainSICK;

	//GLOBAL SETTING
	double * cov_Global;

	//FILTER SETTING
	int x_dim;
	int n_steps; // number of iteration
	Unscented_scheme* my_filter; //using unscented filters
	Robot_predict robot_predict;
	SynGlobal_observe synGlobal_observe;
	SICK_observe sick_observe_main;
	SICK_observe sick_observe_minor;
	GPS_observe gps_observe;
	ODO_observe odo_observe;
	BN_observe bn_observe;

	//SENSOR SETTING
	zhpsm::OrientedPoint2D* opd_x_p;
	zhpsm::OrientedPoint2D* opd_x_u;
	zhpsm::OrientedPoint2D* oriInRobot_odo;
	zhpsm::OrientedPoint2D* oriInRobot_minorSICK;
	zhpsm::OrientedPoint2D* oriInRobot_mainSICK;


	//SICK SETTING
	zhpsm::OrientedPoint2D* relp_mainSick;
	float* scan_mainSick;
	int np_mainSick;
	TTimeStamp t_mainSick;
	int syn_mainSick;
	double* cov_mainSick;
	zhpsm::OrientedPoint2D* relp_minorSick;
	float* scan_minorSick;
	int np_minorSick;
	TTimeStamp t_minorSick;
	int syn_minorSick;
	double* cov_minorSick;

	//ODO SETTING
	zhpsm::OrientedPoint2D* relp_odo;
	TTimeStamp t_odo;


	volatile bool _MAIN_SICK_READY;
	volatile bool _MINOR_SICK_READY;
	volatile bool _ODO_READY;
	volatile bool _GPS_READY;

	//LOCKERS
	QMutex mutex_filter;
	QMutex mutex_mainSICK;
	QMutex mutex_minorSICK;
	QMutex mutex_odo;
};


#endif /* CLIENTFUSION_H_ */
