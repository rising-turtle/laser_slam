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

extern float m_fSLAMParams[15];
extern bool _EXTERN_SLAM_PARAM;

class CFrontendSICK;
//class LocalV1;

class CClientFusion : public CClientFrontend
{
	Q_OBJECT
public:
	CClientFusion();
	~CClientFusion();

public Q_SLOTS:
	void recvRelMainSICK(double,double,double,float*,int,TTimeStamp,double*,int,bool);
	void recvRelMainSICK2(double,double,double,void*,int,bool);
	void recvRelMinorSICK(double,double,double,float*,int,TTimeStamp,double*,int,bool);
	void recvRelMinorSICK2(double,double,double,void*,int,bool);
	void recvBN(float,float,float);
	void recvODO(float,float,float,float);

	void recvLocalizedPose(int id, void* pose, bool);
	void recvRelGlobalOptimized(int id, void* pose, double* cov);
	//void recvRelOdo();
	//void recvRelGPS();
Q_SIGNALS:
	void sendODO(float,float,float,float);
	void sendBN(float,float,float);
	void sendLD(float,float,float); // Localization data

public:
	void setGlobalParam( bool use_extern_param);
	bool initFusion();
	void runFusionInfo(); //model 3 // odo first // unscented information filter for sensor fusion
	void runFusionPara(); //model 3 // anyone can be first // unscented information filter for sensor fusion
	void runODO(); //model 4
	void runBN(); // model 5
	void updateSynGlobalNode();
	void updateMainSICKNode();
	void updateMinorSICKNode();
	void updateOdoNode();
	void updateBNNode();

	void relpToRobotFrame(const zhpsm::OrientedPoint2D, zhpsm::OrientedPoint2D &);
	void cvtOPD2X(zhpsm::OrientedPoint2D& opd, FM::Vec & x);
	void cvtX2OPD(FM::Vec& x, zhpsm::OrientedPoint2D& opd);

	CFrontendSICK* mainSICK;
	CFrontendSICK* minorSICK;
	QThread mThreadMainSICK;
	QThread mThreadMinorSICK;

	// SYN_NUM 
	// int main_syn;
	// int minor_syn;

	//FILTER SETTING
	int x_dim;
	int n_steps; // number of iteration
	Unscented_scheme* my_filter; //using unscented filters
	Robot_predict robot_predict;
	SynGlobal_observe synGlobal_observe;
	SynLocalize_observe synLocalize_observe;
	SICK_observe sick_observe_main;
	SICK_observe sick_observe_minor;
	//GPS_observe gps_observe;
	ODO_observe odo_observe;
	BN_observe bn_observe;


	//SENSOR SETTING
	zhpsm::OrientedPoint2D* opd_x_p;
	zhpsm::OrientedPoint2D* opd_x_u;
	zhpsm::OrientedPoint2D* oriInRobot_odo;
	zhpsm::OrientedPoint2D* oriInRobot_minorSICK;
	zhpsm::OrientedPoint2D* oriInRobot_mainSICK;

	//SICK SETTING
	struct _PMScan* m_pFrame_mainSick;
	zhpsm::OrientedPoint2D* absPose_mainSick;
	zhpsm::OrientedPoint2D* relp_mainSick;
	zhpsm::OrientedPoint2D* mainSick_xp;
	zhpsm::OrientedPoint2D* mainSick_xu;
	float* scan_mainSick;
	int np_mainSick;
	TTimeStamp t_mainSick;
	double* cov_mainSick;
	int nmovecnt_main;
	bool mainSick_valid;

	struct _PMScan* m_pFrame_minorSick;
	zhpsm::OrientedPoint2D* absPose_minorSick;
	zhpsm::OrientedPoint2D* relp_minorSick;
	zhpsm::OrientedPoint2D* minorSick_xp;
	zhpsm::OrientedPoint2D* minorSick_xu;
	float* scan_minorSick;
	int np_minorSick;
	TTimeStamp t_minorSick;
	double* cov_minorSick;
	int nmovecnt_minor;
	bool minorSick_valid;

	//BN SETTING
	zhpsm::OrientedPoint2D* absPose_bn;

	//ODO SETTING
	zhpsm::OrientedPoint2D* absPose_odo_cur;
	float odo_t_cur;
	zhpsm::OrientedPoint2D* absPose_odo_last;
	float odo_t_last;
	zhpsm::OrientedPoint2D* relPose_odo;
	zhpsm::OrientedPoint2D* odo_xp;
	zhpsm::OrientedPoint2D* odo_xu;

	//GLOBAL SETTING
	double * cov_Global;
	int m_syn_num;
	double* m_cov;
	double* m_cov_inc;
	zhpsm::OrientedPoint2D* absPose_global;

	//LOCALIZATION SETTING
	zhpsm::OrientedPoint2D* absPose_localize;

	volatile bool _MAIN_SICK_READY;
	volatile bool _MINOR_SICK_READY;
	volatile bool _ODO_READY;
	volatile bool _GPS_READY;
	volatile bool _BN_READY;
	volatile bool _SYN_GLOBAL_READY;
	volatile bool _SYN_LOCALIZE_READY;
	volatile bool _ODO_FIRST;
	volatile bool _LOCALIZE_FIRST;

	volatile bool _MAIN_SICK_UPDATE;
	volatile bool _MINOR_SICK_UPDATE;
	volatile bool _ODO_UPDATE;
	volatile bool _GPS_UPDATE;
	volatile bool _BN_UPDATE;
	volatile bool _SYN_GLOBAL_UPDATE;

	// for localization
	bool m_bSent2Localization;
	bool _USE_MAIN_SICK;
	bool _USE_MINOR_SICK;
	bool _USE_ODO;
	bool _USE_BN;


	//LOCKERS
	QMutex mutex_synGlobal;
	QMutex mutex_filter;
	QMutex mutex_mainSICK;
	QMutex mutex_minorSICK;
	QMutex mutex_odo;
	QMutex mutex_bn;
};


#endif /* CLIENTFUSION_H_ */
