#ifndef CLIENT_FRONTEND_H
#define CLIENT_FRONTEND_H
#include <QObject>
#include <QMutex>
#include <string>
#include <vector>

#include <Eigen/Core>

//timestamp
#include "timestamp.h"

#define USE_COV

namespace Bayesian_filter{class Unscented_scheme;}
class Robot_predict;
class SICK_observe;
class SynGlobal_observe;

namespace zhpsm{class OrientedPoint2D;}
using namespace std;

class CPolarMatch;
class CCanonicalMatcher;
struct _PARAM;
struct _PMScan;

class CClientFusion;
class CSICK;
class CObs2DScan;

class CClientFrontend : public QObject
{
	Q_OBJECT
public:
	CClientFrontend(bool useCSM = 1, QObject* parent=0);
	~CClientFrontend();
	void setModel(int);
	void setFile(string);
	void setSICKIP(vector<string>, vector<unsigned int>);
	void setFusion();
	void getEigen(Eigen::Matrix3d&, double m[6]);
public Q_SLOTS:
	void receUpdatePose(int, void*, double*);
	void runFrontEnd();
	void stop();
	void recvTime(TTimeStamp);
Q_SIGNALS:
	void sendFrameInfo(double,double,double,float*,int,int);
	void sendFrameInfoCov(double,double,double,float*,int,int,double*);

	/*emit relative frame information: x,y,theta,scan_vector, number_points, timestamp, synchronization number, covarance matrix*/
	void sendRelFrameInfo(double,double,double,float*,int,TTimeStamp,int,double*);

	void finished();
public:
	bool constructPSMfromRawSeed(char* line, struct _PMScan&, CPolarMatch*);
	bool constructPSMfromCarmon(char* line, struct _PMScan&, CPolarMatch*);
	bool constructPSMFromSICK(const CObs2DScan, CPolarMatch* , struct _PMScan& );
	void runRawSeed(); //model 0
	void runCarmon(); //model 1
	void runSICK(string, unsigned int); //model 2
	void runfile();
	void run(struct _PMScan&);
	bool notMove(zhpsm::OrientedPoint2D&);
	bool smallMove(zhpsm::OrientedPoint2D&);
	bool validMove(zhpsm::OrientedPoint2D&);
	void fromRel2AbsPose(zhpsm::OrientedPoint2D&, zhpsm::OrientedPoint2D&, zhpsm::OrientedPoint2D&);
	void addMatrix(double *, double*);
	void addEigen(Eigen::Matrix3d&, double*);
public:
	bool m_bCSM;
	volatile int work_model;
	volatile bool m_stop_thread;
	string m_file;

#ifdef USE_COV
	Eigen::Matrix3d curr_cov;
	Eigen::Matrix3d rel_cov;
	Eigen::Matrix3d sent_cov;
	Eigen::Matrix3d R_cov;
	Eigen::Matrix3d H_cov;
	Eigen::Matrix3d K_cov;
	Eigen::Matrix3d Q_cov;
	Eigen::Matrix3d I_cov;
#endif
	CPolarMatch * m_pPSM;
	CCanonicalMatcher* m_pCSM;
	vector<zhpsm::OrientedPoint2D*> m_traj;
	vector<int> traj_index;
	volatile int m_syn_num;
	QMutex m_mutex;

	CClientFusion* m_pFusion;

	vector<string> sick_ip;
	vector<unsigned int> sick_port;
	volatile TTimeStamp recv_t;
	QMutex mutex_recv_t;

	// for run things
	zhpsm::OrientedPoint2D* sent_pose;
	bool bFirst_run;
	double* matrix;
	int scnt;
	bool bSetScnt;

	// for carmon reading
	bool bFirst_carmon;

	// for UKF 
	bool use_UKF;
	Bayesian_filter::Unscented_scheme* m_pUKF;
	unsigned int x_dim;
	SICK_observe* sick_observe;
	Robot_predict* robot_predict;
	SynGlobal_observe* synGlobal_observe;

	// for corridor matrix
	double corridor_m[6];
};

#endif


