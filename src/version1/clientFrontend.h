#ifndef CLIENT_FRONTEND_H
#define CLIENT_FRONTEND_H
#include <QObject>
#include <QMutex>
#include <string>
#include <vector>

//timestamp
#include <Eigen/Core>
#include "timestamp.h"

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
public Q_SLOTS:
	void receUpdatePose(int, void*, double*);
	void receUpdatePose2(int, void*, bool); // rece updated pose from Localization
	void runFrontEnd();
	void stop();
	void receScan(float*, uint64_t);
	void recvOdo(float , float , float , float );
	void recvBn(float , float , float );
Q_SIGNALS:

	void send2CBPose(double,double,double);
	void send2CBScan(float*,int,double,double,double);
	void send2CBScanOnly(float*, int);
	void sickCNKError();

	void sendOdo(float , float , float , float );
	void sendBn(float , float , float );

Q_SIGNALS:
	void sendScanFrame(void*, int);
	void sendFirstFrame(void*);
	void sickError();
	void sendScanFrameCov(void*, int, double*);

	void sendFrameInfo(double,double,double,float*,int);
	/*emit relative frame information: x,y,theta,scan_vector, number_points, timestamp, covarance matrix, not move count, bool of good match*/
	void sendRelFrameInfo(double,double,double,float*,int,TTimeStamp,double*,int,bool);
	/*emit relative frame information: x,y,theta,PMScan, not move count, bool of good match*/
	void sendRelFrameInfo2(double,double,double,void*,int,bool);

	void finished();
public:
	bool constructPSMfromRawSeed(char* line, struct _PMScan&, CPolarMatch*);
	bool constructPSMfromCarmon(char* line, struct _PMScan&, CPolarMatch*);
	inline bool constructPSMFromSICK(const CObs2DScan, CPolarMatch* , struct _PMScan& );
	void runRawSeed(); //model 0
	void runCarmon(); //model 1
	void runSICK(string, unsigned int); //model 2
	void runSICK2(); //model X
	void runfile();
	void run(struct _PMScan&);
	bool notMove(zhpsm::OrientedPoint2D&);
	bool validMove(zhpsm::OrientedPoint2D&);
	inline bool smallMove(zhpsm::OrientedPoint2D&);
	void fromRel2AbsPose(zhpsm::OrientedPoint2D&, zhpsm::OrientedPoint2D&, zhpsm::OrientedPoint2D&);
	inline void addMatrix(double*, double*);
	inline void addMatrix(double*, double*, int );
	inline void addEigen(Eigen::Matrix3d&, double*);
	inline void getEigen(Eigen::Matrix3d&, double*);
public:
	bool m_bCSM;
	volatile int work_model;
	volatile bool m_stop_thread;
	string m_file;
	
	CPolarMatch * m_pPSM;
	CCanonicalMatcher* m_pCSM;
	vector<zhpsm::OrientedPoint2D*> m_traj;
	vector<int> traj_index;
	volatile int m_syn_num;
	QMutex m_mutex;
	QMutex m_mutex_csm;

	CClientFusion* m_pFusion;
	vector<string> sick_ip;
	vector<unsigned int> sick_port;
	
	CObs2DScan* m_scan;
	QMutex m_mutex_scan;
	volatile bool _SCAN_READY;

	// for scan-match status
	bool m_bGoodMatch;

	// used in run()
	zhpsm::OrientedPoint2D* sent_pose;
	bool bFirst_run;
	double * matrix;
	int scnt;
	int nmovecnt;
	bool bSetScnt;
	Eigen::Matrix3d curr_cov;
	Eigen::Matrix3d rel_cov;
	Eigen::Matrix3d sent_cov;


	// for UKF 
	bool use_UKF;
	Bayesian_filter::Unscented_scheme *m_pUKF;
	unsigned int x_dim;
	SICK_observe * sick_observe;
	Robot_predict* robot_predict;
	SynGlobal_observe* synGlobal_observe;

	// for corridor matrix
	double corridor_m[6];

	// for localization
	bool m_bSent2Localization;

	//odo
	zhpsm::OrientedPoint2D* absOdo_cur;
	zhpsm::OrientedPoint2D* absOdo_last;
	float odo_t;

	//bn
	zhpsm::OrientedPoint2D* absBn_cur;
};

#endif



