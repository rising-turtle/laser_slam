#ifndef SLAM_V1_H
#define SLAM_V1_H

#include <QObject>
#include <QWidget>
#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <QCoreApplication>
#include <string>
#include <vector>
#include <stdint.h>

using namespace std;

#define SYS_ERR_CTRL_BATTERY_LOW 1
#define SYS_ERR_POWER_BATTERY_LOW 2
#define SYS_LOST_CNC_SICK_A 3
#define SYS_LOST_CNC_SICK_B 4
#define SYS_LOST_BN_SERIAL 5
#define SYS_LOST_LOW_CTRL_SERIAL 6



typedef int (*CallBack_Odometry)(float *pfOdometry);
typedef int (*CallBack_BNLocation)(float *pfLocation);

typedef int (*CallBack_MainSICKForSLAM)(float*, uint64_t&);
typedef int (*CallBack_MainSICKForOD)(float*, uint64_t&);
typedef int (*CallBack_MinorSICKForSLAM)(float*, uint64_t&);
typedef int (*CallBack_MinorSICKForOD)(float*, uint64_t&);

typedef int (*CallBack_SICK)(vector<float>&);
typedef int (*CallBack_DataFusionResult)(float fX,float fY,float fTheta);
typedef int (*CallBack_LocalMap)(float *pfLocalMapData);
typedef int (*CallBack_GlobalMap)(float *pfGlobalMapData);
typedef int (*CallBack_ErrList)(int nErrCode);
typedef int (*CallBack_OnlySLAMResult)(float fX,float fY,float fTheta);
typedef int (*CallBack_OnlyOdoResult)(float fX,float fY,float fTheta);
typedef int (*CallBack_OnlyBNResult)(float fX,float fY,float fTheta);
typedef int (*CallBack_DataFusionAndPC)(float* b, int n, double px, double py, double pth);

typedef int (*CallBack_Localization)(double px, double py, double pth);
typedef struct SLAM_CallBack
{
	CallBack_Odometry cbOdometry;
	CallBack_BNLocation cbBNLocation;
	CallBack_MainSICKForSLAM cbMainSICKForSLAM;
	CallBack_MainSICKForOD cbMainSICKForOD;
	CallBack_MinorSICKForSLAM cbMinorSICKForSLAM;
	CallBack_MinorSICKForOD cbMinorSICKForOD;
	CallBack_SICK cbSICKA;
	CallBack_SICK cbSICKB;
	CallBack_DataFusionResult cbDataFusionResult;
	CallBack_LocalMap cbLocalMap;
	CallBack_GlobalMap cbGlobalMap;
	CallBack_ErrList cbErrList;
	CallBack_OnlySLAMResult cbOnlySLAMResult;
	CallBack_OnlyOdoResult cbOnlyOdoResult;
	CallBack_OnlyBNResult cbOnlyBNResult;
	CallBack_DataFusionAndPC cbDataFusionAndPC;
	CallBack_Localization cbLocalization;
} SLAM_CallBack;

typedef struct SLAMParams
{
	char cHost_IP_A[16];
	int nHost_Port_A;
	char cSICK_IP_A[16];
	int nSICK_Port_A;

	char cHost_IP_B[16];
	int nHost_Port_B;
	char cSICK_IP_B[16];
	int nSICK_Port_B;
}SLAMParams;

class CClientFrontend; // frontend process
class CClientBackend; // backend process
class LocalV1;


class CSlamV1 : public QThread
{
	Q_OBJECT
public:
	CSlamV1();
	~CSlamV1();
	void init(SLAM_CallBack* pCBSet = NULL, SLAMParams* pParams = NULL );
	void run();
	void setSystem();
	void default_init();
	void initFromParams(SLAMParams*);

public Q_SLOTS:
	void receCurrentPose(double,double,double);
	void receCurrentScan(float*,int,double,double,double);
	void receCurrentScanOnly(float*,int);
	void stop();
	void receMainSickCNKError();
	void receMinorSickCNKError();
	void receMainSickSLAM(double,double,double,float*,int);
	void receODO(float, float, float, float);
	void receBN(float, float, float);
	void receLD(float, float, float);
Q_SIGNALS:
	void stopAllThread();
	void stopFrontend();
	void stopBackend();
	void sendBN(float,float,float);
	void sendODO(float,float,float,float);
	void sendScanMain(float*, uint64_t);
	void sendScanMinor(float*, uint64_t);
	void stopLocalization();
public:
	string m_file_name;
	int m_work_model;
	
	string m_sick1_ip;
	string m_sick2_ip;
	unsigned int m_sick1_port;
	unsigned int m_sick2_port;
public:
	// QThread m_threadSocket;
	QThread m_threadFrontend;
	QThread m_threadBackend;
	CClientFrontend* m_pThreadFrontend;
	CClientBackend* m_pThreadBackend;
	QThread m_threadLocalization;
	LocalV1* m_pLocalization;
public:
	SLAM_CallBack* m_pCBSet;
	QMutex m_mutex;
	QWaitCondition m_wait_cond;
	float m_px;
	float m_py; 
	float m_pth;
	vector<float> sx;
	vector<float> sy;
	vector<float> m_b;
	volatile bool m_stopped;
	volatile bool m_received;
	volatile bool m_received_fusion;
	volatile bool m_received_scan;

	float* bnPose;
	float* odoPose;
	float* scanMain;
	float* scanMinor;

	uint64_t t_scanMain_cur;
	uint64_t t_scanMinor_cur;

	uint64_t t_scanMain_last;
	uint64_t t_scanMinor_last;

	bool odo_first;
	bool bn_first;
	float odoTime_last;
	float odoTime_cur;

public:
	void startEventLoop(int& argc, char** argv);
	QCoreApplication* qt_app;
	
private:
	CSlamV1(const CSlamV1&);
	CSlamV1 operator=(const CSlamV1&);
};


#endif
