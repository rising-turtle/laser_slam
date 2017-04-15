#ifndef LOCALV1_H
#define LOCALV1_H
#include "localization.h"
#include <string>
#include <vector>
#include <QMutex>

using namespace std;

class LocalV1 : public threadLocalization
{
	Q_OBJECT
public:
	LocalV1();
	virtual ~LocalV1();
	void init(const char* imgfile, int start_x, int start_y);
public Q_SLOTS:
	void receFirstFrame(void*);
	void recePoseScan(void*, int);
	void runLocalization();
	void stop();
Q_SIGNALS:
	void sendUpdatePose(int, void*, bool);
	void finished();
public:
	// back up
	string m_imgfile;
	int m_ori_x;
	int m_ori_y;
	vector<OrientedPoint2D*> m_pPose;
	vector<struct _PMScan*> m_pScan;
	struct _PMScan* m_pFirstFrame;
	OrientedPoint2D* m_synPose;
	vector<int> m_syn_num;
	volatile bool m_bSynCoordinate;
	volatile bool m_bStopLocalization;
	volatile bool m_bFinishInit;
	QMutex m_mutex;
};

#endif
