/*
 * clientOdo.h
 *
 *  Created on: Jan 28, 2013
 *      Author: liu
 */

#ifndef CFRONTENDODO_H_
#define CFRONTENDODO_H_


#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <unistd.h>

#include <QMutexLocker>
#include <QThread>
#include <QMutex>
#include <QObject>

#include "timestamp.h"
#include "point.h"

namespace zhpsm{class OrientedPoint2D;}
using namespace std;

class COdoNode{
public:
	COdoNode();
	~COdoNode();

	TTimeStamp m_timestamp;
	zhpsm::OrientedPoint2D* m_pose;
	zhpsm::OrientedPoint2D* m_relpose;
};

class CFrontendOdo : public QObject
{
	Q_OBJECT
public:
	CFrontendOdo();
	~CFrontendOdo();
	void setFile(string);

Q_SIGNALS:
	void sendOdoNode(double , double , double , TTimeStamp );
	void sendRelOdoNode(double , double , double , TTimeStamp );
	void finished();

public Q_SLOTS:
	void runRawseedOdo(); // run from file
	void stopThreadOdo();
	void pauseThreadOdoReader();
	void resumeThreadOdoReader();
	void getTimeRecvOdo(TTimeStamp );
public:

	string m_file_name_odo;
	QMutex m_mutex_update_odo;
	QMutex m_mutex_prepare_odo;
	QMutex m_mutex_time_odo;
	volatile bool m_stop_thread_odo;
	volatile bool m_pause_thread_odo;
	volatile TTimeStamp m_time_recv_odo;
};


#endif /* CFRONTENDODO_H_ */
