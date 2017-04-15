/*
 * threadOdo.h
 *
 *  Created on: Dec 25, 2012
 *      Author: liu
 */

#ifndef THREADODO_H_
#define THREADODO_H_

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

#include "qobjectdefs.h"
#include "timestamp.h"

class OrientedPoint2D;

using namespace std;

class COdoNode{
public:
	COdoNode();
	~COdoNode();

	TTimeStamp m_timestamp;
	OrientedPoint2D* m_pose;
	OrientedPoint2D* m_relpose;
};

class ThreadOdo : public QObject
{
	Q_OBJECT
public:
	ThreadOdo();
	~ThreadOdo();
	void setLogPathOdo(const char*);

Q_SIGNALS:
	void sendOdoNode(void* param);
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
	volatile bool m_stop_thread_odo;
	volatile bool m_pause_thread_odo;
	volatile TTimeStamp m_time_recv_odo;
};


#endif /* THREADODO_H_ */
