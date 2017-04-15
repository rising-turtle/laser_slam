/*
 * threadOdo.cpp
 *
 *  Created on: Dec 25, 2012
 *      Author: liu
 */


#include "threadOdo.h"
#include "stdlib.h"
#include <geometry/point.h>

COdoNode::COdoNode():
m_timestamp(0),
m_pose(new OrientedPoint2D),
m_relpose(new OrientedPoint2D)
{}

COdoNode::~COdoNode()
{
	if(m_pose) delete m_pose;
	if(m_relpose) delete m_relpose;
}

ThreadOdo::ThreadOdo():
														m_stop_thread_odo(false),
														m_file_name_odo("")
{ }

ThreadOdo::~ThreadOdo(){
	//
}
// stop threadOdo
void ThreadOdo::stopThreadOdo(){
	m_stop_thread_odo = true;
}

// pause threadOdo log reader
void ThreadOdo::pauseThreadOdoReader(){
	m_pause_thread_odo = true;
}

//resume threadOdo log reader
void ThreadOdo::resumeThreadOdoReader(){
	m_pause_thread_odo = false;
}


void ThreadOdo::getTimeRecvOdo(TTimeStamp t)
{
	cout<<"getTimeRecvOdo: "<<(int)QThread::currentThreadId()<<" "<<t<<endl;
	m_time_recv_odo = t;
}

void ThreadOdo::setLogPathOdo(const char* path){
	m_file_name_odo = string(path);
}

void ThreadOdo::runRawseedOdo(){
	cout<<"ThreadOdo::runRawseedOdo ID: "<<(int)QThread::currentThreadId()<<endl;

	ifstream inf(m_file_name_odo.c_str());
	if(!inf.is_open()){
		cout<<"ThreadOdo::runRawseedOdo - failed to open file: "<<m_file_name_odo<<endl;
		return ;
	}

	char line[8192];
	double timestamp_d; //must be double
	int roll_counter;
	int tickets_right;
	int tickets_left;
	double x;
	double y;
	double theta; // robot pose

	string delim(",");
	COdoNode m_odo;
	COdoNode m_odo_last;

	bool start = true;

	//ofstream ofs;
	//ofs.open("tmp.txt");
	while(!m_stop_thread_odo && !inf.eof() ){

		inf.getline(line,8192);
		cout<<"Odo: "<<line<<endl;
		char* pch = NULL;
		char* saveptr;

		pch = strtok_r(line, delim.c_str(), &saveptr);
		int idx = 0;

		if(!pch)
		{
			cout<<"ThreadOdo::runRawseedOdo - This is possibly the end"<<endl;
			return ;
		}

		while(pch)
		{
			//cout<<pch<<endl;
			switch(++idx)
			{

			case 1:
				m_odo.m_timestamp = (int64_t)(atof(pch)*10000000.0);
				break;
			case 2:
				roll_counter = atoi(pch);
				break;
			case 3:
				tickets_right = atoi(pch);
				break;
			case 4:
				tickets_left = atoi(pch);
				break;
			case 5:
				m_odo.m_pose->x = atof(pch);
				break;
			case 6:
				m_odo.m_pose->y = atof(pch);
				break;
			case 7:
				m_odo.m_pose->theta = atof(pch);
				break;

			}

			pch = strtok_r(NULL, delim.c_str(), &saveptr);
		}

		if(start)
		{
			*(m_odo.m_relpose)= *(m_odo.m_pose);
			start = false;
		}
		else
		{
			//send the last node, not the current;
			sendOdoNode(&m_odo_last);
			cout<<"ThreadOdo::runRawseedOdo - sendOdoNode pose: " <<m_odo_last.m_timestamp <<" "<<m_odo_last.m_pose->x*100<<" "<<m_odo_last.m_pose->y*100<<" "<<m_odo_last.m_pose->theta*180/M_PI<<endl;
			cout<<"ThreadOdo::runRawseedOdo - sendOdoNode relpose: "<<m_odo_last.m_timestamp <<" " <<m_odo_last.m_relpose->x*100<<" "<<m_odo_last.m_relpose->y*100<<" "<<m_odo_last.m_relpose->theta*180/M_PI<<endl;
			*(m_odo.m_relpose) = m_odo_last.m_pose->ominus(*(m_odo.m_pose));

			while(m_time_recv_odo != m_odo_last.m_timestamp)
			{
				//cout<<(int)QThread::currentThreadId()<<" LOOOOP " <<lastNode->m_timestamp<<" "<<m_time_recv_sick<<endl;
				QThread::yieldCurrentThread();
				continue;
			}
		}

		m_odo_last.m_timestamp = m_odo.m_timestamp;
		*(m_odo_last.m_pose) = *(m_odo.m_pose);
		*(m_odo_last.m_relpose) = *(m_odo.m_relpose);

	}

	finished();
	return ;
}

