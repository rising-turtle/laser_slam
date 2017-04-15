/*
 * clientOdo.cpp
 *
 *  Created on: Jan 28, 2013
 *      Author: liu
 */


#include "frontendOdo.h"
#include "stdlib.h"

using namespace zhpsm;
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

CFrontendOdo::CFrontendOdo():
m_stop_thread_odo(false)
{ }

CFrontendOdo::~CFrontendOdo(){
	//
}
// stop threadOdo
void CFrontendOdo::stopThreadOdo(){
	m_stop_thread_odo = true;
}

// pause threadOdo log reader
void CFrontendOdo::pauseThreadOdoReader(){
	m_pause_thread_odo = true;
}

//resume threadOdo log reader
void CFrontendOdo::resumeThreadOdoReader(){
	m_pause_thread_odo = false;
}


void CFrontendOdo::getTimeRecvOdo(TTimeStamp t)
{
	cout<<"getTimeRecvOdo: "<<(int)QThread::currentThreadId()<<" "<<t<<endl;
	QMutexLocker lock(&m_mutex_time_odo);
	m_time_recv_odo = t;
}

void CFrontendOdo::setFile(string path){
	m_file_name_odo = path;
}

void CFrontendOdo::runRawseedOdo(){
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
			//sendOdoNode(m_odo_last.m_pose->x, m_odo_last.m_pose->y,m_odo_last.m_pose->theta,m_odo_last.m_timestamp);
			//cout<<"ThreadOdo::runRawseedOdo - sendOdoNode pose: " <<m_odo_last.m_timestamp <<" "<<m_odo_last.m_pose->x*100<<" "<<m_odo_last.m_pose->y*100<<" "<<m_odo_last.m_pose->theta*180/M_PI<<endl;
			*(m_odo.m_relpose) = m_odo_last.m_pose->ominus(*(m_odo.m_pose));
			OrientedPoint2D pp = m_odo_last.m_pose->oplus(*(m_odo.m_relpose));
			cout<<"### "<<m_odo_last.m_pose->x<<" "<<m_odo_last.m_pose->y<<" "<<m_odo_last.m_pose->theta<<endl;
			cout<<"*** "<<m_odo.m_pose->x<<" "<<m_odo.m_pose->y<<" "<<m_odo.m_pose->theta<<endl;
			cout<<"--- "<<m_odo.m_relpose->x<<" "<<m_odo.m_relpose->y<<" "<<m_odo.m_relpose->theta<<endl;
			cout<<"+++ "<<pp.x<<" "<<pp.y<<" "<<pp.theta<<endl;

			cout<<"ThreadOdo::runRawseedOdo - sendOdoNode relpose: "<<m_odo.m_timestamp <<" " <<m_odo.m_relpose->x*100<<" "<<m_odo.m_relpose->y*100<<" "<<m_odo.m_relpose->theta*180/M_PI<<endl;
			sendRelOdoNode(m_odo.m_relpose->x, m_odo.m_relpose->y,m_odo.m_relpose->theta,m_odo.m_timestamp);

			TTimeStamp t = m_odo.m_timestamp;
			while(1)
			{
				QMutexLocker lock(&m_mutex_time_odo);
				//cout<<(int)QThread::currentThreadId()<<" LOOOOP " <<lastNode->m_timestamp<<" "<<m_time_recv_sick<<endl;
				if(m_time_recv_odo != t)
				{
					QThread::yieldCurrentThread();
					continue;
				}
				else
					break;
			}
		}

		m_odo_last.m_timestamp = m_odo.m_timestamp;
		*(m_odo_last.m_pose) = *(m_odo.m_pose);
		*(m_odo_last.m_relpose) = *(m_odo.m_relpose);

	}

	finished();
	return ;
}

