#include "threadSICK.h"
#include "FlirterNode.h"
#include "ZHPolar_Match.h"
#include "MapNode.h"

static QMutex m_mutex_frontEnd; // jsut for thread safe of csm;

ThreadSICK::ThreadSICK(string name):// m_pPSM(new CPolarMatch("LMS151")),
									m_pPSM(0), // all new resource must be initialized in main_loop
									m_stop_thread(false),
									m_pause_thread_sick(false),
									m_file_name(""),
									oriInRobot(new OrientedPoint2D(0.0, 0.0, 0.0)),
									m_time_recv_sick(0),
									m_SName(name),
									sick_ip(""),
									sick_port(0)
{}
ThreadSICK::~ThreadSICK(){
	if(m_pPSM!=0) delete m_pPSM;
	if(oriInRobot!=0) delete oriInRobot;
}
void ThreadSICK::stopThreadSICK(){
	// cout<<"thread: "<<(int)QThread::currentThreadId()<<" stop threadLocal1"<<endl;
	m_stop_thread = true;
}

// pause ThreadSICK log reader
void ThreadSICK::pauseThreadSICKReader(){
	cout<<"pause: "<<(int)QThread::currentThreadId()<<endl;
	m_pause_thread_sick = true;
}

//resume ThreadSICK log reader
void ThreadSICK::resumeThreadSICKReader(){
	cout<<"resume: "<<(int)QThread::currentThreadId()<<endl;
	m_pause_thread_sick = false;
}

void ThreadSICK::getTimeRecvSICK(TTimeStamp t){
	/*CFliterNode* lastNode = m_graph[m_graph.size()-2];
	cout<<"getTimeRecvSICK: "<<(int)QThread::currentThreadId()<<" "<<t<<" actually need: "<<lastNode->m_timestamp<<endl;*/
	cout<<"SICK "<<m_SName<<" recv Time: "<<t<<endl;
	QMutexLocker locker(&m_mutex_time_recv);
	m_time_recv_sick= t;
}


void ThreadSICK::setLogPath(const char* path){
	m_file_name = string(path);
}
void ThreadSICK::runLog()
{
	cout<<"Fusion: runLog() ThreadSICK: "<<(int)QThread::currentThreadId()<<endl;
	m_pPSM = new CPolarMatch("LMS151");
	cout<<m_file_name<<endl;
	if(!m_pPSM->readCarmon(m_file_name,m_pPSM->m_pParam->pm_laser_name))
	{
		cerr<<"failed to read file: "<<m_file_name<<endl;
		finished();
		return ;
	}
	// run pfg Local, here we do not use graph optimizition 
	int cnt=0;
	PMScan* ls;
	vector<float> obs_x;
	vector<float> obs_y;
	while(cnt<m_pPSM->m_SickScans.size() && !m_stop_thread)
	{
		ls = m_pPSM->m_SickScans[cnt];
		cnt++;
		CFliterNode* pcurNode = new CFliterNode(ls,m_pPSM);
		pcurNode->m_timestamp = getCurrentTime(); // adding a timestamp
		pcurNode->m_id = m_graph.size();
		runFrontEnd(pcurNode);
	}
	// send the last node
	sendPoseNode((void*)(m_graph[m_graph.size()-1]));
	// cout<<"quit thread1 loop!"<<endl;
	// stop threadLocal1
	// cout<<"quit threadLocal1!"<<endl;
	finished();
	return ;
}


void ThreadSICK::runRawseed()
{
	//cout<<"Fusion: runRawseed() ThreadSICK: "<<(int)QThread::currentThreadId()<<endl;
	//string filename = "/media/ShareRegion/Datasets/Rawseeds/Bicocca_2009-02-25b/Bicocca_2009-02-25b-SICK_FRONT.csv";
	//string laserName = "SICK_FRONT";
	//string laserName = "SICK_REAR";
	string laserType = "LMS211"; // LMS2xx in fact

	m_pPSM = new CPolarMatch(laserType);
	cout<<"ThreadSICK::runRawseed: "<<m_SName<<" "<<(int)QThread::currentThreadId()<<" has "<<m_pPSM<<endl;

	/*
	m_pPSM->m_pParam->pm_dfi = PM_D2R;
	m_pPSM->m_pParam->pm_fov = 180;
	m_pPSM->m_pParam->pm_fi_min = -M_PI_2;
	m_pPSM->m_pParam->pm_fi_max = M_PI_2;
	m_pPSM->m_pParam->pm_max_range = 8000;
	m_pPSM->m_pParam->pm_l_points = 181;
	m_pPSM->pm_init();
	 */

	PMScan ls(m_pPSM->m_pParam->pm_l_points);
	ls.rx = 0;
	ls.ry = 0;
	ls.t = 0;
	ls.th = 0;

	ifstream inf(m_file_name.c_str(), ifstream::in);
	if(!inf.is_open()){
		cout<<"in runRawseed() failed to open file: "<<m_file_name<<endl;
		return ;
	}

	char line[8192];
	double timestamp; //must be double
	int num_points;
	string delim(",");
	float r;
	int k=0;
	while(!m_stop_thread && !inf.eof() ){

		for(int i =0; i<20; i++)
		inf.getline(line,8192);
		char* pch;
		char* saveptr;
		bool badScan = false;

		pch = strtok_r(line, delim.c_str(), &saveptr);
		int idx = 0;
		int offset = 9999;
		if(!pch)
		{
			cout<<"This is possibly the end"<<endl;
			return ;
		}

		while(pch && offset == 9999 && idx<4)
		{
			cout<<pch<<endl;
			switch(++idx)
			{
			case 1:
				timestamp = strtod(pch, NULL);
				break;
			case 2:
				num_points = strtol(pch, NULL, 0);
				if(num_points !=m_pPSM->m_pParam->pm_l_points)
				{
					cout<<"ThreadSICK::runRawseed: NOT ENOUGH POINTS"<<endl;
					badScan = true;
					//return ;
					//continue;
				}
				break;
			case 3:
				offset = strtol(pch, NULL, 0);
				break;
			}
			pch = strtok_r(NULL, delim.c_str(), &saveptr);
		}

		if(badScan)
			continue;

		int i;
		for(i = 0 ; i < num_points && pch; i++, pch = strtok_r(NULL, delim.c_str(), &saveptr))
		{

			ls.r[i] = strtof(pch, NULL)*100.0; // from [m] 2 [cm]
			ls.x[i] = ls.r[i] * m_pPSM->pm_co[i];
			ls.y[i] = ls.r[i] * m_pPSM->pm_si[i];
			ls.bad[i] = 0;
			if(ls.r[i]<PM_MIN_RANGE){
				ls.r[i]=m_pPSM->m_pParam->pm_max_range+1; //set it to a value larger than the max
			}
		}

		if(i!=num_points){
			cout<<"Not enough points in file!"<<endl;
			return ;
		}

		CFliterNode* pcurNode = new CFliterNode(new PMScan(ls),m_pPSM);
		pcurNode->m_timestamp = (int64_t)(timestamp*10000000); // adding a timestamp (int64_t)
		pcurNode->m_id = m_graph.size();

		if(!runFrontEnd(pcurNode))
		{
			cout<<"+++++FrontEnd failed!!!!!"<<endl;
		}

		if(m_graph.size()>1)
		{
			// send pose node
			CFliterNode* lastNode = m_graph[m_graph.size()-2];
			TTimeStamp lastNode_t = lastNode->m_timestamp; // YOU would not want to access this class object all the time;
			cout<<"SICK "<<m_SName<<" sendPose time: "<<lastNode_t<<endl;
			sendPoseNode((void*)(m_graph[m_graph.size()-2]));

			// check feedback
			int bug_finder = 0;
			while(1)
			{
				QMutexLocker locker(&m_mutex_time_recv);
				if(m_time_recv_sick != lastNode_t)
				{
					//cout<<(int)QThread::currentThreadId()<<" LOOOOP " <<lastNode->m_timestamp<<" "<<m_time_recv_sick<<endl;
					QThread::yieldCurrentThread();
					bug_finder++;
					//continue;
				}
				else break;
				if(bug_finder>200000){
					bug_finder = 0;
					cout<<m_SName<<" lastNode->time: "<<lastNode_t<<" received: "<<m_time_recv_sick<<endl;
				}
			}
		}
	}

	cout<<m_SName<<" quit!!!!!!!"<<endl;
	cout<<"IMPOSSIBLE!!"<<endl;

	// send the last node
	sendPoseNode((void*)(m_graph[m_graph.size()-1]));
	finished();
	return ;
}

bool ThreadSICK::IsValidMotion(OrientedPoint2D& pose){
#define MOTION_THRE 0.0001 
#define ANGLE_THRE 0.0001  
	if(fabs(pose.x)+ fabs(pose.y) >= MOTION_THRE)
		return true;
	if(fabs(pose.theta) >= ANGLE_THRE)
		return true;
	return false;
}

bool ThreadSICK::runFrontEnd(CFliterNode* pcurNode)
{
	vector<float> obs_x;
	vector<float> obs_y;
	{
		TTimeStamp t_s = getCurrentTime();
		QMutexLocker locker(&m_mutex_update);
		if(m_graph.size()<=0)
		{
			//initial SICK pose referenced to the robot frame
			pcurNode->m_pose.x = oriInRobot->x;
			pcurNode->m_pose.y = oriInRobot->y;
			pcurNode->m_pose.theta = oriInRobot->theta;
			m_graph.insert(make_pair(0,pcurNode));
			return true;

		}else{

			// CFliterNode* prefNode = m_graph[m_graph.size()-1];
			CFliterNode* prefNode = m_graph.rbegin()->second;
			try{
				// 0 Failed 1 PSM succeed 2 ICP succeed
				// int status = pcurNode->matchNodeFrontend(prefNode);
				// 0 Failed 1 CSM succeed
				// static int cnt = 0;
				// cout<<" the "<<++cnt<<" match!"<<endl;
				{
					QMutexLocker lock(&m_mutex_frontEnd);
					int status = pcurNode->matchNodeFrontend2(prefNode);
				}

				/*if(!status)
							{
								cout<<"@@@@@@@@@@@PSM failed!"<<endl;
								delete pcurNode;
								return false;
							}*/

				/*if(status == 1 && !IsValidMotion(pcurNode->m_relpose))
				{
					cout<<"~~~~~~~~Not move!"<<endl;
				    delete pcurNode;
					return false;
				}
				 */

				// translate to robot frame;

				pcurNode->m_pose = prefNode->m_pose.oplus(pcurNode->m_relpose);
				m_graph.insert(make_pair(m_graph.size(),pcurNode));
			}catch(...){
				std::cout<<"catch wrong in RunPFGLocal runlog()!"<<std::endl;
				return false;
			}
		}
		TTimeStamp t_e = getCurrentTime();
		//cout<<"Time: "<<timeDifference(t_s, t_e) * 1000<<endl;
	}
	return true;
}

PMScan* ThreadSICK::constructPSM(vector<float>& bearing, CPolarMatch* psm){
	if(psm->m_pParam->pm_l_points != bearing.size())
	{
		cout<<"Number of Bearings not right!"<<endl;
		return NULL;
	}
	PMScan* ret = new PMScan(psm->m_pParam->pm_l_points);
	ret->rx = 0.;
	ret->ry = 0.;
	ret->th = 0.;
	ret->t = 0.;
	for(int i=0;i<psm->m_pParam->pm_l_points;i++){

		ret->r[i] = bearing[i]*100.0; // PSM using cm
		ret->x[i] = ret->r[i] * psm->pm_co[i];
		ret->y[i] = ret->r[i] * psm->pm_si[i];
		ret->bad[i] = 0;
		if(ret->r[i] < PM_MIN_RANGE || ret->r[i] < 0){
			ret->r[i] = psm->m_pParam->pm_max_range+ 1;
		}
	}
	return ret;
}

void ThreadSICK::runSick(){
	cout<<"in runSick()!"<<endl;
	// single SICK situation

	string laserType = "LMS151";
	m_pPSM = new CPolarMatch(laserType);

	cout<<"sick addr, ip: "<<sick_ip<<" port: "<<sick_port<<endl;
	CSICK laser(sick_ip,sick_port);
	laser.setSensorLabel(m_SName);
	CObs2DScan outObs;
	bool isOutObs, hardwareError;
	if(!laser.turnOn()){
		cout<<"failed to turnOn SICK!"<<endl;
		return ;
	}
	cout<<"succeed to turnOn SICK!"<<endl;
	laser.doProcessSimple(isOutObs, outObs, hardwareError);
	PMScan* ls;
	while(1){

		laser.doProcessSimple(isOutObs, outObs, hardwareError); // 20ms
		if(hardwareError) continue;
		// TODO:
		ls = constructPSM(outObs.scan, m_pPSM);
		if(ls == NULL){
			cout<<"this frame maybe not right! "<<endl;
			continue;
		}
		CFliterNode* pcurNode = new CFliterNode(ls, m_pPSM, outObs.timestamp); // this is too slow, e.g., 300ms
		pcurNode->m_id = m_graph.size();

		if(!runFrontEnd(pcurNode)){
			// cout<<"failed to add this frame!"<<endl;
		}else{
			// cout<<"++++++ succed to add node: +++++++"<<pcurNode->m_id<<endl;

			if(m_graph.size()>1)
			{
				// send pose node
				CFliterNode* lastNode = m_graph[m_graph.size()-2];
				TTimeStamp lastNode_t = lastNode->m_timestamp; // YOU would not want to access this class object all the time;
				cout<<"SICK "<<m_SName<<" sendPose time: "<<lastNode_t<<endl;
				sendPoseNode((void*)(m_graph[m_graph.size()-2]));
			}
		}
	}

	cout<<"!!!!!! SLAM LOCAL EXIT!"<<endl;
	return ;
}
void ThreadSICK::synFromGlobal(int id, void* pose)
{
	cout<<"synFromGlobal! graph.size(): "<<m_graph.size()<<", id: "<<id<<endl;
	OrientedPoint2D* new_pose = static_cast<OrientedPoint2D*> (pose);
	if(new_pose == NULL){
		cout<<"error in synFromGlobal!"<<endl;
		return ;
	}
	{
		QMutexLocker locker(&m_mutex_update);
		map<int, CFliterNode*>::iterator it = m_graph.find(id);
		if(it == m_graph.end()){
			cerr<<"error in synFromGlobal!"<<endl;
			return ;
		}
		// only update the last node is enough
		// it->second->m_pose = *new_pose;
		CFliterNode* last = m_graph.rbegin()->second;
		last->m_pose = last->m_pose.oplus(it->second->m_pose.ominus(*new_pose));
		// update all the newly node
		/*while(++it_next!=m_graph.end()){
			it_next->second->m_pose = m_last_pose.oplus(it_next->second->m_relpose);
			m_last_pose = it_next->second->m_pose;
		}*/

		// delete these not valid pose
		/*if(it!=m_graph.begin()){
			m_graph.erase(m_graph.begin(),it);
		}*/
		// cout<<"~~~~~~clean observation!!!"<<endl;
		// to clean observations before
		cleanObservation((double)last->m_pose.x,(double)last->m_pose.y);
	}
}

void ThreadSICK::translate2RobotFrame(OrientedPoint2D* newPose)
{
	float fcos = cosf(oriInRobot->theta);
	float fsin = sinf(oriInRobot->theta);

	double tx, ty, ttheta;
	tx = fcos*newPose->x - fsin*newPose->y + oriInRobot->x;
	ty = fsin*newPose->x + fcos*newPose->y + oriInRobot->y;
	ttheta = newPose->theta + oriInRobot->theta;

	newPose->x = tx;
	newPose->y = ty;
	newPose->theta = ttheta;

}
void ThreadSICK::translate2GlobalFrame(float* bearing, vector<float>& fx,vector<float>& fy, double rx, double ry, double th)
{
	const float bad_range = -100000;
	int BEAR_NUM = m_pPSM->m_pParam->pm_l_points;
	// cout<<"BEAR_NUM: "<<BEAR_NUM<<endl;
	if(fx.size()!=BEAR_NUM){
		fx.resize(BEAR_NUM);
		fy.resize(BEAR_NUM);
	}

	float x,y,tx,ty,nIdx,nIdy;
	float fcos = cosf(th);
	float fsin = sinf(th);
	for(int i=0;i<BEAR_NUM;i++){
		if(bearing[i]>m_pPSM->m_pParam->pm_max_range){ // this is bad range
			fx[i]=fy[i]=bad_range;
			continue;
		}
		x=(bearing[i]/100.0)*m_pPSM->pm_co[i];
		y=(bearing[i]/100.0)*m_pPSM->pm_si[i];

		tx=fcos*x-fsin*y+rx;
		ty=fsin*x+fcos*y+ry;

		/*tx = fsin*x+y*fcos+rx;
		ty = -fcos*x+y*fsin+ry;*/

		fx[i] = tx;
		fy[i] = ty;
	}
	// cout<<"finish translate!!"<<endl;
}
