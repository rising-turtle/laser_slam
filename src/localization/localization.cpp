#include "localization.h"
#include "globaldef.h"
#include "VPmap.h"
#include "PolarParameter.h"
#include "ZHPolar_Match.h"
#include "ZHCanonical_Matcher.h"
#include "particles.h"
#include <algorithm>
#include <iostream>
#include <stdlib.h>
#include <sys/time.h>


using namespace std;

namespace{
	string iniFile("/mnt/hgfs/SharedFold/dataset/lenovo/LMS151/work_circles_2.txt");
	static double dummy_x = 0;
	static double dummy_y = 0;
	static double dummy_th = 0;

	bool isSmallMove(OrientedPoint2D& p)
	{
	const float SMALL_MOTION = 0.2*0.2;// 0.2*0.2;
	const float SMALL_ANGLE = M_PI/6.;
		if(p.x*p.x + p.y*p.y >= SMALL_MOTION)
			return false;
		if(fabs(p.theta) >= SMALL_ANGLE)
			return false;
		return true;
	}
}

threadLocalization::threadLocalization():
m_PMap(new CVPmap),
m_Particles(new CParticles(g_num_of_particles,3)),
m_pPSM(new CPolarMatch("LMS151")),
m_pCSM(new CCanonicalMatcher(m_pPSM->m_pParam)),
m_scanFile(new ifstream(iniFile.c_str())),
m_bDrawLocalization(false),
m_bPFLocalization(false),
m_bScanMatchSim(false),
m_cur_scan(0),
m_sim_scan(0),
m_x_range(0),
m_y_range(0)
{}
threadLocalization::~threadLocalization(){
	delete m_PMap;
	delete m_Particles;
	delete m_pPSM;
	delete m_pCSM;
	delete m_scanFile;
}

bool threadLocalization::receivePMAP(QImage* pm)
{
	if( m_PMap->constructFromImage(pm))
	{
		cout<<"localization.cpp: succeed constructPMAP!"<<endl;
		m_x_range = ((m_PMap->m_size_x-1)*(m_PMap->m_resolution));
		m_y_range = ((m_PMap->m_size_y-1)*(m_PMap->m_resolution));
		updatePMAP(m_PMap);
		return true;
	}
	cout<<"localization.cpp: failed constructPMAP!"<<endl;
	return false;
}

void threadLocalization::ranParticles()
{
	OrientedPoint2D pose;
	vector<int> px(g_num_of_particles); 
	vector<int> py(g_num_of_particles);
	for(int i=0;i<g_num_of_particles;)
	{
		randomPose(pose);
		if(m_PMap->isValidPose(pose))
		{
			px[i] = m_PMap->x2idx(pose.x);
			py[i] = m_PMap->y2idx(pose.y);
			i++;
		}
	}
	sendParticles(&px[0],&py[0],px.size());
}

void threadLocalization::randomPose(OrientedPoint2D& pose)
{
	static bool bfirst = true;
	if(bfirst)
	{
		srand(time(NULL));
		bfirst = false;
	}
	pose.x = (rand()%1000)*0.001*m_x_range;
	pose.y = (rand()%1000)*0.001*m_y_range;
	if(pose.x >= m_x_range || pose.y >= m_y_range)
	{
		cout<<"localization: error range: "<<pose.x<<", "<<pose.y<<endl;
	}
	pose.theta = (rand()%360)*PM_D2R;
}

void threadLocalization::randomScan()
{
	OrientedPoint2D randPose;
	while(1)
	{
		randomPose(randPose);
		if(m_PMap->isValidPose(randPose)) break;
	}
	PMScan scan(541);
	
	m_PMap->laserScanSimulator(&scan,randPose);

	// send to display
	vector<int> px;
	vector<int> py;
	int pose[2];
	float theta = randPose.theta;
	m_PMap->gen2Display(px,py,pose,&scan);
	send2Scan(&px[0],&py[0],px.size(),pose,theta);
}

void threadLocalization::topK(vector<int>& index, vector<float>& we, int k)
{
	if(k >= we.size() ) return ;
	vector<float> weight;
	weight.insert(weight.begin(),we.begin(),we.end());
	vector<bool> flag(we.size(),false);

	sort(weight.begin(),weight.end());
	index.resize(k);
	for(int i=0;i<k;i++)
	{
		float pivot = weight[weight.size()-1-i];
		for(int j=0;j<we.size();j++)
		{
			if(pivot == we[j] && !flag[j]) 
			{
				flag[j] = true;
				index[i] = j; 
				break;
			}
		}
	}
	return ;
}

bool threadLocalization::getCurrScan()
{
	bool found = false;
	// static int cnt = 0;
	if(m_scanFile!=0 && m_scanFile->is_open() && !m_scanFile->eof())
	{
		if(m_cur_scan == 0) m_cur_scan = new PMScan(541);
		while(1/*!m_scanFile->eof()*/)
		{
			char line[4096];
			m_scanFile->getline(line,4096);
			if(m_scanFile->eof())
			{
				return false;
			}
			// cout<<cnt++<<" frames!"<<endl;
			if(!constructPSMfromRawSeed(line,*m_cur_scan,m_pPSM))
			{
				continue;
			}
			found = true;
			break;
		}
		if(!found) return false;
		cm2m(m_cur_scan);
	}else{
		cout<<"localization.cpp: failed to read scan file!"<<endl;
		return false;
	}
	return true;
}

void threadLocalization::localize2()
{
	if(m_cur_scan == 0)
	{
		if(!getCurrScan())
		{
			cout<<"localization.cpp: failed to enable localize!"<<endl;
			return ;
		}
	}
	m_bPFLocalization = true;
	cout<<"localization.cpp: enable pf localization!"<<endl;
}

void threadLocalization::localize()
{
	if(m_cur_scan == 0)
	{
		if(!getCurrScan())
		{
			cout<<"localization.cpp: failed to enable localize!"<<endl;
			return ;
		}
	}
	m_bDrawLocalization = true;
	cout<<"localization.cpp: enable draw localization!"<<endl;
}

void threadLocalization::globalize(int step)
{
	char line[4096];
	cout<<"localization.cpp: begin to globalize!"<<endl;

	for(int i=0;i<step;i++)
	{
		if(!getCurrScan())
		{
			cout<<"localization.cpp: file is finished!"<<endl;
			return;
		}
	}

	OrientedPoint2D pose;
	globalization(m_cur_scan, pose);
	// send scan to display
	vector<int> px;
	vector<int> py;
	int p[2];
	float theta = pose.theta;
	m_cur_scan->rx = pose.x;
	m_cur_scan->ry = pose.y;
	m_cur_scan->th = pose.theta;
	m_PMap->gen2Display(px,py,p,m_cur_scan);
	send2Scan(&px[0],&py[0],px.size(),p,theta);
	
	// send curr_scan to display
	sendRelScan(m_cur_scan->x,m_cur_scan->y,m_cur_scan->np,&dummy_x,&dummy_y,&dummy_th);

	// send sim to display
	if(m_sim_scan == 0) m_sim_scan = new PMScan(541);
	m_PMap->laserScanSimulator(m_sim_scan,pose);
	sendSimScan(m_sim_scan->x,m_sim_scan->y,m_sim_scan->np,&dummy_x,&dummy_y,&dummy_th);
	paintReady();
	cout<<"localization.cpp: finish globalize!"<<endl;
}

void threadLocalization::enableSM()
{
	m_bScanMatchSim = true;
	cout<<"localization.cpp: enable ScanMatch Simulator! ";
	cout<<" choose start point!"<<endl;
}

namespace {
	ofstream recordTK("topk.log");
	void recordScan(PMScan* ls, const char* file)
	{
		ofstream rs(file);
		for(int i=0;i<ls->np;i++)
		{
			rs<<ls->r[i]<<" "<<ls->x[i]<<" "<<ls->y[i]<<endl;
		}
	}
}

bool threadLocalization::runScanMatch(OrientedPoint2D& pose)
{
	static vector<OrientedPoint2D*> traj; 
	OrientedPoint2D rel_pose;
	static OrientedPoint2D sent_pose;
	bool first = false;
	m2cm(m_cur_scan);
	float ret = m_pCSM->FMatchKeyFrame(m_cur_scan);
	cm2m(m_cur_scan);
	if(ret < 0)
	{
		cout<<"localization.cpp: CSM failed but continue!"<<endl;
		return false;
	}
	if(traj.size()==0)
	{
		sent_pose = pose;
		first = true;
	}else{
		rel_pose = OrientedPoint2D(m_cur_scan->rx/100.,m_cur_scan->ry/100.,m_cur_scan->th);
		pose = traj[traj.size()-1]->oplus(rel_pose);
	}
	traj.push_back(new OrientedPoint2D(pose));
	rel_pose = sent_pose.ominus(pose);
	// cout<<"localization.cpp: rel_pose "<<rel_pose<<endl;
	if(!isSmallMove(rel_pose) || first)
	{
		// PF to update the state
		// cout<<"localization.cpp send rel_pose: "<<rel_pose<<endl;
		sent_pose = pose;
		// m_Particles->predict(rel_pose);
		// m_Particles->update(m_PMap,m_cur_scan);
		return true;
	}
	return false;
}

void threadLocalization::simulateSM(int px,int py)
{
	if(m_bScanMatchSim == false)
	{
		cout<<"localization.cpp: ScanMatch simulator is not enabled!"<<endl;
		return ;
	}
	m_bScanMatchSim = false;
	OrientedPoint2D g_pose(m_PMap->idx2x(px),m_PMap->idx2y(py),M_PI/2.);
	OrientedPoint2D robot_pose;
	cout<<"localization.cpp: start SM simulator !"<<endl;
	while(getCurrScan())
	{
		if(runScanMatch(robot_pose))
		{
			// send to display
			sendParticle(m_PMap->x2idx(robot_pose.x+g_pose.x),m_PMap->y2idx(-1.*robot_pose.y+g_pose.y));
			// cout<<"localization.cpp: send pose: "<<robot_pose<<endl;
			paintReady();
		}
	}
	cout<<"localization.cpp: finish SM simulator!"<<endl;
}

bool threadLocalization::runPFLocalization(OrientedPoint2D& outMP)
{
	static vector<OrientedPoint2D*> traj; 
	OrientedPoint2D pose;
	OrientedPoint2D rel_pose;
	static OrientedPoint2D sent_pose;
	bool first = false;
	m2cm(m_cur_scan);
	float ret = m_pCSM->FMatchKeyFrame(m_cur_scan);
	cm2m(m_cur_scan);
	if(ret < 0)
	{
		cout<<"localization.cpp: CSM failed but continue!"<<endl;
		return false;
	}
	if(traj.size()==0)
	{
		sent_pose = pose;
		first = true;
	}else{
		rel_pose = OrientedPoint2D(m_cur_scan->rx/100.,m_cur_scan->ry/100.,m_cur_scan->th);
		pose = traj[traj.size()-1]->oplus(rel_pose);
	}
	traj.push_back(new OrientedPoint2D(pose));
	rel_pose = sent_pose.ominus(pose);
	if(!isSmallMove(rel_pose) || first)
	{
		// PF to update the state
		// cout<<"localization.cpp start PF! rel_pose: "<<rel_pose<<endl;
		sent_pose = pose;
		m_Particles->predict(rel_pose);
		m_Particles->update(m_PMap,m_cur_scan);
		m_Particles->getMax(outMP);
		return true;
	}
	return false;
}

void threadLocalization::localization2(int px1, int py1, float angle)
{
	if(!m_bPFLocalization)
	{
		cout<<"localization.cpp: PF localization is not enabled!"<<endl;
		return ;
	}
	cout<<"localization.cpp: start PF localization from "<<px1<<" "<<py1<<endl;
	m_bPFLocalization = false;
	// OrientedPoint2D startPose(m_PMap->idx2x(px1),m_PMap->idx2y(py1),angle);
	float inipose[3] = {0,};
	m_Particles->init(inipose,3);
	inipose[0] = m_PMap->idx2x(px1);
	inipose[1] = m_PMap->idx2y(py1);
	inipose[2] = angle;
	m_Particles->setOriginalPose(inipose);

	OrientedPoint2D meanP;
	OrientedPoint2D maxP;
	vector<int> px, py;
	int p[2];
	int frames = 400;
	int f = 0;
	while(f<frames && !m_scanFile->eof())
	{
		while(1)
		{
			if(!getCurrScan())
			{
				cout<<"localization.cpp: finish PF localization."<<endl;
				return ;
			}
			if(runPFLocalization(maxP))
				break;
		}
		f++;	
		// get mean value
		// m_Particles->getMax(maxP);
		// cout<<"localization.cpp: send pose: "<<maxP<<"!"<<endl;
		m_Particles->getMean(maxP);
		// send pose to display
		m_cur_scan->rx = maxP.x;
		m_cur_scan->ry = maxP.y;
		m_cur_scan->th = maxP.theta;
		m_PMap->gen2Display(px,py,p,m_cur_scan);
		// send2Scan(&px[0],&py[0],px.size(),p,maxP.theta);
		send2Robot(p[0],p[1],maxP.theta);
		paintReady();
		
		// send real scan to display
		sendRelScan(m_cur_scan->x,m_cur_scan->y,m_cur_scan->np,&dummy_x,&dummy_y,&dummy_th);
		// send sim scan to display
		if(m_sim_scan == 0) m_sim_scan = new PMScan(541);
		m_PMap->laserScanSimulator(m_sim_scan,maxP);
		sendSimScan(m_sim_scan->x,m_sim_scan->y,m_sim_scan->np,&dummy_x,&dummy_y,&dummy_th);

	}
	cout<<"localization.cpp: finish PF localization."<<endl;
}

void threadLocalization::localization1(int px1, int py1, float angle)
{
	if(!m_bDrawLocalization)
	{
		cout<<"localization.cpp: localization is not enabled!"<<endl;
		return ;
	}

	OrientedPoint2D pose(m_PMap->idx2x(px1),m_PMap->idx2y(py1),angle);
	// cout<<"localization.cpp: recePose: "<<pose<<endl;
	vector<int> px;
	vector<int> py;
	int p1[2];
	float theta;
	if(m_PMap->isValidPose(pose))
	{
		/*float score = m_PMap->obsLikelyhood3(m_cur_scan,*pose);
		cout<<"localization.cpp: pose: "<<*pose<<" score: "<<score<<endl;*/
		if(!m_sim_scan)
		{
			m_sim_scan = new PMScan(541);
		}
		m_PMap->laserScanSimulator(m_sim_scan,pose);
		
		// send scan to display
		theta = pose.theta;
		m_sim_scan->rx = pose.x;
		m_sim_scan->ry = pose.y;
		m_sim_scan->th = pose.theta;
		m_PMap->gen2Display(px,py,p1,m_sim_scan);
		send2Scan(&px[0],&py[0],px.size(),p1,theta);

		// send simulated scan to display
		if(m_sim_scan!=0) 
		{
		sendSimScan(m_sim_scan->x,m_sim_scan->y,m_sim_scan->np,&dummy_x,&dummy_y,&dummy_th);
		}
		// send real scan to display
		if(m_cur_scan!=0) 
		{
		// cout<<"localization.cpp: send points: "<<m_cur_scan->np<<endl;
		sendRelScan(m_cur_scan->x,m_cur_scan->y,m_cur_scan->np,&dummy_x,&dummy_y,&dummy_th);
		}
		// recordScan(m_sim_scan,"simscan.log");
		// recordScan(m_cur_scan,"curscan.log");
		paintReady();
		
		// float score = m_PMap->obsLikelyhood(m_cur_scan,pose);
		float score = m_PMap->obsLikelyhood3(m_cur_scan,pose);
		cout<<"localization.cpp score: " <<score<<endl;
	}else
	{
		cout<<"localization.cpp: unvalid pose: "<<pose<<endl;
	}
}

void threadLocalization::globalization(PMScan* scan, OrientedPoint2D& gp)
{
	// generate particles in the valid area
	vector<OrientedPoint2D> pps(g_num_of_global_particles);
	vector<int> ppi(g_num_of_global_particles);
	vector<float> ppf(g_num_of_global_particles);

	// ofstream time_rec("m1_time.log");
	// struct timeval st,et;
	// gettimeofday(&st,0);
	OrientedPoint2D rel_pose;
	for(int i=0;i<g_num_of_global_particles;)
	{
 		randomPose(pps[i]);
		if(m_PMap->isValidPose(pps[i]))
		{
			// ppf[i] = m_PMap->obsLikelyhood3(scan, pps[i]);
			ppf[i] = m_PMap->obsLikelyhood(scan,pps[i], rel_pose);
			// ppf[i] = m_PMap->obsLikelyhood2(scan,pps[i]);
			// cout<<"localization.cpp: "<<i<<" score: "<<ppf[i]<<" pose: "<<pps[i]<<endl;
			ppi[i] = i;
			i++;
		}
	}
	// gettimeofday(&et,0);
	// time_rec<<(et.tv_sec-st.tv_sec)*1000.+(et.tv_usec-st.tv_usec)/1000.<<endl;
	topK(ppi, ppf);
	vector<int> px(g_num_of_particles);
	vector<int> py(g_num_of_particles);
	for(int i=0;i<g_num_of_particles;i++)
	{
		recordTK<<"localization.cpp: "<<ppi[i]<<" score: "<<ppf[ppi[i]]<<" "<<pps[ppi[i]]<<endl;
		px[i] = m_PMap->x2idx(pps[ppi[i]].x);
		py[i] = m_PMap->y2idx(pps[ppi[i]].y);
	}
	sendParticles(&px[0],&py[0],g_num_of_particles);
	gp = pps[ppi[0]];
	scan->rx = gp.x;
	scan->ry = gp.y;
	scan->th = gp.theta;
	
	/*ofstream ps("pose.log"); 
	// send scan to display
	for(int i=1;i<g_num_of_particles;i++)
	{
		vector<int> spx;
		vector<int> spy;
		ps<<pps[ppi[i]]<<endl;
		int p[2];
		float theta = pps[ppi[i]].theta;
		m_PMap->gen2Display(spx,spy,p,scan);
		p[0] = pps[ppi[i]].x;
		p[1] = pps[ppi[i]].y;
		send2Scan(&spx[0],&spy[0],spx.size(),p,theta);
	}*/
	// if()
	return ;
}

void threadLocalization::setScanFile(string file)
{
	if(m_scanFile!=0)
	{
		delete m_scanFile;
	}
	m_scanFile = new ifstream(file.c_str());
	if(!m_scanFile->is_open())
	{
		cout<<"localization.cpp: failed to set scanfile: "<<file<<endl;
		delete m_scanFile;
		m_scanFile = 0;
		return;
	}
}
