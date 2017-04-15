#include "localv1.h"
#include "globaldef.h"
#include "VPmap.h"
#include "PolarParameter.h"
#include "ZHPolar_Match.h"
#include "particles.h"
#include "ZHCanonical_Matcher.h"
#include <QImage>
#include <QThread>
#include <QMutexLocker>

LocalV1::LocalV1():
m_bStopLocalization(false),
m_bSynCoordinate(false),
m_bFinishInit(false)
{}
LocalV1::~LocalV1()
{
	for(int i=0;i<m_pPose.size();i++)
	{
		delete m_pPose[i];
		m_pPose[i] = 0;
		delete m_pScan[i];
		m_pScan[i] = 0;
	}
	delete m_pFirstFrame;
	delete m_synPose;
}

void LocalV1::init(const char* imgfile, int start_x, int start_y)
{
	m_imgfile = string(imgfile);
	m_ori_x = start_x;
	m_ori_y = start_y;
}

void LocalV1::runLocalization()
{
	cout<<"localv1.cpp: threadLoacalization starts!"<<endl;
	// 1, initialize input image
	if(m_PMap == 0) m_PMap = new CVPmap;
	QImage *pm = new QImage(m_imgfile.c_str());
	if(pm->isNull())
	{
		cout<<"LocalV1: failed to read image: "<<m_imgfile<<endl;
		return;
	}
	if(!receivePMAP(pm)) return ;
	
	// 2, initialize map coordinate
	if(m_Particles == 0) m_Particles = new CParticles(g_num_of_particles,3);
	float inipose[3] = {0,};
	m_Particles->init(inipose,3);
	inipose[0] = m_PMap->idx2x(m_ori_x);
	inipose[1] = m_PMap->idx2y(m_ori_y);
	inipose[2] = M_PI/2.;
	m_Particles->setOriginalPose(inipose);
	
	// syn coordinates
	cout<<"FISNISHED BEGIN! "<<inipose[0]<<" "<<inipose[1]<<" "<<inipose[2]<<endl;
	while(1)
	{

		if(!m_bSynCoordinate)
		{
			QThread::yieldCurrentThread();
			continue;
		}
		OrientedPoint2D ori_pose;
		m_Particles->predict2(ori_pose);
		cm2m(m_pFirstFrame);
		m_Particles->update(m_PMap,m_pFirstFrame);
		m_synPose = new OrientedPoint2D(*(m_Particles->getMean2()));
		cout<<"localv1.cpp: finish syn coordinate!"<<" "<<*m_synPose<<endl;
		break;
	}

	cout<<"FISNISHED OVER"<<endl;

	OrientedPoint2D meanP;
	vector<OrientedPoint2D*> pose;
	vector<PMScan*> scan;
	vector<int> syn;
	OrientedPoint2D t_pose;
	while(1)
	{
		{
			QMutexLocker locker(&m_mutex);
			if(m_pPose.size()<=0)
			{
				if(m_bStopLocalization)
				{
					break;
				}else{
					QThread::yieldCurrentThread();
					continue;
				}
			}
			pose.swap(m_pPose);
			scan.swap(m_pScan);
			syn.swap(m_syn_num);
		}
		for(int i=0;i<pose.size();i++)
		{
			bool bIgnore = false;
			/* m_pPSM->pm_segment_scan(scan[i]);
			if(m_pPSM->pm_is_corridor(scan[i]))
			{
				bIgnore = true;
			}else*/

			{	
				t_pose = m_synPose->oplus(*pose[i]);
				m_Particles->predict2(t_pose/**pose[i]*/);
				cm2m(scan[i]);
				bIgnore = m_Particles->update(m_PMap, scan[i]);
				t_pose = m_synPose->ominus(*(m_Particles->getMean2()));
			}
			// sendUpdatePose(syn[i],(void*)(m_Particles->getMean2()));
			sendUpdatePose(syn[i],(void*)(&t_pose), bIgnore);
			//cout<<"sendUpdatePose: "<<syn[i]<<" "<<t_pose<<endl;
		}
		for(int i=0; i<pose.size(); i++)
		{
			delete pose[i];
			pose[i] = 0;
			delete scan[i];
			scan[i] = 0;
		}
		pose.clear();
		scan.clear();
		syn.clear();
	}
	cout<<"localv1.cpp: localization quit!"<<endl;
	finished();
}

void LocalV1::stop()
{
	// cout<<"localv1.cpp: set threadLocal quit flag!"<<endl;
	m_bStopLocalization = true;
}

void LocalV1::receFirstFrame(void* l)
{
	//cout<<"LocalV1::receFirstFrame: "<<endl;
	PMScan* ls = static_cast<PMScan*>(l);
	m_pFirstFrame = new PMScan(*ls);
	m_bSynCoordinate = true;
}

void LocalV1::recePoseScan(void* l, int syn)
{
	//cout<<"LocalV1::recePoseScan: "<<syn<<endl;

	/*if(!m_bFinishInit)
	{
		cout<<"LOCALV1 NOT READY!"<<endl;
		return ;
	}*/

	PMScan* ls = static_cast<PMScan*>(l);
	//cout<<"LocalV1::recePoseScan: "<<syn<<" "<<ls->rx<<" "<<ls->ry<<" "<<ls->th<<endl;
	{
	QMutexLocker locker(&m_mutex);
	m_pScan.push_back(new PMScan(*ls));
	m_pPose.push_back(new OrientedPoint2D(ls->rx/100.,ls->ry/100.,ls->th));
	m_syn_num.push_back(syn);
	}
}

