#include <QApplication>
#include <iostream>
#include "dialog_server.h"
#include "serverBackend.h"
#include "clientFrontend.h"
#include "point.h"
#include "points.h"
#include "MapNode.h"
#include "MapGraph.h"
#include "ZHPolar_Match.h"
#include "ZHCanonical_Matcher.h"
#include "timestamp.h"

//using zhpsm::OrientedPoint2D;
using namespace std;

void sendtoDisplay(Dialog_Server* m_Dialog, PMScan& ls)
{
	const float bad_range = -100000;
	static vector<float> fx, fy;
	static double rx,ry,rth;
	if(fx.size() < ls.np) {
		fx.resize(ls.np);
		fy.resize(ls.np);
	}

	rx = ls.rx/100.;
	ry = ls.ry/100.;
	rth = ls.th;
	float x,y;
	float fcos = cosf(rth);
	float fsin = sinf(rth);
	for(int i=0;i<ls.np;i++)
	{
		if(ls.r[i] >= 5000)
		{
			fx[i] = fy[i] = bad_range;
		}else{
			x = ls.x[i]/100.;
			y = ls.y[i]/100.;
			fx[i] = fcos*x - fsin*y + rx;
			fy[i] = fsin*x + fcos*y + ry;
		}
	}

	m_Dialog->m_points_dis->receMapInfo(&fx[0],&fy[0],ls.np,&rx,&ry,&rth);
	m_Dialog->m_points_dis->paintReady();

}

int main(int argc, char *argv[])
{
	QApplication app(argc, argv);

	Dialog_Server* m_Dialog = new Dialog_Server;
	m_Dialog->show();

	string m_file = "/media/ShareRegion/Datasets/LMS151_20130219/t1.txt";
	int work_model = 0;
	bool use_cov = true;

	CMapGraph* m_pMGraph = new CMapGraph("LMS151");

	CServerBackend* m_pBackend = new CServerBackend(work_model, use_cov);
	CClientFrontend* m_pFrontend = new CClientFrontend;
	m_pFrontend->setFile(m_file);
	m_pFrontend->setModel(work_model);

	m_pFrontend->m_pPSM = new CPolarMatch("LMS151");
	m_pFrontend->m_pCSM = new CCanonicalMatcher(m_pFrontend->m_pPSM->m_pParam);

	PMScan ls(m_pFrontend->m_pPSM->m_pParam->pm_l_points);
	ifstream inf(m_file.c_str());
	if(!inf.is_open()){
		cout<<"in runCarmon() failed to open file: "<<m_file<<endl;
		return 0;
	}
	ofstream fout("oneThread.log");
	char line[8192];
	double cov2[6];
	bool bloop;
	while(!inf.eof() && inf.getline(line,8192))
	{

		TTimeStamp ts = getCurrentTime();

		//frontend
		if(!m_pFrontend->constructPSMfromRawSeed(line,ls,m_pFrontend->m_pPSM))
			continue;
		m_pFrontend->run(ls);


		//backend
		//create fliter node
		//cout<<m_pFrontend->scnt<<endl;

		if(!m_pFrontend->scnt)
		{
			int index = m_pFrontend->m_traj.size();

			ls.rx = m_pFrontend->m_traj[index - 1]->x*100;
			ls.ry = m_pFrontend->m_traj[index - 1]->y*100;
			ls.th= m_pFrontend->m_traj[index - 1]->theta;

			CFliterNode* pcurNode = new CFliterNode(new PMScan(ls),m_pFrontend->m_pPSM);
			pcurNode->m_psyn= 0;//m_pFrontend->m_syn_num;
			pcurNode->m_id = m_pFrontend->m_traj.size();

			vector<double> cov(6);
			memcpy(&cov[0],m_pFrontend->matrix,6*sizeof(double));
			pcurNode->setCov(cov);

			// create map node
			CMapNode* m_pMapNode = new CMapNode;
			int n_added_featrue = m_pMapNode->addPoseNode(pcurNode);

			if(!m_pMGraph->addMapNodeCov(m_pMapNode,cov2,bloop))
			{
				cout<<"MapNode failed to be added!"<<endl;
				delete m_pMapNode;
				continue;
			}
			CMapNode* plast = m_pMGraph->getLastNode();
			m_pFrontend->m_traj[index-1]->x = plast->m_rootPose->x;
			m_pFrontend->m_traj[index-1]->y = plast->m_rootPose->y;
			m_pFrontend->m_traj[index-1]->theta = plast->m_rootPose->theta;

			ls.rx = m_pFrontend->m_traj[index - 1]->x*100;
			ls.ry = m_pFrontend->m_traj[index - 1]->y*100;
			ls.th= m_pFrontend->m_traj[index - 1]->theta;

			sendtoDisplay(m_Dialog, ls);


			TTimeStamp te = getCurrentTime();
			fout<<timeDifference(ts, te)<<" "<<m_pFrontend->m_traj[index-1]->x<<" "<<m_pFrontend->m_traj[index-1]->y<<" "<<m_pFrontend->m_traj[index-1]->theta<<endl;
		}


	}

	fout.close();
	// cout<<"quit runfile()!"<<endl;

	return app.exec();
}

