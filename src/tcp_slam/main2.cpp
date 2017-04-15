#include <QApplication>
#include <iostream>
#include "dialog_client.h"
#include "serverBackend.h"
#include "MapNode.h"

#include "clientFrontend.h"
#include "point.h"
#include "MapGraph.h"
#include "ZHPolar_Match.h"
#include "ZHCanonical_Matcher.h"

// using zhpsm::OrientedPoint2D;
using namespace std;

int main(int argc, char *argv[])
{
	QApplication app(argc, argv);


	string m_file;
	int work_model;
	bool use_cov;

	//CMapGraph* m_pMGraph = new CMapGraph("LMS151");

	CServerBackend* m_pBackend = new CServerBackend(work_model, use_cov);
	CClientFrontend* m_pFrontend = new CClientFrontend;
	m_pFrontend->setFile(m_file);
	m_pFrontend->setModel(work_model);

	float x = m_pFrontend->m_traj[0]->x*100;

	/*
	m_pFrontend->m_pPSM = new CPolarMatch("LMS151");
	m_pFrontend->m_pCSM = new CCanonicalMatcher(m_pFrontend->m_pPSM->m_pParam);

	PMScan ls(m_pFrontend->m_pPSM->m_pParam->pm_l_points);
	ifstream inf(m_file.c_str());
	if(!inf.is_open()){
		cout<<"in runCarmon() failed to open file: "<<m_file<<endl;
		return 0;
	}
	ofstream fout("client.log");
	char line[8192];
	while(!inf.eof() && inf.getline(line,8192))
	{

		//frontend
		if(!m_pFrontend->constructPSMfromRawSeed(line,ls,m_pFrontend->m_pPSM))
			continue;
		m_pFrontend->run(ls);
		int index = m_pFrontend->m_traj.size();
		ls.x = m_pFrontend->m_traj[index - 1]->x*100;
		ls.y = m_pFrontend->m_traj[index - 1]->y*100;
		ls.th= m_pFrontend->m_traj[index - 1]->theta;



		//backend
		//create fliter node
		CFliterNode* pcurNode = new CFliterNode(new PMScan(ls),m_pFrontend->m_pPSM);
		pcurNode->m_psyn= 0;//m_pFrontend->m_syn_num;
		pcurNode->m_id = m_pFrontend->m_traj.size();

		vector<double> cov(6);
		memcpy(&cov[0],m_pFrontend->matrix,6*sizeof(double));
		pcurNode->setCov(cov);

		// create map node
		CMapNode* m_pMapNode = new CMapNode;
		int n_added_featrue = m_pMapNode->addPoseNode(pcurNode);


		if(!m_pMGraph->addMapNode(m_pMapNode))
		{
			cout<<"MapNode failed to be added!"<<endl;
			CMapNode* plast = m_pMGraph->getLastNode();
			*(m_pFrontend->m_traj[index-1]) = *(plast->m_rootPose);
		}
		


	}*/
	// cout<<"quit runfile()!"<<endl;

	//dialog.show();



	return app.exec();
}

