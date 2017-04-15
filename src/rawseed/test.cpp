/*
This test is to :
	1 find the relationship between the number of Flirt features and the accuracy of PSM.
	2 display the trajectory from psm, ODO and GT 
*/

#include <iostream>
#include <string>
#include "FlirterNode.h"
#include "ZHPolar_Match.h"
#include "gtReader.h"
#include "odoReader.h"

using namespace std;

string F_DIR = "/home/lyxp/work/mkproj/SVN/PFG/trunk/data/RawSeed/indoor/";
string GT_F = F_DIR + "GT_extended.log"; //"GROUNDTRUTH.log";
string SICK_F = F_DIR + "SICK_FRONT.log";
string ODO_F = F_DIR + "ODOMETRY_XYT.log";

ofstream record("record.log");
ofstream psmlog("psm.log");
ofstream odolog("odo.log");
ofstream gtlog("gt.log");
#define SQ(x) ((x)*(x))

double err_pose(OrientedPoint2D& gt, OrientedPoint2D& est){
	return sqrt(SQ(est.x-gt.x)+SQ(est.y-gt.y));
}

int main(int argc, char* argv[])
{
	CPolarMatch* pPSM = new CPolarMatch("LMS211");
	if(!pPSM->readCarmon(SICK_F,pPSM->m_pParam->pm_laser_name))
	{
		cout<<"failed to read file: "<<SICK_F<<endl;
		return -1;
	}
	GTReader* pGT = new GTReader;
	if(!pGT->readGT(GT_F.c_str())){
		cout<<"failed to read GT: "<<GT_F<<endl;
		return -1;
	}
	ODOReader* pODO = new ODOReader;
	if(!pODO->readODO(ODO_F.c_str())){
		cout<<"failed to read ODO: "<<ODO_F<<endl;
		return -1;
	}

	// GT record
	OrientedPoint2D GTrelpose;
	OrientedPoint2D GTlast;
	OrientedPoint2D GTcurr;

	// ODO record
	OrientedPoint2D ODOrelpose;
	OrientedPoint2D ODOlast;
	OrientedPoint2D ODOcurr;
	
	// run PSM 
	int cnt=0;
	PMScan* ls;
	CFliterNode* prefNode = NULL;
	while(cnt<pPSM->m_SickScans.size())
	{
		ls = pPSM->m_SickScans[cnt];
		cnt++;
		CFliterNode* pcurNode = new CFliterNode(ls,pPSM);
		GTcurr = OrientedPoint2D(pGT->x[cnt-1],pGT->y[cnt-1],pGT->th[cnt-1]);
		ODOcurr = OrientedPoint2D(pODO->x[cnt-1],pODO->y[cnt-1],pODO->th[cnt-1]);
		if(!prefNode){
			GTlast = GTcurr;
			ODOlast = ODOcurr;
			pcurNode->m_pose =  GTlast;
			prefNode = pcurNode;
		}
		else{
			// 0 Failed 1 PSM succeed 2 ICP succeed
			int status = pcurNode->matchNodeFrontend(prefNode);
			if(status == 0 ){
				//	record<<prefNode->m_featurePointsLocal.size()<<" "<<pcurNode->m_featurePointsLocal.size()<<" "<<100<<endl;
					pcurNode->m_pose = prefNode->m_pose.oplus(ODOlast.ominus(ODOcurr));
			}else{

				pcurNode->m_pose = prefNode->m_pose.oplus(pcurNode->m_relpose);
				// GET GT record
				/*GTlast = OrientedPoint2D(pGT->x[cnt-2],pGT->y[cnt-2],pGT->th[cnt-2]);
				GTcurr = OrientedPoint2D(pGT->x[cnt-1],pGT->y[cnt-1],pGT->th[cnt-1]);
				GTrelpose = GTlast.ominus(GTcurr);*/

				// record<<prefNode->m_featurePointsLocal.size()<<" "<<pcurNode->m_featurePointsLocal.size()<<" "<<err_pose(GTrelpose,pcurNode->m_relpose)<<endl;
			}
			
			delete prefNode;
			prefNode = pcurNode;
			ODOlast = ODOcurr;
			GTlast = GTcurr;
		}
		psmlog<<prefNode->m_pose.x<<" "<<prefNode->m_pose.y<<" "<<prefNode->m_pose.theta<<endl;
		odolog<<ODOlast.x<<" "<<ODOlast.y<<" "<<ODOlast.theta<<endl;
		gtlog<<GTlast.x<<" "<<GTlast.y<<" "<<GTlast.theta<<endl;
	}
	delete pPSM;
	delete pGT;
	delete pODO;
	return 0;
}
