#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "MapNode.h"
#include "FlirterNode.h"
#include "ZHPolar_Match.h"
#include "MatchingResult.h"
#include <feature/InterestPoint.h>
#include <feature/BetaGrid.h>
#include "stdlib.h"


using namespace std;

vector<CMapNode*> BNodes;
vector<CFliterNode*> Nodes;
string file("/home/lyxp/work/mkproj/SVN/PFG/trunk/data/intel-lab.log");
// string file("mit-cscail.log");
// string file("fr079.log");

int constructNodes(string);
int constructBNodes(int);
bool constructPSMfromCarmon(char* line, PMScan& ls, CPolarMatch* m_pPSM);
int calculateMisMatches();
int randomMisMatches(int times, int T_n);
int randomMisMatchOneFrame();

bool bigTrans(OrientedPoint2D&, OrientedPoint2D&);
double disTrans(OrientedPoint2D&, OrientedPoint2D&);
void deleteBNodes();
void deleteNodes();

void recordfeatures(string, vector<InterestPoint*>& );

CMapNode* buildLocalMap(int index, int T_n);

int W_match;
int R_match;
double Error_dis;

int main(int argc, char* argv)
{
	cout<<"start expr1!"<<endl;
	ofstream record("fr_mis_rate.log");
	int step = 10;
	int total ;
	int mis;
	/*for(int i=0;i<Nodes.size();i+=5){
		CFliterNode* pRef = Nodes[i];
		for(int j=i+10;j<Nodes.size();j++){
			CFliterNode* pCur = Nodes[j];
			pair<int,double> ret = pCur->matchNodePairLocal(pRef,trans);
			if(ret.first >= 10){
				r++;
				cout<<"matched between "<<i<<" "<<j<<" "<<ret.first<<endl; 
			}
		}
	}
*/
	constructNodes("LMS211");
	// constructNodes("LMS511");
	total = randomMisMatchOneFrame();
	record<<"1 "<<" "<<total<<" "<<W_match<<" "<<R_match<<" "<<endl;

	// for(int i=10; i<=50; i+=step )
	/*for(int i=2;i<=6;i++)
	{
		// constructNodes("LMS211");
		constructNodes("LMS511");
		constructBNodes(i);
		// mis = calculateMisMatches();
		total = randomMisMatches(BNodes.size()/2,i);
		cout<<"Tn: "<<i<<" total: "<<total<<" mis: "<<W_match<<endl;
		record<<i<<" "<<total<<" "<<W_match<<" "<<R_match<<" "<<Error_dis<<endl;
		deleteBNodes();
		deleteNodes();
	}*/
	return 0;
}

CMapNode* buildLocalMap(int index, int T_n){
	CMapNode* ret = new CMapNode;
	int n_of_features = 0;
	int n_of_nodes = 0;
	cout<<"build local map from: "<<index<<endl;
	int T_d = T_n/10 + 1;
	while(1)
	{	
		if(index >= Nodes.size())
			break;
		int n_add_features = ret->addPoseNode(Nodes[index]);
		n_of_features+=n_add_features;
		n_of_nodes++;
		// if(n_of_features >= T_n)
		if(n_of_nodes >= T_n)
		// if(n_of_features >= T_n || n_of_nodes >= T_d)
			break;
		index++;
	}
	ret->finishReduction();
	ret->m_id = index;
	return ret;
}

int randomMisMatchOneFrame()
{
	int ret = 0;
	int r_match = 0;
	int w_match = 0;
	double err = 0;
	int N = Nodes.size();

	// brute force
	srand(time(NULL));
	
	int cnt = 0;
	vector<bool> flags(Nodes.size(),false);
	int rtimes = 500;
	for(int i=0;i<rtimes;i++){
		int index = rand()%N;
		if(flags[index]) continue;
		flags[index] = true;

		CFliterNode* pCur = Nodes[index];
		bool succ = false;
		double mean_dis = 0;
		int succ_times = 0;
		int threshold = pCur->m_featurePointsLocal.size()*0.7;
		if(threshold <=3 ) continue;
		cnt++;
		for(int j=0;j<Nodes.size();j++){
			if(j==i) continue;
			CFliterNode* pRef = Nodes[j];
			OrientedPoint2D trans;
			pair<int,double> ret = pCur->matchFeaturePoints(pRef->m_featurePointsLocal,pCur->m_featurePointsLocal,trans);
			if(ret.first >= threshold)
			{
				OrientedPoint2D gtrans;
				gtrans = pRef->m_gtpose.oplus(trans);
				if(bigTrans(pCur->m_gtpose,gtrans)){
					w_match ++;
				}else{
					succ_times ++;
					if(!succ) {
						r_match ++;
						succ = true;
					}
				}
			}

		}
	}
	
	cout<<"total: "<<cnt<<" Mis: "<<w_match<<" Rig: "<<r_match<<endl;
	R_match = r_match;
	W_match = w_match;
	return cnt;
}

int randomMisMatches(int times, int T_n)
{
	int ret = 0;
	int r_match = 0;
	int w_match = 0;
	double err = 0;
	cout<<"BNodes: "<<BNodes.size()<<endl;
	int N = Nodes.size() - 10;
	// brute force
	
	srand(time(NULL));
	
	int cnt = 0;
	vector<bool> flags(Nodes.size(),false);
	int rtimes = times > 500 ? 500:times;
	for(int i=0;i<rtimes;i++){
		int index = rand()%N;
		if(flags[index]) continue;
		flags[index] = true;

		CMapNode* pCur = buildLocalMap(index, T_n);
		if(pCur->m_featurePoints.size()<0.3*T_n){
			continue;
		}
		
		cnt++;
		bool succ = false;
		double mean_dis = 0;
		int succ_times = 0;
		for(int j=0;j<BNodes.size();j++){
			CMapNode* pRef = BNodes[j];
			MatchingResult mr;
			pCur->matchNodePair(pRef, mr);
			if(mr.id1>=0){
				OrientedPoint2D trans;
				OrientedPoint2D gtrans;
				((CFliterNode*)0)->fromSE22OrientedPoint(trans,mr.m_edge);
				gtrans = pRef->m_rootPose->oplus(trans);
				if(bigTrans(*(pCur->m_rootPose),gtrans)){
					w_match ++;
				}else{
					mean_dis += disTrans(*(pCur->m_rootPose),gtrans);
					succ_times ++;
					if(!succ) {
						r_match ++;
						succ = true;
					}
				}
			}

		}
		mean_dis = (mean_dis/(double)(succ_times));
		if(succ) Error_dis+=mean_dis;
	}
	
	cout<<"total: "<<cnt<<" Mis: "<<w_match<<" Rig: "<<r_match<<endl;
	R_match = r_match;
	W_match = w_match;
	Error_dis = err;
	return cnt;
}

int calculateMisMatches(){
	int ret = 0;
	int r_match = 0;
	int w_match = 0;
	double err = 0;
	cout<<"BNodes: "<<BNodes.size()<<endl;
	// brute force

	
	for(int i=0;i<BNodes.size();i+=5){
		CMapNode* pRef = BNodes[i];
		for(int j=i+5;j<BNodes.size();j++)
		{
			CMapNode* pCur = BNodes[j];
			MatchingResult mr;
			pCur->matchNodePair(pRef, mr);
			if(mr.id1>=0){
				OrientedPoint2D trans;
				OrientedPoint2D gtrans;
				((CFliterNode*)0)->fromSE22OrientedPoint(trans,mr.m_edge);
				gtrans = pRef->m_rootPose->oplus(trans);
				if(bigTrans(*(pCur->m_rootPose),gtrans)){
					cout<<"mismatch : "<<i<<" "<<j<<endl;
					ret++;
					w_match ++;
				}else{
					err += disTrans(*(pCur->m_rootPose),gtrans);
					cout<<"match : "<<i<<" "<<j<<endl;
					r_match ++;
				}
			}
		}
	}
	cout<<"Mis: "<<w_match<<" Rig: "<<r_match<<endl;
	R_match = r_match;
	W_match = w_match;
	Error_dis = err;
	return ret;
}

int constructBNodes(int T_n){
	CMapNode* pNode = new CMapNode;
	int n_of_features = 0;
	int n_of_nodes= 0;
	int T_d = T_n/10 + 1;
	
	cout<<"start to construct BNodes!"<<endl;
	for(int i=0;i<Nodes.size();i++){
		CFliterNode * new_node = Nodes[i];
		int n_added_features = pNode->addPoseNode(new_node);
		//pNode->finishReduction();
		//BNodes.push_back(pNode);
		//pNode = new CMapNode;

		n_of_features += n_added_features;
		n_of_nodes++;
		// if(n_of_features>=T_n){
		if(n_of_nodes>=T_n)
		// if(n_of_features>=T_n || n_of_nodes>=T_d)
		{
			pNode->finishReduction();
			pNode->m_id = BNodes.size();
			BNodes.push_back(pNode);
			pNode = new CMapNode;
			n_of_features = 0;
			n_of_nodes = 0;
		}
	}
	cout<<"construct BNodes: "<<BNodes.size()<<" at T_n: "<<T_n<<endl;
	return BNodes.size();
}

int constructNodes(string laser){
	ifstream inf(file.c_str());
	if(!inf.is_open()){
		cout<<"failed to open file: "<<file<<endl;
		return -1;
	}
	// char line[4096];
	char* line = new char[4096];
	int node_id = 0;
	CPolarMatch* m_pPSM = new CPolarMatch(laser.c_str());
	PMScan ls(m_pPSM->m_pParam->pm_l_points);
	
	cout<<"start to read file!"<<endl;
	while(inf.getline(line,4096)){
		if(!constructPSMfromCarmon(line,ls,m_pPSM))
		{
			// cout<<"failed to read line!"<<endl;
			continue;
		}
		CFliterNode * new_node = new CFliterNode(new PMScan(ls),m_pPSM);
		new_node->m_id = node_id++;
		new_node->m_pose = new_node->m_gtpose;
		// cout<<"node: "<<node_id-1<<" pose: "<<new_node->m_pose<<endl;
		Nodes.push_back(new_node);
	}

	cout<<"finish read files Nodes: "<<Nodes.size()<<endl;
	return Nodes.size();
}

bool constructPSMfromCarmon(char* line, PMScan& ls, CPolarMatch* m_pPSM){
	ls.rx=ls.ry=ls.th=0;
	char** last;
	string delim(" ");
	double start, fov, resolution, maxRange, accuracy;
	int laserType, remissionMode, num_points;
	
	string tag;

	// cout<<"before strtok() "<<line<<endl;
	tag = strtok(line,delim.c_str());
	// strtok(line,delim.c_str());
	// cout<<"line: "<<line<<endl;
	if(strcmp(tag.c_str(),"ROBOTLASER1")){
		//cout<<"this is a pose!"<<endl;
		return false;
	}
	laserType = atoi(strtok(NULL,delim.c_str()));
	start = atof(strtok(NULL,delim.c_str()));
	fov = atof(strtok(NULL,delim.c_str()));
	resolution = atof(strtok(NULL,delim.c_str()));
	maxRange = atof(strtok(NULL,delim.c_str()));
	accuracy = atof(strtok(NULL,delim.c_str()));
	remissionMode = atoi(strtok(NULL,delim.c_str()));
	num_points = atoi(strtok(NULL,delim.c_str()));
	if(num_points != m_pPSM->m_pParam->pm_l_points && \
		num_points != m_pPSM->m_pParam->pm_l_points-1){
		cout<<"not enough points!"<<endl;
		return false;
	}
	static bool bFirst_carmon = true;
	// set parameters
	if(bFirst_carmon){
		bFirst_carmon = false;
		m_pPSM->m_pParam->pm_fi_min = start;
		m_pPSM->m_pParam->pm_fi_max = start + fov*PM_D2R;
		m_pPSM->pm_init();
		if(maxRange*100 != m_pPSM->m_pParam->pm_max_range)
			m_pPSM->m_pParam->pm_max_range = maxRange*100;
	}
	// read bearings
	for(int i=0;i<num_points;i++){
		ls.r[i] = atof(strtok(NULL,delim.c_str()))*100.0; // from [m] 2 [cm]
		ls.x[i] = ls.r[i] * m_pPSM->pm_co[i];
		ls.y[i] = ls.r[i] * m_pPSM->pm_si[i];
		ls.bad[i] = 0;
		if(ls.r[i]<m_pPSM->m_pParam->pm_min_range){
			ls.r[i]=m_pPSM->m_pParam->pm_max_range+1; //set it to a value larger than the max
			ls.bad[i] = 1;
		}
	}
	if(num_points == m_pPSM->m_pParam->pm_l_points-1){
		ls.r[num_points] = m_pPSM->m_pParam->pm_max_range+1;
		ls.x[num_points] = 0;
		ls.y[num_points] = 0;
		ls.bad[num_points] = 1;
	}
	// remission data
	strtok(NULL,delim.c_str());
	// laser pose
	ls.rx = atof(strtok(NULL,delim.c_str())) * 100;
	ls.ry = atof(strtok(NULL,delim.c_str())) * 100;
	ls.th = atof(strtok(NULL,delim.c_str())) * 100;

	return true;
}

void recordfeatures(string fname, vector<InterestPoint*>& fp)
{	
	ofstream of(fname.c_str());
	cout<<"number : "<<fp.size()<<endl;
	for(int i=0;i<fp.size();i++){
		of<<fp[i]->getPosition()<<" "<<endl;
		const BetaGrid* pdes = dynamic_cast<const BetaGrid*>(fp[i]->getDescriptor());
		BetaGridGenerator* pgen = dynamic_cast<BetaGridGenerator*>(	CFliterNode::g_descriptor);
		for(int j=0;j<pgen->getBinRho();j++){
			for(int k=0;k<pgen->getBinPhi();k++){
				of<<pdes->getHistogram()[k][j]<<" ";
			}
		}
		of<<endl;
	}
	of.close();
}
#define SQUARE(x) ((x)*(x))
bool bigTrans(OrientedPoint2D& p1, OrientedPoint2D& p2){
#define PI 3.141592654
#define R2D(r) (180.*r/PI)
	if(SQUARE(p1.x-p2.x) + SQUARE(p1.y-p2.y) >= 0.25)
		return true;
	// if(R2D(p1.theta) - R2D(p2.theta))
	return false;
}
double disTrans(OrientedPoint2D& p1, OrientedPoint2D& p2)
{
	return (SQUARE(p1.x-p2.x)+SQUARE(p1.y-p2.y));
}

void deleteNodes(){
	for(int i=0;i<Nodes.size();i++)
	{
		delete Nodes[i];
		Nodes[i] = NULL;
	}
	Nodes.clear();
}

void deleteBNodes(){
	for(int i=0;i<BNodes.size();i++){
		delete BNodes[i];
		BNodes[i]=NULL;
	}
	BNodes.clear();
}

	/*OrientedPoint2D trans;
	CFliterNode* pR1 = Nodes[0];
	CFliterNode* pC1 = Nodes[95];
	pair<int,double> ret = pC1->matchFeaturePoints(pR1->m_featurePointsLocal,pC1->m_featurePointsLocal,trans);
	cout<<"Matched "<<ret.first<<endl;
	recordfeatures("pr1.log",pR1->m_featurePointsLocal);
	recordfeatures("pc1.log",pC1->m_featurePointsLocal);

	CMapNode* pR2 = new CMapNode;
	CMapNode* pC2 = new CMapNode;
	pR2->addPoseNode(pR1);
	pC2->addPoseNode(pC1);
	pR2->finishReduction();
	pC2->finishReduction();
	MatchingResult mr;
	pC2->matchNodePair(pR2, mr);
	
	recordfeatures("pr2.log",pR2->m_featurePoints);
	recordfeatures("pc2.log",pC2->m_featurePoints);
*/

