#include "MapGraph.h"
#include "MatchingResult.h"
#include "ZHPolar_Match.h"
#include <sensorstream/CarmenLog.h>
#include <geometry/point.h>
#include <feature/InterestPoint.h>

#include <string.h>
#include <stdlib.h>
#include <utility>

#include "g2o/math_groups/se2.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/math_groups/se3quat.h"
#include "g2o/types/slam3d/edge_se3_quat.h"

#include "g2o/apps/g2o_incremental/graph_optimizer_sparse_incremental.h"
#include "g2o/apps/g2o_interactive/graph_optimizer_sparse_online.h"
#include "g2o/core/graph_optimizer_sparse.h"
#include "g2o/core/hyper_dijkstra.h"
#include "g2o/core/block_solver.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"

#include "MapNode.h"
#include "FlirterNode.h"

#define LASER_SICK_NUM		361
#define M_PI 3.141592654
#define D2R(d) (d*M_PI/180.0) 
#define R2D(r) (r*180.0/M_PI)
#define MIN_ANGLE_D 0
#define MAX_ANGLE_D 180
#define MIN_ANGLE_R 0
#define MAX_ANGLE_D M_PI
#define LASER_BEARING_D	0.5
#define LASER_BEARING_R 0.008726646261
#define MAX_LASER_RANGE 50.0

#define DIS_THRESHOLD 1.50	// max distance translation 1.5m
#define ICP_QUALITY_THRESHOLD 0.8 // icp quality threshold same direction
#define ICP_QUALITY_REVERSE_THRESHOLD 0.45 // 0.5 // icp quality threshold different direction

namespace{
	float disPoeses(OrientedPoint2D& p1, OrientedPoint2D& p2){
		return sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));
	}
	bool statis(vector<float>& x, float& mean, float& sigma){
		if(x.size() < 3) {
			return false;	
		}
		float sum = 0;
		for(int i=0;i<x.size();i++)
			sum+=x[i];
		mean = sum/x.size();
		float square_sum = 0;
		for(int i=0;i<x.size();i++)
			square_sum += (x[i]-mean)*(x[i]-mean);
		sigma = square_sum/(x.size()-1);
		return true;
	}
	void getMatrix(double cov[6], Eigen::Matrix3d& M){
		cov[0] = M(0,0); cov[1] = M(0,1); cov[2] = M(0,2);
				 cov[3] = M(1,1); cov[4] = M(1,2);
				 		  cov[5] = M(2,2);
	}

	// normal Angle to [base, base+2PI)
	double normalAng(double angle, double base){
		double pi2 = 2*M_PI;
		double min2pi = base+pi2;
		while(angle >= min2pi) angle -=pi2;
		while(angle < base) angle += pi2;
		return angle;
	}
}

CMatchedPair::CMatchedPair(int id, float goodness, OrientedPoint2D& pose, MatchingResult* mr):
m_id(id),
m_good(goodness),
m_pose(new OrientedPoint2D(pose)),
m_mr(mr)
{
	// cout<<"MapGraph: MatchedPair has good: "<<m_good<<endl;
}

CMatchedPair::CMatchedPair(const CMatchedPair& other){
	m_id = other.m_id;
	m_good = other.m_good;
	m_pose = new OrientedPoint2D(*(other.m_pose));
	m_mr = new MatchingResult(*(other.m_mr));
}

CMatchedPair::~CMatchedPair(){
	if(m_pose) delete m_pose;
}

CMatchedPair& CMatchedPair::operator=(const CMatchedPair& other){
	if(this == &other){
		return *this;
	}
	m_id = other.m_id;
	m_good = other.m_good;
	m_pose = new OrientedPoint2D(*(other.m_pose));
	m_mr = new MatchingResult(*(other.m_mr));
	return *this;
}

int CMapGraph::calculateFinalPose(CMPSet& mset, OrientedPoint2D& pose, double cov[]){
	float min_dis = 1000000;
	int index = -1;
	// OrientedPoint2D pose;
	calculateWeightedPose(mset,pose);
	// cout<<"final pose: "<<pose<<endl;
	for(int i=0;i<mset.size();i++){
		float dis = disPoeses(pose, *(mset[i].m_pose));
		// cout<<"cur_pose "<<*(mset[i].m_pose)<<endl;
		if(dis < min_dis){
			min_dis = dis;
			index = i;
		}
	}
	if(index == -1){
		cout<<"MapGraph Yade !"<<endl;
		return -1;
	}
	Eigen::Matrix3d tmp_cov = mset[index].m_mr->m_edge.information().inverse();
	map<int,CMapNode*>::iterator it = m_mapGraph.find(mset[index].m_id);
	if(it == m_mapGraph.end()){
		cout<<"MapGraph: error in calculateFinalPose!"<<endl;
		return -1;
	}
	OrientedPoint2D trans = it->second->m_rootPose->ominus(pose);
	((CFliterNode*)0)->fromOrientedPoint2SE2(trans,mset[index].m_mr->m_edge);
	getMatrix(cov,it->second->m_root_Cov);
	cov[0] += tmp_cov(0,0);
	cov[3] += tmp_cov(1,1);
	cov[5] += tmp_cov(2,2);
	// *(new_node->m_rootPose) = pose;
	// *(new_node->m_rootPose) = it->second->m_rootPose->oplus(trans);
	// *(new_node->m_rootPoseBack) = *(new_node->m_rootPose);
	return index;
}

void CMapGraph::calculateWeightedPose(CMPSet& mset, OrientedPoint2D& pose)
{
	float total_num = 0;
	// calculate weights
	for(int i=0;i<mset.size();i++){
		// cout<<i<<" has "<<mset[i].m_good;
		total_num += mset[i].m_good;
	}
	vector<float> weights(mset.size());
	float px = 0;
	float py = 0;
	float pth = 0;
	// cout<<"total_num: "<<total_num<<endl;
	for(int i=0;i<mset.size();i++){
		weights[i] = (float)mset[i].m_good/(float)total_num;
		// cout<< weights[i]<<" with "<<mset[i].m_good<<endl;
		px += weights[i]*mset[i].m_pose->x;
		py += weights[i]*mset[i].m_pose->y;
		pth += weights[i]*mset[i].m_pose->theta;
	}
	pose = OrientedPoint2D(px,py,pth);
}

void CMapGraph::deleteMisMatch(CMPSet& mset){
	/*CMPSet::iterator it = mset.begin();
	CMPSet::iterator it_prev = it;
	it++;
	vector<float> dis_set;
	// statistic distance between poses
	while(it!=mset.end()){
		dis_set.push_back(disPoeses(*((*it_prev).m_pose),*((*it).m_pose)));
		it++;
		it_prev++;
	}*/
	vector<float> dis_set;
	for(int i=0;i<mset.size();i++){
		float mean_dis = 0;
		for(int j=0;j<mset.size();j++){
			if(j==i) continue;
			mean_dis += disPoeses(*(mset[i].m_pose),*(mset[j].m_pose));
		}
		dis_set.push_back(mean_dis/(mset.size()-1));
	}
	float mean, sigma;
	if(!statis(dis_set,mean,sigma)){
		// cout<<"MapGraph: matched_num: "<<dis_set.size()<<" too little!"<<endl;
		return; 
	}
	cout<<"mean : "<<mean<<" sigma: "<<sigma<<endl;
	float scale = 1.731;
	
	vector<float>::iterator it_dis = dis_set.begin();
	CMPSet::iterator it_pair = mset.begin();
	int num_deleted = 0;
	while(it_dis!=dis_set.end()){
		if(*it_dis > mean + scale*sigma){
			it_pair = mset.erase(it_pair);
			num_deleted++;
		}else
			it_pair++;
		it_dis++;
	}
	cout<<"delete mismatches: "<<num_deleted<<endl;
}


typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  SlamBlockSolver;
// what 's the meaning of BlockSolverTraits<m,n>
//typedef g2o::BlockSolver< g2o::BlockSolverTraits<6, 3> >  SlamBlockSolver;
typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearCholmodSolver;
//typedef g2o::LinearSolverPCG<SlamBlockSolver::PoseMatrixType> SlamLinearPCGSolver;
//typedef std::map<int, g2o::VertexSE3*> VertexIDMap;
typedef std::tr1::unordered_map<int, g2o::HyperGraph::Vertex*>     VertexIDMap;
//std::tr1::unordered_map<int, g2o::HyperGraph::Vertex* >
typedef std::set<g2o::HyperGraph::Edge*> EdgeSet;


CMapGraph::CMapGraph(string laser_name):
		#ifdef SAVE_PMAP
			m_record_matched("matched_nodes.log"),
		#endif
			optimizer_(0),
			m_reset_request(false),
			b_delayed_g2o(false),
			m_bUseConstantCov(true), // false
			m_pPSM(new CPolarMatch(laser_name.c_str()))
{
#ifndef ONLINE
	// allocating the optimizer
	optimizer_ = new g2o::SparseOptimizer();
	//SlamLinearSolver* linearSolver = new SlamLinearSolver();
	SlamLinearCholmodSolver* linearSolver = new SlamLinearCholmodSolver();
	//SlamLinearPCGSolver* linearSolver = new SlamLinearPCGSolver();
	linearSolver->setBlockOrdering(false);
	SlamBlockSolver* solver = new SlamBlockSolver(optimizer_, linearSolver);
	optimizer_->setSolver(solver);
#else
	optimizer_ = new g2o::SparseOptimizerIncremental();
	optimizer_->initSolver(3,0);
	// optimizer_ = new g2o::SparseOptimizerOnline();
#endif
	optimizer_->setVerbose(true);
	// for constant covariance
	float adj_val = 50;
	float loop_val = 10;
	float corridor_val = 0.5;
	m_cov_adjnode.fill(0.);
	m_cov_loopnode.fill(0.);
	m_cov_corridor.fill(0.);
	for(int i=0;i<3;i++){
		m_cov_adjnode(i,i) = adj_val;
		m_cov_loopnode(i,i) = loop_val;
		m_cov_corridor(i,i) = corridor_val;
	}
	m_cov_corridor(2,2) = adj_val; // loop_val;
}
CMapGraph::~CMapGraph(){

	//TODO: delete all Nodes
	//for (unsigned int i = 0; i < optimizer_->vertices().size(); ++i) {
	std::map<int,CFliterNode*>::iterator it = m_graph.begin();
	while(it!=m_graph.end())
	{
		delete it->second;
		it->second = NULL;
		it++;
	}
	m_graph.clear();
	for(int i=0;i<m_loopEdges.size();i++){
		delete m_loopEdges[i];
		m_loopEdges[i] = 0;
	}

	/*std::map<int,OrientedPoint2D*>::iterator it_tr = m_before_opt.begin();
	while(it_tr!=m_before_opt.end())
	{
		delete it_tr->second;
		it_tr->second = NULL;
		it_tr++;
	}
	m_before_opt.clear();*/
	delete (optimizer_);
	delete (m_pPSM);
}

bool CMapGraph::runSicklog()
{
	// firstly we only match two frames
	if(m_log.size()<=0){
		cout<<"log is empty!"<<endl;
		return false;
	}

	for(int i=0;i<m_log.size();i++){
		cout<<m_log.size()<<"	Step:	"<<i<<endl;
		LaserReading* pcurRead = dynamic_cast<LaserReading*> (m_log[i]);
		// create Flirter Node 
		CFliterNode * pcurNode = new CFliterNode(pcurRead);
		if(pcurNode==NULL || pcurNode->m_featurePoints.size()<=0){
			cout<<"failed to create node at frame: "<<i+1<<endl;
			continue;
		}
		// this is first frame
		if(m_graph.size()==0){
			//pcurNode->m_pose=OrientedPoint2D(0,0,0);
			pcurNode->m_relpose = pcurNode->m_pose;
		}
		else{
			CFliterNode * prefNode = (*m_graph.rbegin()).second;
			OrientedPoint2D transform;
			//if(!pcurNode->matchNodePairGlobal(prefNode,transform))
			pair<int,double> ret = pcurNode->matchNodePairGlobal(prefNode,transform); 
			if( ret.first<=0 || ret.second <0 )
			{
				cout<<"failed to match "<<i+1<<" with "<<i<<endl;
				continue;
			}
		}
		m_graph.insert(make_pair(m_graph.size(),pcurNode));
	}
	return true;
}

bool CMapGraph::readSicklog(string logfile, std::vector<AbstractReading*>& log)
{
	std::ifstream infile(logfile.c_str());
	if(!infile.is_open()){
		cout<<"failed to read file!"<<endl;
		return false;
	}
	char line[8192];
	std::vector<double> phi(LASER_SICK_NUM);
//	std::vector<double> cphi(LASER_SICK_NUM);
//	std::vector<double> sphi(LASER_SICK_NUM);
	for(int i=0;i<LASER_SICK_NUM;i++){
		phi[i]	=	 MIN_ANGLE_R + i*LASER_BEARING_R;
//		cphi[i] = cos(phi[i]);
//		sphi[i] = sin(phi[i]);
	}
	
	std::vector<double> rho(LASER_SICK_NUM);
	std::vector<double> remission;
	OrientedPoint2D laserPose,robotPose;
	while(infile.getline(line,8192))
	{
		strtok(line, " ");
		int N = (int)(atof(strtok(NULL," ")));
		if(N!=LASER_SICK_NUM){
			cout<<"laser number is not right:  file: "<<N<<" Laser: "<<LASER_SICK_NUM<<endl;
			return false;
		}
		for(int i=0;i<N;i++){
			float tmp= atof(strtok(NULL," "));
			if(tmp==0) rho[i] = MAX_LASER_RANGE;
			else rho[i] = tmp;
		}

		LaserReading* frame = new LaserReading(phi,rho);
		
		frame->setMaxRange(MAX_LASER_RANGE);
		//frame->setRemission(remission);
		//frame->setLaserPose(laserPose);
		log.push_back(frame);
	}
	return true;
}

// read ground-truth in carmon format
bool CMapGraph::readGTCarmon(string filename){
	return m_pPSM->readCarmon(filename,"ROBOTLASER1");
}
void CMapGraph::runGTCarmon(int run_num){
	return m_pPSM->runlog(run_num);
}

// read Our carmon data
bool CMapGraph::readOurCarmon(string filename)
{
	return m_pPSM->readCarmon(filename,m_pPSM->m_pParam->pm_laser_name);	
}
void CMapGraph::runOurCarmon(int run_num)
{
	return m_pPSM->runlogImproved(run_num);
}
// read Our RawSeed data
bool CMapGraph::readOurRawSeed(string filename)
{
	return m_pPSM->readRawSeed(filename);
}


// compare frontend with flirt
void CMapGraph::runComparison(int run_num)
{
	if(run_num<0)
		run_num = m_pPSM->m_SickScans.size()+1;

	// Make comparison between PSM with Flirt
	ofstream frontend("/mnt/hgfs/SharedFold/log/frondend.log");
	ofstream backend("/mnt/hgfs/SharedFold/log/backend.log");
	
	int cnt=0;
	PMScan* ls;
	while(cnt<m_pPSM->m_SickScans.size() && cnt<run_num)
	{
		ls = m_pPSM->m_SickScans[cnt];
		cnt++;
		
		CFliterNode* pcurNode = new CFliterNode(ls,m_pPSM);
	
		if(addNodeImproved(pcurNode)){
			std::cout<<"succeed to add Node: "<<cnt<<std::endl;
		}else{
			std::cout<<"failed to add Node: "<<cnt<<std::endl;
			delete pcurNode;
		}		
	}
	// In the final step using g2o offline optimization
	// optimizer_->optimize(1,false); // 1 time offline optimization 

	std::cout<<"finish adding Nodes."<<std::endl;
	std::map<int,CFliterNode*>::iterator it = m_graph.begin();
	while(it!=m_graph.end()){
		// record pose info from frontend
		frontend<<it->second->m_pose<<std::endl;
		// record pose info from backend
		g2o::VertexSE2* pVertex=dynamic_cast<g2o::VertexSE2*>(optimizer_->vertex(it->first));
		//Eigen::Vector3d ptrans = pVertex->estimate().toVector();
		double translation[3];
		pVertex->getEstimateData(translation);
		backend<<translation[0]<<" "<<translation[1]<<" "<<translation[2]<<std::endl;
		it++;
	}
	/*for(int i=0;i<optimizer_->vertices().size();i++){
		g2o::VertexSE2* pVertex=dynamic_cast<g2o::VertexSE2*>(optimizer_->vertex(i));
		//Eigen::Vector3d ptrans = pVertex->estimate().toVector();
		double translation[3];
		pVertex->getEstimateData(translation);
		traj_FL<<translation[0]<<" "<<translation[1]<<" "<<translation[2]<<std::endl;
	}*/
}
void CMapGraph::testGT1(string outfile, int num_of_frame){
	if(m_pPSM->m_SickScans.size()<=0){
		std::cout<<"No input data!"<<std::endl;
		return ;
	}
	ofstream outf(outfile.c_str());
	if(!outf.is_open()){
		cout<<"cannot open file: "<<outfile<<endl;
		return ;
	}
	int match_status = -1; // identify matching status
	int cnt=0;
	PMScan* ls;
	if(num_of_frame < 0)
		num_of_frame = m_pPSM->m_SickScans.size()+1;
	while(cnt<m_pPSM->m_SickScans.size() && cnt<num_of_frame)
	{
		ls = m_pPSM->m_SickScans[cnt];
		cnt++;
		match_status = -1;
		CFliterNode* pcurNode = new CFliterNode(ls,m_pPSM);
		// pcurNode->m_pFMatch->pm_preprocessScan(pcurNode->m_pScan);
		
		if(m_graph.size()<=0)
			m_graph.insert(make_pair(0,pcurNode));
		else
		{
			CFliterNode* prefNode = m_graph[m_graph.size()-1];
			try{
				if(!pcurNode->matchNodeFrontendGT(prefNode,match_status)){
					std::cout<<"FMatch failed!"<<std::endl;
					delete pcurNode;
					continue;
				}
				// pcurNode->m_pose = prefNode->m_pose.oplus(pcurNode->m_relpose);
				m_graph.insert(make_pair(m_graph.size(),pcurNode));
			}catch(int err){
				std::cout<<"FMatch failed!"<<std::endl;
				delete pcurNode;
			}
		}
	}	
	map<int, CFliterNode*>::iterator it = m_graph.begin();
	while(it!=m_graph.end()){
		outf<<it->second->m_pose<<endl;
		it++;
	}
	outf.close();
	
	return ;
}




void CMapGraph::synPoseAfterOpt(){
	for(int i=1;i<m_mapGraph.size();i++){
		CMapNode* pret = m_mapGraph[i];
		g2o::VertexSE2* pVertex=dynamic_cast<g2o::VertexSE2*>(optimizer_->vertex(pret->m_id));
		double translation[3];
		if(pVertex==NULL){
			std::cout<<"pVertex is NULL!"<<std::endl;
			continue;
		}
		pVertex->getEstimateData(translation);
		*(pret->m_rootPose) = OrientedPoint2D(translation[0],translation[1],normalAng(translation[2],-M_PI));
	}
}

// return the updated pose info
CMapNode* CMapGraph::getLastNode()
{
	if(m_mapGraph.size()<=0){
		std::cout<<"MapGraph is empty!"<<std::endl;
		return NULL;
	}
	CMapNode* pret = m_mapGraph[m_mapGraph.size()-1];
	if(b_delayed_g2o) return pret;
	g2o::VertexSE2* pVertex=dynamic_cast<g2o::VertexSE2*>(optimizer_->vertex(pret->m_id));
	double translation[3];
	if(pVertex==NULL){
		std::cout<<"pVertex is NULL!"<<std::endl;
		return NULL;
	}
	pVertex->getEstimateData(translation);
	OrientedPoint2D rootpose(translation[0],translation[1],translation[2]);
	*(pret->m_rootPose) = rootpose;
	return pret;
}

void CMapGraph::testGT(string outfile, int num_of_frame){
	if(m_pPSM->m_SickScans.size()<=0){
		std::cout<<"No input data!"<<std::endl;
		return ;
	}
	
	int match_status = -1; // identify matching status
	int n_of_gt = 0; // number of pose using gt
	int n_of_angle = 0; // number for big angle
	int n_of_err = 0; // number for big err

	int cnt=0;
	PMScan* ls;
	
	if(num_of_frame < 0)
		num_of_frame = m_pPSM->m_SickScans.size()+1;
	while(cnt<m_pPSM->m_SickScans.size() && cnt<num_of_frame)
	{
		ls = m_pPSM->m_SickScans[cnt];
		cnt++;
		match_status = -1;
		
		CFliterNode* pcurNode = new CFliterNode(ls,m_pPSM/*"LMS211"*/);
		if(m_graph.size()<=0)
			m_graph.insert(make_pair(0,pcurNode));
		else
		{
			CFliterNode* prefNode = m_graph[m_graph.size()-1];
			try{
				if(!pcurNode->matchNodeFrontendGT(prefNode,match_status)){
					std::cout<<"FMatch failed!"<<std::endl;
					delete pcurNode;
					continue;
				}
				// pcurNode->m_pose = prefNode->m_pose.oplus(pcurNode->m_relpose);
				n_of_gt++;
				switch(match_status){
				case 0:
					n_of_gt--;
					break;
				case 1:
					n_of_angle++;
					break;
				case 2:
					n_of_err++;
					break;
				default:
					cout<<"error status!"<<endl;
					break;
				}	
				m_graph.insert(make_pair(m_graph.size(),pcurNode));
			}catch(int err){
				std::cout<<"FMatch failed!"<<std::endl;
				delete pcurNode;
			}
		}
	}
	
	// record all the pose of these nodes
	ofstream outf(outfile.c_str());
	if(!outf.is_open()){
		cout<<"failed to open file: "<<outfile<<endl;
		return ;
	}
	cout<<"the number of nodes in pose-graph is: "<<m_graph.size()<<endl; 
	cout<<"number of using gt: "<<n_of_gt<<endl;
	cout<<"number for big angle: "<<n_of_angle<<endl;
	cout<<"number for big err: "<<n_of_err<<endl;
	map<int, CFliterNode*>::iterator it = m_graph.begin();
	while(it!=m_graph.end()){
		outf<<it->second->m_pose.x<<" "<<it->second->m_pose.y<<" "<<it->second->m_pose.theta<<endl;
		it++;
	}
	outf.close();
	return ;
}

bool CMapGraph::constructPoseGraphGT(){
	if(m_pPSM->m_SickScans.size()<=0){
		std::cout<<"No input data!"<<std::endl;
		return false;
	}
	int cnt=0;
	PMScan* ls;
	int match_status=0;
	while(cnt<m_pPSM->m_SickScans.size())
	{
		ls = m_pPSM->m_SickScans[cnt];
		cnt++;
		
		CFliterNode* pcurNode = new CFliterNode(ls,m_pPSM/*"LMS211"*/);
		if(m_graph.size()<=0)
			m_graph.insert(make_pair(0,pcurNode));
		else
		{
			CFliterNode* prefNode = m_graph[m_graph.size()-1];
			try{
				if(!pcurNode->matchNodeFrontendGT(prefNode,match_status)){
					std::cout<<"FMatch failed!"<<std::endl;
					delete pcurNode;
					continue;
				}
				// pcurNode->m_pose = prefNode->m_pose.oplus(pcurNode->m_relpose);
				m_graph.insert(make_pair(m_graph.size(),pcurNode));
			}catch(int err){
				std::cout<<"FMatch failed!"<<std::endl;
				delete pcurNode;
			}
		}
	}
	return true;
}


// 1 employ PSM to calculate pose for each node
// 2 insert all these nodes into m_graph
bool CMapGraph::constructPoseGraph(){
	if(m_pPSM->m_SickScans.size()<=0){
		std::cout<<"No input data!"<<std::endl;
		return false;
	}
	int cnt=0;
	PMScan* ls;
	while(cnt<m_pPSM->m_SickScans.size() && cnt<20)
	{
		ls = m_pPSM->m_SickScans[cnt];
		cnt++;
		
		CFliterNode* pcurNode = new CFliterNode(ls,m_pPSM);
		if(m_graph.size()<=0)
			m_graph.insert(make_pair(0,pcurNode));
		else
		{
			CFliterNode* prefNode = m_graph[m_graph.size()-1];
			try{
				/*if(!pcurNode->matchNodeFrontend(prefNode)){
					std::cout<<"FMatch failed!"<<std::endl;
					delete pcurNode;
					continue;
				}*/
				// pcurNode->m_pose = prefNode->m_pose.oplus(pcurNode->m_relpose);
				m_graph.insert(make_pair(m_graph.size(),pcurNode));
				cout<<"succeed to add pose-node: "<<cnt-1<<endl;
			}catch(int err){
				std::cout<<"FMatch failed!"<<std::endl;
				delete pcurNode;
			}
		}
	}
	return true;
}

void CMapGraph::testbug(){
	// create mapNode 0 and MapNode 63
	static int duo_size = 3;
	std::map<int, CFliterNode*>::iterator it = m_graph.begin();
	std::vector<CFliterNode*> m_tmpV(CMapGraph::g_session_size);

	// 1 create mapNode 0
	for(int cnt=0;cnt<CMapGraph::g_session_size;cnt++,it++){
		m_tmpV[cnt] = it->second;
	}
	CMapNode * pfNode = new CMapNode;
	pfNode->reduceIntoMapNode(m_tmpV,duo_size);

	// 2 create mapNode 63
	it = m_graph.find(630);
	if(it == m_graph.end()){
		std::cout<<"node 630 doesnot exist!"<<std::endl;
		return ;
	}
	for(int cnt=0;cnt<CMapGraph::g_session_size;cnt++,it++){
		m_tmpV[cnt] = it->second;
	}
	CMapNode* psNode = new CMapNode;
	psNode->reduceIntoMapNode(m_tmpV,duo_size);

 	OrientedPoint2D transform;
	pair<int,double> match_result = psNode->m_pFMatch->matchFeaturePoints(pfNode->m_featurePoints,psNode->m_featurePoints,transform);	// 3 match these two mapNodes
	std::cout<<"matched feature number: "<<match_result.first<<std::endl;
	std::cout<<"matched transform: "<<transform<<std::endl;
}


// this is the number of the nodes contained in the map-node
// actually this number shoule rely the number of distinct feature points

int CMapGraph::g_session_size = 10;// ;8; //10;

bool CMapGraph::constructMapGraph(){
	if(m_graph.size()<=0){
		std::cout<<"No pose node is available!"<<std::endl;
		return false;
	}
	int cnt = 0;
	// actually this should be parameterized
	static int duo_size = 3;
	int n_mapNode = 0;
	std::map<int, CFliterNode*>::iterator it = m_graph.begin();
	std::vector<CFliterNode*> m_tmpV(CMapGraph::g_session_size);
	while(it!=m_graph.end()){
		if(cnt>=CMapGraph::g_session_size){
			CMapNode * pcurNode = new CMapNode;
			// 1 reduce into a single mapNode
			pcurNode->reduceIntoMapNode(m_tmpV,duo_size); // duo size =3
			// 2 insert this mapNode into MapGraph
			// m_mapGraph.insert(make_pair(m_mapGraph.size(),pcurNode));
			if(!addMapNode(pcurNode)){ // addMapNode(pcurNode)
				std::cout<<"false to add this MapNode!"<<std::endl;
				delete pcurNode;
			}else
				{
				std::cout<<"succeed to add MapNode: "<<++n_mapNode<<std::endl;
				// std::cout<<"It has "<<pcurNode->m_featurePoints.size()<<" features!"<<std::endl;
				}
			cnt = 0; 
		}
		m_tmpV[cnt] = it->second;
		cnt++;
		it++;
	}
	// add the last several nodes
	if(cnt>0 && cnt>duo_size){
		std::cout<<"Add the last MapNode! "<<std::endl;
		CMapNode * pcurNode = new CMapNode;	
		m_tmpV.resize(cnt);
		pcurNode->reduceIntoMapNode(m_tmpV,duo_size);
		// m_mapGraph.insert(make_pair(m_mapGraph.size(),pcurNode));
		if(!addMapNode(pcurNode)){
			std::cout<<"failed to add the last several nodes!"<<std::endl;
			delete pcurNode;
		}else
		{	std::cout<<"succeed to MapNode: "<<++n_mapNode<<std::endl;
			std::cout<<"It has "<<pcurNode->m_featurePoints.size()<<" features!"<<std::endl;}
	}
	
	// save g2o information
	optimizer_->save("g2o.log");
	std::cout<<"finish saving!"<<std::endl;


	// only for test 
	/*map<int,CMapNode*>::iterator it2 = m_mapGraph.begin();
	ofstream outfile("/mnt/hgfs/SharedFold/log/test.log");
	
	CMapNode* pcurNode, *prevNode;
	while(it2!=m_mapGraph.end()){
		pcurNode = it2->second;
		OrientedPoint2D rootNode(*(pcurNode->m_rootPose));
		if(it2 == m_mapGraph.begin()) { // first node
			outfile<<OrientedPoint2D()<<endl;
			prevNode = pcurNode;
			it2++;
			continue;
		}
		OrientedPoint2D tmp((prevNode->m_rootPose->ominus(*(pcurNode->m_rootPose))));
		outfile<<prevNode->m_rootPose->oplus(tmp)<<endl;
		prevNode = pcurNode;
		it2++;
	}
	outfile.close();
*/
	return true;
}

// Incrementally construct Map Graph using GT data
bool CMapGraph::constructMapGraphGTIncrementally(){
	if(m_graph.size()<=0){
		std::cout<<"No pose node is available!"<<std::endl;
		return false;
	}
	// actually this should be parameterized
	static int duo_size = 1;
	int n_mapNode = 0;
	int cnt=0;
	std::map<int, CFliterNode*>::iterator it = m_graph.begin();
	while(it!=m_graph.end()){
		CMapNode * pcurNode = new CMapNode;
		cnt=0;
		// 1 reduce into a single mapNode
		while(it!=m_graph.end() && !pcurNode->addPoseNodeIncrementally(it->second))
		{
			it++;
			cnt++;
		}
		pcurNode->finishReduction();
		// 2 insert this mapNode into MapGraph
		if(!addMapNodeGT(pcurNode)){ // addMapNode(pcurNode)
			std::cout<<"false to add this MapNode!"<<std::endl;
			delete pcurNode;
		}else
		{
			std::cout<<"succeed to add MapNode: "<<++n_mapNode<<" with "<<cnt<<" nodes, "<<pcurNode->m_featurePoints.size()<<" features!"<<std::endl;
		}
		if(it==m_graph.end())
			break;
		it++;
	}

	// save g2o information
	// optimizer_->save("g2o.log");
	// std::cout<<"finish saving!"<<std::endl;
	return true;
}

bool CMapGraph::constructMapGraphGT(){
	if(m_graph.size()<=0){
		std::cout<<"No pose node is available!"<<std::endl;
		return false;
	}
	int cnt = 0;
	// actually this should be parameterized
	static int duo_size = 1;
	int n_mapNode = 0;
	std::map<int, CFliterNode*>::iterator it = m_graph.begin();
	std::vector<CFliterNode*> m_tmpV(CMapGraph::g_session_size);
	while(it!=m_graph.end()){
		if(cnt>=CMapGraph::g_session_size){
			CMapNode * pcurNode = new CMapNode;
			// 1 reduce into a single mapNode
			pcurNode->reduceIntoMapNode(m_tmpV,duo_size); // duo size =3
			// 2 insert this mapNode into MapGraph
			// m_mapGraph.insert(make_pair(m_mapGraph.size(),pcurNode));
			if(!addMapNodeGT(pcurNode)){ // addMapNode(pcurNode)
				std::cout<<"false to add this MapNode!"<<std::endl;
				delete pcurNode;
			}else
				{
				std::cout<<"succeed to add MapNode: "<<++n_mapNode<<std::endl;
				// std::cout<<"It has "<<pcurNode->m_featurePoints.size()<<" features!"<<std::endl;
				}
			cnt = 0; 
		}
		m_tmpV[cnt] = it->second;
		cnt++;
		it++;
	}
	// add the last several nodes
	if(cnt>0 && cnt>duo_size){
		std::cout<<"Add the last MapNode! "<<std::endl;
		CMapNode * pcurNode = new CMapNode;	
		m_tmpV.resize(cnt);
		pcurNode->reduceIntoMapNode(m_tmpV,duo_size);
		// m_mapGraph.insert(make_pair(m_mapGraph.size(),pcurNode));
		if(!addMapNodeGT(pcurNode)){
			std::cout<<"failed to add the last several nodes!"<<std::endl;
			delete pcurNode;
		}else
		{	std::cout<<"succeed to MapNode: "<<++n_mapNode<<std::endl;
			std::cout<<"It has "<<pcurNode->m_featurePoints.size()<<" features!"<<std::endl;}
	}
	
	// save g2o information
	optimizer_->save("g2o.log");
	std::cout<<"finish saving!"<<std::endl;

	// only for test 
	/*map<int,CMapNode*>::iterator it2 = m_mapGraph.begin();
	ofstream outfile("/mnt/hgfs/SharedFold/log/test.log");
	
	CMapNode* pcurNode, *prevNode;
	while(it2!=m_mapGraph.end()){
		pcurNode = it2->second;
		OrientedPoint2D rootNode(*(pcurNode->m_rootPose));
		if(it2 == m_mapGraph.begin()) { // first node
			outfile<<OrientedPoint2D()<<endl;
			prevNode = pcurNode;
			it2++;
			continue;
		}
		OrientedPoint2D tmp((prevNode->m_rootPose->ominus(*(pcurNode->m_rootPose))));
		outfile<<prevNode->m_rootPose->oplus(tmp)<<endl;
		prevNode = pcurNode;
		it2++;
	}
	outfile.close();
*/
	return true;
}

bool CMapGraph::isAvaliableArea(CMapNode* cur_node, OrientedPoint2D& trans)
{
	// use covariance to calculate LoopyArea
	static const float beta = 25; //25;//100 ;// 300 ; // 16; //25;//1.5; 
	static const float angle_thre = M_PI/3.0;
	if(fabs(trans.theta) >= angle_thre) return false;

	float dis_x, dis_y, dis;
	
	// loop trans allow < 2.5 meters for constant cov
	if(m_bUseConstantCov){
		dis = 2.5*2.5;
	}else{
		dis_x = cur_node->m_root_Cov(0,0);
		dis_y = cur_node->m_root_Cov(1,1);
		dis = (fabs(dis_x) + fabs(dis_y))*beta;
	}

	float cur_dis = (trans.x)*(trans.x) + (trans.y)*(trans.y);
	if(dis >= cur_dis)
	{
		cout<<"dis: "<<cur_dis<<" cov_dis: "<<dis<<endl;
		return true;
	}
	else{
		// cout<<"cur_pose: "<<cur_pose<<endl;
		// cout<<"pre_pose: "<<pre_pose<<endl;
		// cout<<"dis: "<<cur_dis<<" cov: "<<dis<<endl;
	}
	return false;
}

namespace{
	void calBounding(Point2D& ld, Point2D& ru)
	{	
		float x_min = ld.x < ru.x ? ld.x : ru.x;
		float x_max = ld.x > ru.x ? ld.x : ru.x;
		float y_min = ld.y < ru.y ? ld.y : ru.y;
		float y_max = ld.y > ru.y ? ld.y : ru.y;
		ld.x = x_min; ld.y = y_min;
		ru.x = x_max; ru.y = y_max;
	}
}

bool CMapGraph::isOverlappedArea(CMapNode* cur_node, CMapNode* ref_node)
{
	Point2D ld_cur = cur_node->m_rootPose->oplus(*(cur_node->m_ld_corner));
	Point2D ru_cur = cur_node->m_rootPose->oplus(*(cur_node->m_ru_corner));
	Point2D ld_ref = ref_node->m_rootPose->oplus(*(ref_node->m_ld_corner));
	Point2D ru_ref = ref_node->m_rootPose->oplus(*(ref_node->m_ru_corner));
	calBounding(ld_cur,ru_cur);
	calBounding(ld_ref,ru_ref);
	if(ru_cur.x <= ld_cur.x || ru_cur.y <= ld_cur.y || ru_ref.x <= ld_ref.x || ru_ref.y <= ld_ref.y)
	{
		cout<<"ld_cur: "<<ld_cur<<" ru_cur: "<<ru_cur<<endl; 
		cout<<"ld_ref: "<<ld_ref<<" ru_ref: "<<ru_ref<<endl;
		cout<<"MapGraph error1 in Overlapped Area!"<<endl;
		return false;
	} 

	if(ru_cur.x <= ld_ref.x || ru_cur.y <= ld_ref.y || ru_ref.x <= ld_cur.x || ru_ref.y <= ld_cur.y)
		return false;
	// calculate overlapped area
	float x_min = ld_cur.x > ld_ref.x ? ld_cur.x : ld_ref.x;
	float y_min = ld_cur.y > ld_ref.y ? ld_cur.y : ld_ref.y;
	float x_max = ru_cur.x < ru_ref.x ? ru_cur.x : ru_ref.x;
	float y_max = ru_cur.y < ru_ref.y ? ru_cur.y : ru_ref.y;
	float overlapped = (x_max-x_min)*(y_max-y_min);
	float curArea = (ru_cur.x-ld_cur.x)*(ru_cur.y-ld_cur.y);
	float overlappedRatio = overlapped/curArea;
	if(overlapped <= 0){
		cout<<"MapGraph error2 in Overlapped Area!"<<endl;
		return false;
	}
	return (overlappedRatio >= 0.4);
}

bool CMapGraph::isLoopyArea(CMapNode* cur_node, CMapNode* pre_node, float beta){
	if(!cur_node || !pre_node){
		cout<<"node is null in isLoopyArea()!"<<endl;
		return false;
	}
	// use covariance to calculate LoopyArea
	// static const float beta = 16; // 16; //25;//1.5; 
	/*float dis_x = beta*(cur_node->m_dis_mean_x + pre_node->m_dis_mean_x);
	float dis_y = beta*(cur_node->m_dis_mean_y + pre_node->m_dis_mean_y);
	float dis = dis_x*dis_x + dis_y*dis_y;*/
	
	float dis_x,dis_y,dis;

	
	// only search for 2 meters for constant variance
	if(m_bUseConstantCov){
		dis = 2*2;
	}else{
		dis_x = cur_node->m_root_Cov(0,0);
		dis_y = cur_node->m_root_Cov(1,1);
		dis = (fabs(dis_x) + fabs(dis_y))*beta;
	}

	OrientedPoint2D& cur_pose = *(cur_node->m_rootPose);
	OrientedPoint2D& pre_pose = *(pre_node->m_rootPose);
	float cur_dis = (cur_pose.x-pre_pose.x)*(cur_pose.x-pre_pose.x) + \
			(cur_pose.y-pre_pose.y)*(cur_pose.y-pre_pose.y);
	if(dis >= cur_dis)
	{
		return true;
	}
	else{
		// cout<<"cur_pose: "<<cur_pose<<endl;
		// cout<<"pre_pose: "<<pre_pose<<endl;
		// cout<<"dis: "<<cur_dis<<" cov: "<<dis<<endl;
	}
	return false;
}

// add node which contains the ground truth data
bool CMapGraph::addMapNodeGT(CMapNode* new_node)
{
	cout<<"this MapNode "<<new_node->m_root_id<<" contains: "<<new_node->m_featurePoints.size()<<" features!"<<endl;
	// this should be parameterized
	static int least_features = 150; 
	bool bfeatureless = false;

	if(new_node->m_featurePoints.size()<least_features){
		// std::cout<<"This map-node contains little features!"<<std::endl;
		bfeatureless = true; // does not have enough features
	}
	
	new_node->m_id = m_mapGraph.size();
	//First Node, so only build its index, insert into storage and add a
	//vertex at the origin, of which the position is very certain
	if (m_mapGraph.size()==0){
		m_mapGraph[new_node->m_id] = new_node;
		OrientedPoint2D rpose=*(new_node->m_rootPose);
		g2o::VertexSE2* reference_pose = new g2o::VertexSE2;
		reference_pose->setId(0);
		reference_pose->setEstimate(g2o::SE2(rpose.x,rpose.y,rpose.theta));
		reference_pose->setFixed(true);//fix at origin
		//optimizer_mutex.lock();
		optimizer_->addVertex(reference_pose);
		//optimizer_mutex.unlock();
		//current_poses_.append(latest_transform_);
		return true;
	}
	 unsigned int num_edges_before = optimizer_->edges().size();

	//MAIN LOOP: Compare node pairs ######################################################################
	// 1, Firstly add this map-node into map-graph 
	CMapNode* prev_frame = m_mapGraph[m_mapGraph.size()-1];
	MatchingResult mr;
	OrientedPoint2D tmp((prev_frame->m_rootPose->ominus(*(new_node->m_rootPose))));
	((CFliterNode*)0)->fromOrientedPoint2SE2(tmp,mr.m_edge);
//	((CFliterNode*)0)->fromOrientedPoint2SE2(prev_frame->m_rootPose->ominus(*(new_node->m_rootPose)),mr.m_edge);
	mr.id1 = prev_frame->m_id;
	mr.id2 = new_node->m_id;
	Eigen::Matrix3d information;
	/*information.fill(0.);
	for(int i=0;i<3;i++)
		information(i,i) = least_features;*/
	// USE information of current pose as edge's information
	
	// Is this correct?  
	Eigen::Matrix3d covariance = new_node->m_covariance - prev_frame->m_covariance;
	information = covariance.inverse(); // 
	mr.m_edge.setInformation(information);
	if(!addEdgeToG2O(mr,true,true)){
		std::cout<<"Failed to add Edge: "<<mr.id1<<","<<mr.id2<<std::endl;
		return false;
	}
	
	// 2, Detect loop 
	bool bLoop = false;
	static int adj_win_size = 3; // the adjacent number of mapnodes
	if(1 && !bfeatureless){	// for featureless mapnode, do not detect loop
		if(m_mapGraph.size()>adj_win_size){
			std::map<int, CMapNode*>::iterator it = m_mapGraph.begin();
			while(it!=m_mapGraph.end()){
				// not compare with the adjacent node set
				if(it->first == m_mapGraph.size()-adj_win_size)
					break;
				MatchingResult mr2;
				// TODO: use variance of current pose to filter mismatch
				if(!isLoopyArea(new_node,it->second, 25)){
					it++;
					continue;
				}
				//new_node->matchNodePair(it->second,mr2);
				new_node->matchNodePairGT(it->second,mr2);
				if(mr2.id1>=0){
					std::cout<<"Loop found between: "<<mr2.id1<<","<<mr2.id2<<std::endl;

					// addEdgeToG2O(mr, large_edge, set_estimate)
					addEdgeToG2O(mr2,true,true/*false true*/);
					OrientedPoint2D trans;
					((CFliterNode*)0)->fromSE22OrientedPoint(trans,mr2.m_edge);
					// std::cout<<"Trnas: "<<trans<<std::endl;
					bLoop = true;
				}
				it++;
			}
		}
	}
	static int cnt_op = 0;
	
	// 3, graph optimization
	if (optimizer_->edges().size() > num_edges_before) { //Success
		m_mapGraph[new_node->m_id] = new_node;
		if(bLoop || ++cnt_op>20){
			optimizeGraph(10); // not optimize
			cnt_op = 0;
		}
		if(bLoop){
			synPoseAfterOpt(); // syn pose value after optimization
		}
	}
	return optimizer_->edges().size() > num_edges_before;
}

// if trans's dis is larger than 1 m
// or theta is larger than 45'
bool CMapGraph::isLargeEdge(OrientedPoint2D& p){
	if((p.x*p.x+p.y*p.y) >= 9) return true;
	// if(p.theta >= (M_PI/4)) return true;
	return false;
}

bool CMapGraph::isSmallEdge(OrientedPoint2D& p){
	// if((p.x*p.x + p.y*p.y) > 0.25) return false;
	// if(fabs(p.theta) > 0.02 )  return false;
	if((p.x*p.x + p.y*p.y) > 0.0025) return false;
	if(fabs(p.theta) > 0.01 )  return false;
	return true;
}

bool CMapGraph::constructDelayedG2o(){
	if(!b_delayed_g2o) return true;
	b_delayed_g2o = false;

	// 1, construct adjacent nodes
	/*MatchingResult mr;
	map<int,CMapNode*>::iterator it_curr = m_mapGraph.begin();
	map<int,CMapNode*>::iterator it_prev = it_curr;
	it_curr++;
	CMapNode* prev_frame;
	CMapNode* new_node;
	while(it_curr!=m_mapGraph.end()){
		prev_frame = it_prev->second;
		new_node = it_curr->second;
		OrientedPoint2D tmp((prev_frame->m_rootPoseBack->ominus(*(new_node->m_rootPoseBack))));
		((CFliterNode*)0)->fromOrientedPoint2SE2(tmp,mr.m_edge);
		mr.id1 = prev_frame->m_id;
		mr.id2 = new_node->m_id;
		Eigen::Matrix3d information;
		information.fill(0.); // this is very important
		// TO estimate information matrix
		// 1 use the number of features
		//for(int i=0;i<3;i++)
		//	information(i,i) = least_features;//least_features;
		// 2 use icp covariance 
		information = prev_frame->m_information;
		mr.m_edge.setInformation(information);
		if(!addEdgeToG2O(mr,true,true)){
			std::cout<<"Failed to add Edge: "<<mr.id1<<","<<mr.id2<<std::endl;
			return false;
		}
		it_curr++;
		it_prev++;
	}*/
	
	// 1, construct covariance according to feature distribution
	// calCovAdded();
	
	if(m_loopEdges.size()!= m_bLargeEdges.size()){
		cout<<"MapGraph: error in delayedG2O()!"<<endl;
		return false;
	}else{
		cout<<"MapGraph: size is equal!"<<endl;
	}

	// 2, construct loop edges
	for(int i=0;i<m_loopEdges.size();i++){
		MatchingResult* pEdge = m_loopEdges[i];
		if(!addEdgeToG2O(*pEdge,true,/*true*/ m_bLargeEdges[i])){
			std::cout<<"Failed to add Edge: "<<pEdge->id1<<","<<pEdge->id2<<endl;
			return false;
		}
	}
	cout<<"finish constructing delayed g2o structure!"<<endl;
	return true;
}

bool CMapGraph::calCovAdded()
{
	vector<int> fts;
	calFeatureDistribute(fts);
	Eigen::Matrix3d inf;
	Eigen::Matrix3d cov;
	for(int i=0;i<m_loopEdges.size();i++){
		MatchingResult* pMth = m_loopEdges[i];
		if(pMth->id1 + 1 != pMth->id2) continue;
		inf = pMth->m_edge.information();
		cov = inf.inverse();
		((CMapNode*)0)->featurePDF(fts[pMth->id1],m_f_mean,m_f_sigma,cov);
		pMth->m_edge.setInformation(cov.inverse());
	}
}

bool CMapGraph::calFeatureDistribute(vector<int>& fts)
{
	map<int,CMapNode*>::iterator it = m_mapGraph.begin();
	fts.resize(m_mapGraph.size());
	while(it!=m_mapGraph.end())
	{
		// fts[it->first] = it->second->m_featurePoints.size();
		fts[it->first] = it->second->fvector.size();
		it++;
	}
	vector<int> tmp(fts.size());
	int sum = 0;
	double square_sum = 0;
	tmp[0] = fts[0];
	tmp[tmp.size()-1] = fts[fts.size()-1];
	for(int i=1;i<fts.size()-1;i++)
	{
		tmp[i] = (fts[i-1]+ fts[i] + fts[i+1])/3;
		sum += tmp[i];
	}
	sum = sum + tmp[0] + tmp[tmp.size()-1];
	m_f_mean = (sum)/tmp.size();
	for(int i=0;i<tmp.size();i++)
	{
		square_sum += (tmp[i]-m_f_mean)*(tmp[i]-m_f_mean);
	}
	m_f_sigma = (square_sum)/tmp.size();
	m_f_sigma = sqrt(m_f_sigma);
	fts.swap(tmp);

	cout<<"MapGraph: feature distribution: mean: "<<m_f_mean<<" sigma: "<<m_f_sigma<<endl;
	return true;
}


bool CMapGraph::isBigAngleDiff(OrientedPoint2D* p1, OrientedPoint2D* p2)
{
	// static ofstream angle("angle_diff.log");
	float angDiff = p1->theta - p2->theta;
	if(angDiff < 0 ) angDiff*=-1;
	angDiff = normalAng(angDiff, 0);
	// angle<<angDiff<<" "<<p1->theta<<" "<<p2->theta<<endl;
	// cout<<"angle_diff: "<<angDiff<<" p1: "<<p1->theta<<" p2: "<<p2->theta<<endl;
	return (angDiff >= M_PI/2); 
}

bool CMapGraph::addMapNodeCov(CMapNode* new_node, double cov[], bool& bloop)
{
	// this should be parameterized
	int least_features = 20; 
	b_delayed_g2o = true;
	bool featureless = false;
	if(new_node->m_featurePoints.size()<least_features){
		// std::cout<<"This map-node contains little features!"<<std::endl;
		featureless = true;
	}
	
	new_node->m_id = m_mapGraph.size();
	//First Node, so only build its index, insert into storage and add a
	//vertex at the origin, of which the position is very certain
	if (m_mapGraph.size()==0){
		m_mapGraph[new_node->m_id] = new_node;
		// m_before_opt[new_node->m_id] = new OrientedPoint2D(*(new_node->m_rootPose));
		g2o::VertexSE2* reference_pose = new g2o::VertexSE2;
		OrientedPoint2D rpose = *(new_node->m_rootPose);
		reference_pose->setId(0);
		reference_pose->setEstimate(g2o::SE2(rpose.x,rpose.y,rpose.theta));
		reference_pose->setFixed(true);//fix at origin
		optimizer_->addVertex(reference_pose);
		return true;
	}

	 unsigned int num_edges_before = /* m_mapGraph.size();*/ optimizer_->edges().size();

	//MAIN LOOP: Compare node pairs ######################################################################
	// 1, Firstly add this map-node into map-graph 
	CMapNode* prev_frame = m_mapGraph[m_mapGraph.size()-1];
	MatchingResult* mr = new MatchingResult;
	OrientedPoint2D tmp((prev_frame->m_rootPose->ominus(*(new_node->m_rootPose))));
	((CFliterNode*)0)->fromOrientedPoint2SE2(tmp,mr->m_edge);

	mr->id1 = prev_frame->m_id;
	mr->id2 = new_node->m_id;
	Eigen::Matrix3d information;
	information.fill(0.); // this is very important
	if(m_bUseConstantCov)
	{	
		/*if(prev_frame->m_covariance(0,0) > 0.5 && \
			prev_frame->m_covariance(1,1) > 0.5)
			information = m_cov_corridor;
		else*/
		information = m_cov_adjnode; 
	}
	else
		information = prev_frame->m_information;
	mr->m_edge.setInformation(information);
	// m_loopEdges.push_back(mr);
	// m_bLargeEdges.push_back(true);
	if(!addEdgeToG2O(*mr,true,true)){
		std::cout<<"Failed to add Edge: "<<mr->id1<<","<<mr->id2<<std::endl;
		return false;
	}

	// 2, Detect loop 
	int max_matched_num = -1;
	bool bLoop = false;
	static int adj_win_size = 40; //100 // 5; // the adjacent number of mapnodes
	CMPSet matchSet;
	int matched_num;

	// for ICP quality
	int n_iter;
	float goodness;
	float quality;

	bool bigAngle; 
	if(m_mapGraph.size()>adj_win_size){
		std::map<int, CMapNode*>::iterator it = m_mapGraph.begin();
		while(it!=m_mapGraph.end()){
			// not compare with the adjacent node set
			if(it->first >= m_mapGraph.size()-adj_win_size)
				break;
			// if(!isLoopyArea(new_node,it->second)) // use marginal covariance 
			// 1 check overlapped observation and covariance
			if(!isOverlappedArea(new_node, it->second) || 
				!isLoopyArea(new_node, it->second, 225))
			{
				it++;
				continue;
			}
			bigAngle = false;
			goodness = 1;
			MatchingResult* mr2 = new MatchingResult;
			OrientedPoint2D trans;
			// 2 do not match features, for big angle diff
			/*if(isBigAngleDiff(new_node->m_rootPose,it->second->m_rootPose)) 
			{
				trans = it->second->m_rootPose->ominus(*(new_node->m_rootPose));
				bigAngle = true;
			}else{ // first match features then ICP
				new_node->matchNodePair(it->second,*mr2);
				if(mr2->id1 < 0){
					it++;
					continue;
				}
				((CFliterNode*)0)->fromSE22OrientedPoint(trans,mr2->m_edge);
			}*/
			/* For only icp matching */
			// trans = it->second->m_rootPose->ominus(*(new_node->m_rootPose));

			trans = it->second->m_rootPose->ominus(*(new_node->m_rootPose));

			mr2->id1 = it->second->m_id;
			mr2->id2 = new_node->m_id;
			// OrientedPoint2D featureTrans = trans;
			// 3 ICP 
			new_node->matchNodePairICP(it->second,*mr2,&trans);
			new_node->getICPResult(n_iter,goodness,quality);

			if((!bigAngle && goodness < ICP_QUALITY_THRESHOLD) || \
				(goodness < ICP_QUALITY_REVERSE_THRESHOLD)){
				#ifdef SAVE_PMAP
				// to dump this MapNode
				if(goodness > 0.4){
					static set<int> idx;
					// OrientedPoint2D pt1 = it->second->m_rootPose->oplus(trans);
					// OrientedPoint2D relt = new_node->m_rootPose->ominus(pt1);
					m_record_matched<<mr2->id1<<" "<<mr2->id2<<" "<<matched_num<<" ";
					m_record_matched<<trans.x<<" "<<trans.y<<" "<<trans.theta<<endl;
					if(idx.find(it->second->m_id) == idx.end())
					{
						it->second->dumpToFile();
						idx.insert(it->second->m_id);
					}
					if(idx.find(new_node->m_id) == idx.end())
					{
						new_node->dumpToFile();
						idx.insert(it->second->m_id);
					}
				}
				#endif
				it++;
				continue;
			}

			((CFliterNode*)0)->fromSE22OrientedPoint(trans,mr2->m_edge);
 			OrientedPoint2D newPose = it->second->m_rootPose->oplus(trans);

			/* For feature based matching */		
 			// ((CFliterNode*)0)->fromSE22OrientedPoint(featureTrans,mr2->m_edge);
			// OrientedPoint2D newPose = it->second->m_rootPose->oplus(featureTrans);

			// delete matches whose trans is beyond covariance of new_node
			OrientedPoint2D curr_trans = new_node->m_rootPose->ominus(newPose);
			if(!isAvaliableArea(new_node,curr_trans)){
				cout<<"Good: "<<goodness<<" Matched between: "<<mr2->id1<<","<<mr2->id2<<endl;
				cout<<"But: trans: "<<curr_trans<<" while cov: "<<new_node->m_root_Cov(0,0)<<" "<<new_node->m_root_Cov(1,1)<<" "<<new_node->m_root_Cov(2,2)<<endl;
				it++;
				continue;
			}
			std::cout<<"Good: "<<goodness<<" Loop found between: "<<mr2->id1<<","<<mr2->id2<<std::endl;
			if(m_bUseConstantCov)
				mr2->m_edge.setInformation(m_cov_loopnode);
				// mr2->m_edge.setInformation(m_cov_adjnode);
			matchSet.push_back(CMatchedPair(it->first,goodness,newPose,mr2));
			it++;
		}
	}

	int last_index = -1;
	OrientedPoint2D final_pose;
	if(matchSet.size()>0){
		// delete mismatches according to relative dis
		deleteMisMatch(matchSet);
		// calculate the current robot pose
		last_index = calculateFinalPose(matchSet,final_pose,cov);
	}
	if(last_index >= 0){
		/*OrientedPoint2D curr_trans;
		for(int i=0;i<matchSet.size();i++){
			curr_trans = new_node->m_rootPose->ominus(*(matchSet[i].m_pose));
			bool bSmall = isSmallEdge(curr_trans);
			if(0 && !bSmall){
				bloop = bLoop = bSmall;
				*(new_node->m_rootPose) = *(matchSet[i].m_pose);
				addEdgeToG2O(*(matchSet[last_index].m_mr), true, true);
			}
			else{
				addEdgeToG2O(*(matchSet[last_index].m_mr), true, false);
			}
		}*/
		for(int i=0;i<matchSet.size();i++)
		{
			if(i == last_index) continue;
			// m_loopEdges.push_back(matchSet[i].m_mr);
			// m_bLargeEdges.push_back(false);
			addEdgeToG2O(*(matchSet[i].m_mr),true,false);
		}
		OrientedPoint2D curr_trans = new_node->m_rootPose->ominus(final_pose);
		bool bSmall = isSmallEdge(curr_trans);
		if(!bSmall){
			// m_bLargeEdges.push_back(true);
			bloop = bLoop = true;
			*(new_node->m_rootPose) = final_pose;
			// *(new_node->m_rootPoseBack) = final_pose;
		}
			// m_bLargeEdges.push_back(false);
		// m_loopEdges.push_back(matchSet[last_index].m_mr);
		addEdgeToG2O(*(matchSet[last_index].m_mr), true, !bSmall);
	}
	if(bLoop){ //  loop is found, optimization and update
		optimizeGraph(20); // optimization
		synPoseAfterOpt(); // update robot pose
	}

	m_mapGraph[new_node->m_id] = new_node;
	return optimizer_->edges().size() > num_edges_before;
	// return m_mapGraph.size() > num_edges_before;
}
bool CMapGraph::addMapNode(CMapNode* new_node)
{
	// this should be parameterized
	static int least_features = 20; 
	bool featureless = false;
	if(new_node->m_featurePoints.size()<least_features){
		std::cout<<"This map-node contains little features!"<<std::endl;
		featureless = true;
		// return false;
	}
	
	new_node->m_id = m_mapGraph.size();
	//First Node, so only build its index, insert into storage and add a
	//vertex at the origin, of which the position is very certain
	if (m_mapGraph.size()==0){
		m_mapGraph[new_node->m_id] = new_node;
		// m_before_opt[new_node->m_id] = new OrientedPoint2D(*(new_node->m_rootPose));
		g2o::VertexSE2* reference_pose = new g2o::VertexSE2;
		OrientedPoint2D rpose = *(new_node->m_rootPose);
		reference_pose->setId(0);
		reference_pose->setEstimate(g2o::SE2(rpose.x,rpose.y,rpose.theta));
		reference_pose->setFixed(true);//fix at origin
		//optimizer_mutex.lock();
		optimizer_->addVertex(reference_pose);
		//optimizer_mutex.unlock();
		//current_poses_.append(latest_transform_);
		return true;
	}

	 unsigned int num_edges_before = optimizer_->edges().size();

	//MAIN LOOP: Compare node pairs ######################################################################
	// 1, Firstly add this map-node into map-graph 
	CMapNode* prev_frame = m_mapGraph[m_mapGraph.size()-1];
	MatchingResult mr;
	OrientedPoint2D tmp((prev_frame->m_rootPose->ominus(*(new_node->m_rootPose))));
	((CFliterNode*)0)->fromOrientedPoint2SE2(tmp,mr.m_edge);
//	((CFliterNode*)0)->fromOrientedPoint2SE2(prev_frame->m_rootPose->ominus(*(new_node->m_rootPose)),mr.m_edge);
	mr.id1 = prev_frame->m_id;
	mr.id2 = new_node->m_id;
	Eigen::Matrix3d information;
	information.fill(0.); // this is very important
	// TO estimate information matrix
	// 1 use the number of features
	//for(int i=0;i<3;i++)
	//	information(i,i) = least_features;//least_features;
	// 2 use icp covariance 
	information = prev_frame->m_information;
	mr.m_edge.setInformation(information);
	if(!addEdgeToG2O(mr,true,true)){
		std::cout<<"Failed to add Edge: "<<mr.id1<<","<<mr.id2<<std::endl;
		return false;
	}
	
	// record this trajectory before optimization
	// m_before_opt[new_node->m_id] = new OrientedPoint2D(*(new_node->m_rootPose));

	// 2, Detect loop 
	bool bLoop = false;
	static int adj_win_size = 20; // 5; // the adjacent number of mapnodes
	if(m_mapGraph.size()>adj_win_size && !featureless){
		std::map<int, CMapNode*>::iterator it = m_mapGraph.begin();
		while(it!=m_mapGraph.end()){
			// not compare with the adjacent node set
			if(it->first >= m_mapGraph.size()-adj_win_size)
				break;
			MatchingResult mr2;
			int matched_num = new_node->matchNodePair(it->second,mr2);
			if(mr2.id1>=0){
				std::cout<<"Loop found between: "<<mr2.id1<<","<<mr2.id2<<std::endl;
				OrientedPoint2D trans;
				((CFliterNode*)0)->fromSE22OrientedPoint(trans,mr2.m_edge);
				// std::cout<<"before icp: trans: "<<trans<<std::endl;
				// Use ICP to refine the final trans
				new_node->matchNodePairICP(it->second,mr2,&trans);
				((CFliterNode*)0)->fromSE22OrientedPoint(trans,mr2.m_edge);
			
				// compute its trans from current pose
				OrientedPoint2D pt1 = it->second->m_rootPose->oplus(trans);
				OrientedPoint2D relt = new_node->m_rootPose->ominus(pt1);
				
				// only set large edge
				bool largeEdge = isLargeEdge(relt);
				addEdgeToG2O(mr2,true,!largeEdge/*isLargeEdge(relt)*/);
				if(!largeEdge)
				{
					*(new_node->m_rootPose) = pt1;
					cout<<"Small edge: "<<relt<<endl;
				}else{
					cout<<"Large edge: "<<relt<<endl;
				}
				// std::cout<<"after icp trnas: "<<trans<<std::endl;
				bLoop = true;
				
				//TODO: send the ready covariance back to UKF
				// this may cause error !
				// new_node->m_information = mr2.m_edge.information();
				// new_node->m_covariance = new_node->m_information.inverse();
				
				break;
				#ifdef SAVE_PMAP
					static set<int> idx;
					// OrientedPoint2D pt1 = it->second->m_rootPose->oplus(trans);
					// OrientedPoint2D relt = new_node->m_rootPose->ominus(pt1);
					m_record_matched<<mr2.id1<<" "<<mr2.id2<<" "<<matched_num<<" ";
					m_record_matched<<trans.x<<" "<<trans.y<<" "<<trans.theta<<endl;
					if(idx.find(it->second->m_id) == idx.end())
					{
						it->second->dumpToFile();
						idx.insert(it->second->m_id);
					}
					if(idx.find(new_node->m_id) == idx.end())
					{
						new_node->dumpToFile();
						idx.insert(new_node->m_id);
					}
				#endif
				}
			it++;
		}
	}
	static int cnt_op = 0;
	
	// 3, graph optimization
	if (optimizer_->edges().size() > num_edges_before) { //Success
		float cur_err = optimizer_->activeChi2();
		// cout<<"after add: "<<new_node->m_id<<" chi2: "<<cur_err<<endl;
		m_mapGraph[new_node->m_id] = new_node;
		if(bLoop || ++cnt_op>20){
			// optimizeGraph(10);
			cnt_op = 0;
		}
	}
	return optimizer_->edges().size() > num_edges_before;
}

// read log from ground truth
bool CMapGraph::readlog(string logfile){
	std::ifstream infile(logfile.c_str());
	if(!infile.is_open()){
		cout<<"failed to open file: "<<logfile<<endl;
		return false;
	}
	CarmenLogReader reader;
	reader.readLog(infile,m_log);
	return (m_log.size()>0);
}


void CMapGraph::dumpG2oTrajectory(std::string filename){
	std::ofstream outf(filename.c_str());
	if(!outf.is_open()){
		std::cout<<"failed to open file: "<<filename<<std::endl;
		return ;
	}
	for(int i=0;i<optimizer_->vertices().size();i++){
		g2o::VertexSE2* pVertex=dynamic_cast<g2o::VertexSE2*>(optimizer_->vertex(i));
		//Eigen::Vector3d ptrans = pVertex->estimate().toVector();
		double translation[3];
		pVertex->getEstimateData(translation);
		outf<<translation[0]<<" "<<translation[1]<<" "<<translation[2]<<std::endl;
	}
	return ;
}

void CMapGraph::runG2olog()
{
	if(m_log.size()<=0){
		cout<<"log is empty!"<<endl;
		return;
	}
	
	cout<<"Log Size:	"<<m_log.size()<<endl;

	for(int i=0;i<100/*m_log.size()*/;i++){
		cout<<"	Step:	"<<i<<endl;
		LaserReading* pcurRead = dynamic_cast<LaserReading*> (m_log[i]);
		// 1, create Flirter Node 
		CFliterNode * pcurNode = new CFliterNode(pcurRead);
		if(pcurNode==NULL || pcurNode->m_featurePoints.size()<=0){
			cout<<"failed to create node at frame: "<<i+1<<endl;
			continue;
		}
		// 2, add this node into graph
		if(addNode(pcurNode)){
			std::cout<<"successfully add node : "<<i<<std::endl;
		}else{
			std::cout<<"failed to add node :"<<i<<std::endl;
			delete pcurRead;
		}
	}	
	return ;
}

bool CMapGraph::runlog(){
	if(m_log.size()<=0){
		cout<<"log is empty!"<<endl;
		return false;
	}
	
	cout<<"Log Size:	"<<m_log.size()<<endl;

	for(int i=0;i<100/*i<m_log.size()*/;i++){
		cout<<"	Step:	"<<i<<endl;
		LaserReading* pcurRead = dynamic_cast<LaserReading*> (m_log[i]);
		// create Flirter Node 
		CFliterNode * pcurNode = new CFliterNode(pcurRead);
		if(pcurNode==NULL || pcurNode->m_featurePoints.size()<=0){
			cout<<"failed to create node at frame: "<<i+1<<endl;
			continue;
		}
		// this is first frame
		if(m_graph.size()==0){
			//pcurNode->m_pose=OrientedPoint2D(0,0,0);
			pcurNode->m_relpose = pcurNode->m_pose;
		}
		else{
			CFliterNode * prefNode = (*m_graph.rbegin()).second;
			OrientedPoint2D transform;
			//if(!pcurNode->matchNodePairGlobal(prefNode,transform))
			pair<int,double> ret = pcurNode->matchNodePairLocal(prefNode,transform);
			if(ret.first <=0 || ret.second < 0)
			{
				cout<<"failed to match "<<i+1<<" with "<<i<<endl;
				continue;
			}
		/*	if(IsNoisyMove(transform)){
				cout<<"Noisy move! "<<i+1<<" to "<<i<<endl;
				continue;
			}*/
		}
		m_graph.insert(make_pair(m_graph.size(),pcurNode));
	}
	return true;
}

void CMapGraph::recordPoseTraj(string outfile){
	ofstream outf(outfile.c_str());
	for(int i=0;i<m_graph.size();i++){
		OrientedPoint2D pose = m_graph[i]->m_pose;
		outf<<pose.x<<" "<<pose.y<<" "<<pose.theta<<endl;
	}
	return ;
}
/*
void CMapGraph::recordFuseMap(string outfile){
	ofstream trajectory(outfile.c_str());
	if(trajectory.is_open()){
		map<int, OrientedPoint2D*>::iterator it = m_before_opt.begin();
		map<int, CMapNode*>::iterator it_map;
		while(it!=m_before_opt.end()){
			it_map = m_mapGraph.find(it->first);
			if(it_map == m_mapGraph.end()){
				cout<<"fault in FuseMap!"<<endl;
				return ;
			}
			// calculate root pose for this map-node
			OrientedPoint2D rootpose = *(it->second);
			// trajectory<<rootpose<<std::endl;
			// record relative trajectory in each map-node
			OrientedPoint2D lastpose(rootpose);
			OrientedPoint2D curpose;
			for(int i=0;i<it_map->second->m_trajectory.size();i++){
				// curpose = lastpose.oplus(*(it->second->m_trajectory[i]));
				curpose = lastpose + *(it_map->second->m_abs_trajectory[i]);
				trajectory<<(i==0?"1":"0")<<" "<<curpose.x<<" "<<curpose.y<<" "<<curpose.theta<<" "<<it_map->second->m_root_id<<std::endl;
				// lastpose = curpose;
				vector<float>& b = it_map->second->m_bearing[i];
				trajectory<<b.size()<<" ";
				for(int j=0;j<b.size();j++){
					trajectory<<b[j]<<" ";
				}
				trajectory<<endl;
			}
			it++;
		}//while
	}
	else{
		std::cout<<"failed to open file: "<<outfile<<std::endl;
		return ;
	}
	trajectory.close();
	return ;
}*/
void CMapGraph::recordMap(string outfile)
{
	ofstream trajectory(outfile.c_str());
	if(trajectory.is_open()){
		map<int, CMapNode*>::iterator it = m_mapGraph.begin();
		while(it!=m_mapGraph.end()){
			// calculate root pose for this map-node
			g2o::VertexSE2* pVertex=dynamic_cast<g2o::VertexSE2*>(optimizer_->vertex(it->first));
			double translation[3];
			if(pVertex==NULL){
				std::cout<<"pVertex is NULL!"<<std::endl;
				break;
			}
			pVertex->getEstimateData(translation);
			OrientedPoint2D rootpose(translation[0],translation[1],translation[2]);
			// trajectory<<rootpose<<std::endl;
			// record relative trajectory in each map-node
			OrientedPoint2D lastpose(rootpose);
			OrientedPoint2D curpose;
			for(int i=0;i<it->second->m_trajectory.size();i++){
				curpose = lastpose.oplus(*(it->second->m_trajectory[i]));
				// curpose = lastpose + *(it->second->m_abs_trajectory[i]);
				trajectory<<(i==0?"1":"0")<<" "<<curpose.x<<" "<<curpose.y<<" "<<curpose.theta<<" "<<it->second->m_root_id<<std::endl;
				// lastpose = curpose;
				vector<float>& b = it->second->m_bearing[i];
				trajectory<<b.size()<<" ";
				for(int j=0;j<b.size();j++){
					trajectory<<b[j]<<" ";
				}
				trajectory<<endl;
			}
			it++;
		}//while
	}
	else{
		std::cout<<"failed to open file: "<<outfile<<std::endl;
		return ;
	}
	trajectory.close();
	return ;

}


// record robot trajectory after graph optimization in mapGraph
void CMapGraph::recordTraj(string outfile){
	ofstream trajectory(outfile.c_str());
	if(trajectory.is_open()){
		map<int, CMapNode*>::iterator it = m_mapGraph.begin();
		while(it!=m_mapGraph.end()){
			// calculate root pose for this map-node
			g2o::VertexSE2* pVertex=dynamic_cast<g2o::VertexSE2*>(optimizer_->vertex(it->first));
			double translation[3];
			if(pVertex==NULL){
				std::cout<<"pVertex is NULL!"<<std::endl;
				break;
			}
			pVertex->getEstimateData(translation);
			OrientedPoint2D rootpose(translation[0],translation[1],translation[2]);
			// trajectory<<rootpose<<std::endl;
			// record relative trajectory in each map-node
			OrientedPoint2D lastpose(rootpose);
			OrientedPoint2D curpose;
			for(int i=0;i<it->second->m_trajectory.size();i++){
				curpose = lastpose.oplus(*(it->second->m_trajectory[i]));
				// curpose = lastpose + *(it->second->m_abs_trajectory[i]);
				trajectory<<curpose.x<<" "<<curpose.y<<" "<<curpose.theta<<std::endl;
				// lastpose = curpose;
			}
			it++;
		}//while
	}
	else{
		std::cout<<"failed to open file: "<<outfile<<std::endl;
		return ;
	}
	trajectory.close();
	return ;
}

void CMapGraph::recordG2O(){
	// for debug g2o 
	optimizer_->save("g2o.log");
}

void CMapGraph::recordTrajectory(string outfile)
{
	ofstream trajectory(outfile.c_str());
	if(trajectory.is_open()){
		map<int, CFliterNode*>::iterator it = m_graph.begin();
		bool firstPose=true;
		OrientedPoint2D lastpose,curpose,original;
		while(it!=m_graph.end()){
			if(firstPose){
				original = it->second->m_pose;
				curpose = it->second->m_pose;
				firstPose = false;
			}
			else
				//curpose = lastpose + (it->second->m_relpose); //for global
			curpose = lastpose.oplus(it->second->m_relpose); //			for local
			// record also relative pose
			trajectory<<original.ominus(it->second->m_pose)<<" "<<curpose<<endl;
			//trajectory<<curpose<<endl;
			//lastpose=curpose;
			lastpose=original.ominus(it->second->m_pose);
			it++;		
		}
		trajectory.close();
	}
}


bool CMapGraph::IsNoisyMove(OrientedPoint2D& transform)
{
	if(fabs(transform.x) >= DIS_THRESHOLD || \
		fabs(transform.y) >= DIS_THRESHOLD )
		return true;
	return false;
}


// those below are for graph optimization

void CMapGraph::resetGraph()
{
	delete optimizer_;
#ifndef ONLINE
	optimizer_ = new g2o::SparseOptimizer();
	SlamLinearCholmodSolver* linearSolver = new SlamLinearCholmodSolver();
	linearSolver->setBlockOrdering(false);
	SlamBlockSolver* solver = new SlamBlockSolver(optimizer_,linearSolver);
	optimizer_->setSolver(solver);
#else
	optimizer_ = new g2o::SparseOptimizerIncremental;
	optimizer_->initSolver(3,0);
#endif
	std::map<int,CFliterNode*>::iterator it = m_graph.begin();
	while(it!=m_graph.end())
	{
		delete it->second;
		it->second = NULL;
		it++;
	}
	m_graph.clear();
	m_reset_request = false;
}

bool CMapGraph::addEdgeFromOriented(int id1,int id2,int weight,OrientedPoint2D& trans,bool bigedge, bool reset)
{
	MatchingResult mr;
	((CFliterNode*)(0))->fromOrientedPoint2SE2(trans,mr.m_edge);
	mr.id1 = id1;
	mr.id2 = id2;
	Eigen::Matrix3d information;
	information.fill(0);
	if(weight<=0) weight=2;
	for(int i=0;i<3;i++)
		information(i,i) = weight*weight;
	mr.m_edge.setInformation(information);
	return addEdgeToG2O(mr,bigedge,reset);
}

bool CMapGraph::validVerify(CFliterNode* new_node, std::vector<int> index, OrientedPoint2D& fpose)
{
	// this should also be parameterized 
	static int least_match_num = 20;
	OrientedPoint2D total;
	int num = 0;
	for(int i=0;i<index.size();i++){
		CFliterNode* prefNode = m_graph[index[i]];
		OrientedPoint2D trans;
		pair<int,double> match_result = new_node->matchNodePairLocal(prefNode,trans);
		if(match_result.first < least_match_num)
			return false;
		trans = prefNode->m_pose.oplus(trans); // whether this should be the pose after optimization??
		total.x+=trans.x; total.y+=trans.y;
		num ++ ;
	}
	if(num == 0)
		return false;
	total.x /= num; total.y /= num;
	// this should also be parameterized
	static float max_dis = 1 ; // 1m 
	float average_dis = (total.x - fpose.x)*(total.x-fpose.x) + (total.y - fpose.y)*(total.y-fpose.y);
	// std::cout<<"average dis is: "<<average_dis<<std::endl;
	if(average_dis > max_dis)
		return false;
	return true;
}

// match not only the specified node , but also neighboring nodes in a window_size
bool CMapGraph::comparePreviousNodes(CFliterNode* new_node, int max_targets, int window_size)
{
	static int last_targets = 3; //always compare to the last n, spread evenly for the rest
	bool match_only_last = false; // only match with last several nodes
	int gsize = m_graph.size();
	//Special Cases
	if(gsize == 0){
		std::cout<<"Do not call this function as long as the graph is empty"<<std::endl;
		return false;
	}
	if(max_targets-last_targets < 0) // only compare with last several nodes
		return match_only_last = true;
	bool ret=false;
	
	if(window_size > gsize-1 )
		return match_only_last = true;

	// 1, first compare with last several targets;
	int side = (window_size-1)/2;
	if(side>gsize-1)
		return false;
	// this should be parameterized
	static int least_match_num = 20;
	// this should also be parameterized
	static float max_dis = 0.04 ; // 2dm 
#define SQURE(R) ((R)*(R))
	for(int pi = 1; pi<=side ; pi++)
	{
		OrientedPoint2D trans;
		OrientedPoint2D final;
		CFliterNode* prefNode = m_graph[m_graph.size()-1-pi];	
		pair<int,double> match_result = new_node->matchNodePairLocal(prefNode,trans);

		final = prefNode->m_pose.oplus(trans);
		if(SQURE(final.x-new_node->m_pose.x) + SQURE(final.y-new_node->m_pose.y) > max_dis)
			continue;
		if(addEdgeFromOriented(prefNode->m_id,new_node->m_id,match_result.first,trans,true,false))
			ret = true;
	}

	// 2, randomly compare with the rest
	if(match_only_last || gsize <= window_size*(max_targets-last_targets) )
		return ret;
		
	/*if(max_targets-last_targets == 0){
		ids_to_link_to.insert(0);
		return ids_to_link_to; //only compare to first frame
	} else if(max_targets-last_targets == 1){
		ids_to_link_to.insert(gsize-2);
		return ids_to_link_to; //only compare to previous frame
	}*/
	//End Special Cases
	if(gsize < max_targets)
		max_targets = gsize - 1;
	int total = max_targets - last_targets;
	int cnt = 0;
	std::set<int> record;
	std::vector<int> index;

	while(cnt<total){
		index.clear();
		int sample_id = rand()%(gsize - last_targets);
		if(record.find(sample_id) != record.end())
			continue;
		cnt++;
		record.insert(sample_id);
		CFliterNode* prefNode = m_graph[sample_id];
		OrientedPoint2D trans;
		OrientedPoint2D final;
		pair<int,double> match_result = new_node->matchNodePairLocal(prefNode,trans);
		final = prefNode->m_pose.oplus(trans);
		if(match_result.first < least_match_num)
			continue;
		for(int i=-side; i<=side ; i++){
			if(i==0) continue;
			int cur = sample_id+i;
			if(cur>=0 && cur<m_graph.size())
				index.push_back(cur);
		}
		if(window_size==1 || validVerify(new_node,index,final))
			if(addEdgeFromOriented(prefNode->m_id,new_node->m_id,match_result.first,trans,true,false))
			{
				std::cout<<"###Loop may be found between "<<new_node->m_id<<" and "<<prefNode->m_id<<std::endl;
				ret = true;
			}
	}

	return ret;
	/*std::vector<int> index;
	while(ids_to_link_to.size() < max_targets && ids_to_link_to.size() < gsize-1){
		int sample_id = rand() % (gsize - 1);
		if(ids_to_link_to.find(sample_id)!=ids_to_link_to.end())
			continue;
		ids_to_link_to.insert(sample_id);
	}*/
}

/// The parameter max_targets determines how many potential edges are wanted
/// max_targets < 0: No limit
/// max_targets = 0: Compare to first frame only
/// max_targets = 1: Compare to previous frame only
/// max_targets > 1: Select intelligently
std::set<int> CMapGraph::getPotentialEdgeTargets(CFliterNode* new_node, int max_targets)
{
	int last_targets = 3; //always compare to the last n, spread evenly for the rest
	std::set<int> ids_to_link_to;
	//max_targets = last_targets;
	int gsize = m_graph.size();
	//Special Cases
	if(gsize == 0){
		std::cout<<"Do not call this function as long as the graph is empty"<<std::endl;
		return ids_to_link_to;
	}
	if(max_targets-last_targets < 0)
		return ids_to_link_to;
	if(max_targets-last_targets == 0){
		ids_to_link_to.insert(0);
		return ids_to_link_to; //only compare to first frame
	} else if(max_targets-last_targets == 1){
		ids_to_link_to.insert(gsize-2);
		return ids_to_link_to; //only compare to previous frame
	}
	//End Special Cases

	//All the last few nodes
	if(gsize <= max_targets){
		last_targets = gsize;
	}
	for(int i = 2; i <= gsize && i <= last_targets; i++){//start at two, b/c the prev node is always already checked in addNode{
		ids_to_link_to.insert(gsize-i);
	}
	while(ids_to_link_to.size() < max_targets && ids_to_link_to.size() < gsize-1){
		int sample_id = rand() % (gsize - 1);
		if(ids_to_link_to.find(sample_id)!=ids_to_link_to.end())
			continue;
		ids_to_link_to.insert(sample_id);
	}
	return ids_to_link_to;
}

namespace{
// true iff edge qualifies for generating a new vertex
bool isBigTrafo(const g2o::SE2& edge){

	static double max_translation_meter = 4;
	double dist = edge.translation()[0]*edge.translation()[0] + edge.translation()[1]*edge.translation()[1];
#define M_PI 3.141592654
#define D2R(d) ((d)*M_PI/180.0)

	static double max_rotation_degree = D2R(30);
    // at least 2m or 5deg
    return (dist > max_translation_meter /*ParameterServer::instance()->get<double>("min_translation_meter")*/
    		|| edge.rotation().angle() >  max_rotation_degree/*ParameterServer::instance()->get<int>("min_rotation_degree")*/);
}
}

//! Add new node to the graph.
/// Node will be included, if a valid transformation to one of the former nodes
/// can be found. If appropriate, the graph is optimized
/// graphmanager owns newNode after this call. Do no delete the object
/// \callergraph
bool CMapGraph::addNode(CFliterNode* new_node)
{
	if (new_node->m_featurePoints.size() <= 2){
		printf("found only %i features on image, node is not included\n",(int)new_node->m_featurePoints.size());
		return false;
	}
	if(m_reset_request)
		resetGraph();
	MatchingResult last_matching_result;	// record last matching result

	//set the node id only if the node is actually added to the graph
	//needs to be done here as the graph size can change inside this function
	new_node->m_id = m_graph.size();

	//First Node, so only build its index, insert into storage and add a
	//vertex at the origin, of which the position is very certain
	if (m_graph.size()==0){
		m_graph[new_node->m_id] = new_node;
		g2o::VertexSE2* reference_pose = new g2o::VertexSE2;
		reference_pose->setId(0);
		reference_pose->setEstimate(g2o::SE2());
		reference_pose->setFixed(true);//fix at origin
		//optimizer_mutex.lock();
		optimizer_->addVertex(reference_pose);
		//optimizer_mutex.unlock();
		//current_poses_.append(latest_transform_);
		return true;
	}

	 unsigned int num_edges_before = optimizer_->edges().size();

	//MAIN LOOP: Compare node pairs ######################################################################
	//First check if trafo to last frame is big
	CFliterNode* prev_frame = m_graph[m_graph.size()-1];
	printf("Comparing new node (%i) with previous node %i", new_node->m_id, prev_frame->m_id);
	//MatchingResult mr = new_node->matchNodePair(prev_frame);
	MatchingResult mr;
	new_node->matchNodePair(prev_frame,mr);
	last_matching_result.m_inlier_number = mr.m_inlier_number;

	/*if(mr.edge.id1 >= 0 /*&& !isBigTrafo(mr.edge.mean)){
	              ROS_WARN("Transformation not relevant. Did not add as Node");
	              process_node_runs_ = false;
	              return false;
	          } else*/
	if(mr.id1 >= 0){
		if (addEdgeToG2O(mr, true, true)) {
			/*last_matching_node_ = mr.edge.id1;
	                  last_inlier_matches_ = mr.inlier_matches;
	                  last_matches_ = mr.all_matches;
	                  edge_to_previous_node_ = mr.edge.mean;*/

		} else {
			//process_node_runs_ = false;
			return false;
		}
	}

	// these should be parameterized
	static int max_compare_number = 10;
	static int optimizer_skip_step = 10;

	// this will avoid MatchingResult operator=() is called
	// int max_match_num_of_feature = -1;
	
	// compare with previous nodes
	std::set<int> vertices_to_comp = getPotentialEdgeTargets(new_node, max_compare_number);/*ParameterServer::instance()->get<int>("connectivity"));*/ //vernetzungsgrad
	std::set<int>::reverse_iterator it_comp = vertices_to_comp.rbegin();
	while(it_comp!=vertices_to_comp.rend()){
		CFliterNode* abcd = m_graph[*it_comp];
		MatchingResult mr2;
		new_node->matchNodePair(abcd,mr2);
		if (mr2.id1 >= 0) {
			if (addEdgeToG2O(mr2, isBigTrafo(mr2.m_edge.measurement()),
					last_matching_result.m_inlier_number < mr2.m_inlier_number)) { //TODO: result isBigTrafo is not considered
				if (last_matching_result.m_inlier_number < mr2.m_inlier_number)
					last_matching_result.m_inlier_number=mr2.m_inlier_number;
			}
		}
		it_comp++;
	}

	if (optimizer_->edges().size() > num_edges_before) { //Success
		m_graph[new_node->m_id] = new_node;
		if((optimizer_->vertices().size() %  optimizer_skip_step) /*ParameterServer::instance()->get<int>("optimizer_skip_step"))*/ == 0){
			optimizeGraph(10);
		} else {
		}
	}else{
		if(m_graph.size() == 1){//if there is only one node which has less features, replace it by the new one
			if(new_node->m_featurePoints.size() > m_graph[0]->m_featurePoints.size()){
				this->resetGraph();
				return this->addNode(new_node);
			}
		} else { //delete new_node; //is now  done by auto_ptr
		}
	}
	return optimizer_->edges().size()>num_edges_before;
}
// addNode using psm as frontend match
bool CMapGraph::addNodeImproved(CFliterNode* new_node)
{
	MatchingResult last_matching_result;	// record last matching result
	new_node->m_id = m_graph.size();
	if (m_graph.size()==0){
		m_graph[new_node->m_id] = new_node;
		g2o::VertexSE2* reference_pose = new g2o::VertexSE2;
		reference_pose->setId(0);
		reference_pose->setEstimate(g2o::SE2());
		reference_pose->setFixed(true);//fix at origin
		//optimizer_mutex.lock();
		optimizer_->addVertex(reference_pose);
		//optimizer_mutex.unlock();
		//current_poses_.append(latest_transform_);
		return true;
	}

	 unsigned int num_edges_before = optimizer_->edges().size();

	//MAIN LOOP: Compare node pairs ######################################################################
	//First check if trafo to last frame is big
	CFliterNode* prev_frame = m_graph[m_graph.size()-1];
	

	bool usingFL = false;
	bool usingFM = false;
	OrientedPoint2D fm_trans;
	try{	// using PSM or ICP as frontend match
		usingFM=new_node->matchNodeFrontend(prev_frame);
		fm_trans=new_node->m_relpose;
	}catch(int err){
		std::cout<<"FMatch fails!"<<std::endl;
		usingFM = false;
	}

	// printf("Comparing new node (%i) with previous node %i", new_node->m_id, prev_frame->m_id);
	MatchingResult mr;
	int match_num = new_node->matchNodePair(prev_frame,mr);
	// cout<<"node "<<new_node->m_id<<" has features: "<<new_node->m_featurePoints.size()<<endl;
	// cout<<"node "<<prev_frame->m_id<<" has features: "<<prev_frame->m_featurePoints.size()<<endl;
	// cout<<"node "<<new_node->m_id<<" with "<<prev_frame->m_id<<" match_num: "<<match_num<<endl;

	last_matching_result.m_inlier_number = match_num; //mr.m_inlier_number;

	/*if(mr.edge.id1 >= 0 /*&& !isBigTrafo(mr.edge.mean)){
	              ROS_WARN("Transformation not relevant. Did not add as Node");
	              process_node_runs_ = false;
	              return false;
	          } else*/
	
	if(mr.id1 >= 0){ // successful to match using Flirt
		usingFL = true; // Flirt features have been matched
		if(usingFM){ // using PSM to assign to Edge
			((CFliterNode*)0)->fromOrientedPoint2SE2(fm_trans,mr.m_edge);
			// std::cout<<"using FM instead!"<<std::endl;
		}
		if (addEdgeToG2O(mr, true, true)) {
			/*last_matching_node_ = mr.edge.id1;
	                  last_inlier_matches_ = mr.inlier_matches;
	                  last_matches_ = mr.all_matches;
	                  edge_to_previous_node_ = mr.edge.mean;*/

		} else {
			//process_node_runs_ = false;
			std::cout<<"failed to addEdgeToG2O!"<<std::endl;
			return false;
		}
	}
	
	if(!usingFM && !usingFL){
		std::cout<<"Failed to add Node: "<<new_node->m_id<<std::endl;
		return false;
	}
	else if(usingFM && !usingFL){
		std::cout<<"Using PSM as Initial Pose"<<std::endl;
		new_node->m_pose = prev_frame->m_pose.oplus(fm_trans);
	}

	// these should be parameterized
	static int max_compare_number = 20;//10;
	static int window_size = 3;
	static int optimizer_skip_step = 20;
	// Here, we will use our Randomly Matching Algorithm to compare will previous nodes
	bool preMatch = comparePreviousNodes(new_node,max_compare_number,window_size);
	if(!usingFL) usingFL = preMatch;

	// compare with previous nodes
	// std::set<int> vertices_to_comp = getPotentialEdgeTargets(new_node, max_compare_number);/*ParameterServer::instance()->get<int>("connectivity"));*/ //vernetzungsgrad
	/*std::set<int>::reverse_iterator it_comp = vertices_to_comp.rbegin();
	while(it_comp!=vertices_to_comp.rend()){
		CFliterNode* abcd = m_graph[*it_comp];
		MatchingResult mr2;
		new_node->matchNodePair(abcd,mr2);
		if (mr2.id1 >= 0) {
			usingFL = true; // Flirt features have been matched
			if (addEdgeToG2O(mr2, /*isBigTrafo(mr2.m_edge.measurement())*//*false,
					last_matching_result.m_inlier_number < mr2.m_inlier_number)) { //TODO: result isBigTrafo is not considered
				if (last_matching_result.m_inlier_number < mr2.m_inlier_number)
					last_matching_result.m_inlier_number=mr2.m_inlier_number;
			}
		}
		it_comp++;
	}
	*/

	if(!usingFL && usingFM){
		std::cout<<"Flirt Match failed! Using PSM or ICP create New Edge"<<std::endl;
		mr.id1 = prev_frame->m_id;
		mr.id2 = new_node->m_id;
		Eigen::Matrix3d information;
		information.fill(0.);
		if(match_num<=0) match_num=2;
		for(int i=0;i<3;i++)
			information(i,i) = match_num*match_num;
		mr.m_edge.setInformation(information);
		((CFliterNode*)(0))->fromOrientedPoint2SE2(fm_trans,mr.m_edge);
		if(!addEdgeToG2O(mr,true,true)){
			std::cout<<"Oops, failed to add to G2O!"<<std::endl;
		}
	}

	if (optimizer_->edges().size() > num_edges_before) { //Success
		m_graph[new_node->m_id] = new_node;
		if((optimizer_->vertices().size() %  optimizer_skip_step) /*ParameterServer::instance()->get<int>("optimizer_skip_step"))*/ == 0){
			optimizeGraph(10);
		} else {
		}
	}else{
		if(m_graph.size() == 1){//if there is only one node which has less features, replace it by the new one
			if(new_node->m_featurePoints.size() > m_graph[0]->m_featurePoints.size()){
				this->resetGraph();
				return this->addNode(new_node);
			}
		} else { //delete new_node; //is now  done by auto_ptr
		}
	}
	return optimizer_->edges().size()>num_edges_before;
}


void CMapGraph::optimizeGraph(int iter){
	// This should be parameterized
	static int n_iter=10;
	if(iter<=0) iter = n_iter;
	optimizer_->initializeOptimization();
	static double error_step_threshold = 0.00001;
	double last_err, cur_err;
	last_err = -1;
	for(int i=0; i<iter; i++){
		optimizer_->optimize(1);
		optimizer_->computeActiveErrors();
		cur_err = optimizer_->activeChi2();
		if(last_err >= 0){
			if(fabs(last_err - cur_err)<error_step_threshold)
				break;
		}
		last_err = cur_err;
	}
}

bool CMapGraph::addEdgeToG2O(MatchingResult& mr, bool large_edge, bool set_estimate)
{
	g2o::VertexSE2 * v1 = dynamic_cast<g2o::VertexSE2*>(optimizer_->vertex(mr.id1));
	g2o::VertexSE2 * v2 = dynamic_cast<g2o::VertexSE2*>(optimizer_->vertex(mr.id2));

	// assert the transformation is large enough to avoid too many vertices on the same spot
	if(!v1 || !v2){
		if(!large_edge){
			return false;
		}
	}
	if(!v1 && !v2){
		std::cout<<"both the nodes are not in the graph!"<<std::endl;
		return false;
	}
	else if(v1 && !v2){
		v2 = new g2o::VertexSE2;
		assert(v2);
		v2->setId(mr.id2);
		g2o::SE2 t = v1->estimate()*mr.m_edge.measurement();
		v2->setEstimate(t);
		optimizer_->addVertex(v2);
	}else if(v2 && !v1){
		v1 = new g2o::VertexSE2;
		assert(v1);
		v1->setId(mr.id1);
		g2o::SE2 t = v2->estimate()*mr.m_edge.inverseMeasurement();
		v1->setEstimate(t);
		optimizer_->addVertex(v1);
	}else {
		if(set_estimate){
			g2o::SE2 t = v1->estimate()*mr.m_edge.measurement();
			v2->setEstimate(t);
		}
	}
	g2o::EdgeSE2 * g2o_edge = new g2o::EdgeSE2;
	g2o_edge->vertices()[0] = v1;
	g2o_edge->vertices()[1] = v2;
	g2o_edge->setMeasurement(mr.m_edge.measurement());
	g2o_edge->setInformation(mr.m_edge.information());
	g2o_edge->setInverseMeasurement(mr.m_edge.inverseMeasurement());
	optimizer_->addEdge(g2o_edge);
	return true;
}




	/*if(mr2->id1 >= 0){
				/*if(!isLoopyArea(new_node,it->second)) // use marginal covariance 
				{	
					it++;
					continue;
				}*//*
				std::cout<<"Loop found between: "<<mr2->id1<<","<<mr2->id2<<std::endl;
				OrientedPoint2D trans;
				((CFliterNode*)0)->fromSE22OrientedPoint(trans,mr2->m_edge);
				
				new_node->matchNodePairICP(it->second,*mr2,&trans);
				((CFliterNode*)0)->fromSE22OrientedPoint(trans,mr2->m_edge);
				
				OrientedPoint2D newPose = it->second->m_rootPose->oplus(trans);

				// delete matches whose trans is beyond covariance of new_node
				OrientedPoint2D curr_trans = new_node->m_rootPose->ominus(newPose);
				if(!isAvaliableArea(new_node,curr_trans)){
					it++;
					continue;
				}*/
				/*addEdgeToG2O(*mr2,true,true);
				*(new_node->m_rootPose) = it->second->m_rootPose->oplus(trans);
				*(new_node->m_rootPoseBack) = *(new_node->m_rootPose);
				Eigen::Matrix3d tmp_cov = mr2->m_edge.information().inverse();
				getMatrix(cov,it->second->m_covariance);
				cov[0] += tmp_cov(0,0);
				cov[3] += tmp_cov(1,1);
				cov[5] += tmp_cov(2,2);
				break;*//*
				if(m_bUseConstantCov)
					mr2->m_edge.setInformation(m_cov_loopnode);
				matchSet.push_back(CMatchedPair(it->first,matched_num,newPose,mr2));
				// record this matched edge : features, pose, mr			
				// m_loopEdges.push_back(mr2);
				/*if(!isSmallEdge(curr_trans)){
					m_bLargeEdges.push_back(false);
					bloop = bLoop = true;
				}
				else
					m_bLargeEdges.push_back(true);
				*//*
				}else{
					// failed 
				}*/

