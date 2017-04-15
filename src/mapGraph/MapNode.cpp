#include "MapNode.h"
#include "MatchingResult.h"
#include "ZHPolar_Match.h"
#include "ZHIcp_Warpper.h"
#include <utils/HistogramDistances.h>
#include <geometry/point.h>
#include <feature/InterestPoint.h>
#include <sstream>
#include <fstream>

#ifdef USE_PMAP
#include "PMap.h"
double CMapNode::s_sigma = 8;
int CMapNode::s_mean = 47;
Eigen::Matrix3d CMapNode::max_sigma; 
namespace{
  const double g_sigma_x = 0.05; //0.1;
  const double g_sigma_th = 0.00005; //0.0001;
  const double g_scale = 1.0;//1.414; // 1.732; 
}
#endif

#define MAX_VALUE 100000
#define MIN_VALUE -MAX_VALUE

int CMapNode::g_threshold_duo_size = 10;//10; // Threshold for duo-graph
float CMapNode::g_matching_threshold =  1.0;//0.8;  // 1.0; // 0.4 // Threshold for considering as inliers
float CMapNode::g_distance_threshold =  0.2 ;// 0.1; //0.2; // 0.01 Threshold for considering as the same features

int CMapNode::findCloestFeature(vector<InterestPoint*>& fset, InterestPoint* p1, double& min_dis, double& min_des)
{
	int ret = -1;
	for(int i=0;i<fset.size();i++)
	{
		InterestPoint* p2 = fset[i];
		double descriptor_distance = p1->getDescriptor()->distance(p2->getDescriptor());
		double position_distance = calPositionDis(p1->getPosition(),p2->getPosition());
		bool flag = false;
		if(descriptor_distance < min_des){
			if(position_distance < min_dis ){
				flag = true;
			}else if(min_dis - position_distance <= g_distance_threshold){
				flag = true;
			}
			if(flag){
				min_dis = position_distance;
				min_des = descriptor_distance;
				ret = i;
			}
		}
	}
	return ret;
}

bool CMapNode::featureEqual(InterestPoint* p1, InterestPoint* p2){
	double descriptor_distance = p1->getDescriptor()->distance(p2->getDescriptor());
	double position_distance = calPositionDis(p1->getPosition(),p2->getPosition());
	if(descriptor_distance < g_matching_threshold && position_distance < g_distance_threshold)
	{
		return true;
	}
	return false;
}

CMapNode::CMapNode():
m_rootPose(new OrientedPoint2D),
m_rootPoseBack(new OrientedPoint2D),
m_pFMatch(new CFliterNode()),
m_IsReady(false),
m_id(-1),
#ifdef USE_PMAP
m_pPMAP(new CPMap),
m_pICP(new CICPWarpper),
m_bPMapReady(false),
#endif
m_iNum_of_unique_fps(0),
m_psyn(0),
m_ld_corner(new Point2D(MAX_VALUE,MAX_VALUE)),
m_ru_corner(new Point2D(MIN_VALUE,MIN_VALUE))
{
	#ifdef USE_PMAP
	double sigma_max = g_sigma_x;
	max_sigma.fill(0);
	max_sigma(0,0) = max_sigma(1,1) = sigma_max;
	max_sigma(2,2) = g_sigma_th;
	#endif
}
CMapNode::CMapNode(vector<CFliterNode*>& nodes, int duo_size):
m_rootPose(new OrientedPoint2D),
m_rootPoseBack(new OrientedPoint2D),
m_pFMatch(new CFliterNode()),
m_IsReady(false),
m_id(-1),
#ifdef USE_PMAP
m_pPMAP(new CPMap),
m_pICP(new CICPWarpper),
m_bPMapReady(false),
#endif
m_iNum_of_unique_fps(0),
m_psyn(0),
m_ld_corner(new Point2D(MAX_VALUE,MAX_VALUE)),
m_ru_corner(new Point2D(MIN_VALUE,MIN_VALUE))

{
	#ifdef USE_PMAP
	double sigma_max = g_sigma_x;
	max_sigma.fill(0);
	max_sigma(0,0) = max_sigma(1,1) = sigma_max;
	max_sigma(2,2) = g_sigma_th;
	#endif

	duo_size = duo_size<=0? g_threshold_duo_size:duo_size;
	reduceIntoMapNode(nodes,duo_size);
}
CMapNode::~CMapNode(){
	if(m_rootPose)
		delete m_rootPose;
	if(m_rootPoseBack)
		delete m_rootPoseBack;
	if(m_pFMatch)
		delete m_pFMatch;
	for(int i=0;i<m_trajectory.size();i++)
		delete m_trajectory[i];
	for(int i=0;i<m_abs_trajectory.size();i++)
		delete m_abs_trajectory[i];
	for(int i=0;i<m_featurePoints.size();i++)
		delete m_featurePoints[i];
	if(m_ld_corner)
		delete m_ld_corner;
	if(m_ru_corner)
		delete m_ru_corner;
}
double CMapNode::calPositionDis(const OrientedPoint2D& p1, const OrientedPoint2D& p2)
{
	return  (p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y);
}

namespace{
	void showMatrix(Eigen::Matrix3d& m, string name){
		cout<<name<<endl;
		for(int i=0;i<3;i++){
			for(int j=0;j<3;j++){
				cout<<m(i,j)<<" ";
			}
			cout<<endl;
		}
	}
}

void CMapNode::updateObsRange(const OrientedPoint2D& p)
{
	if(p.x < m_ld_corner->x) m_ld_corner->x = p.x;
	if(p.y < m_ld_corner->y) m_ld_corner->y = p.y;
	if(p.x > m_ru_corner->x) m_ru_corner->x = p.x;
	if(p.y > m_ru_corner->y) m_ru_corner->y = p.y;
}

#ifdef USE_PMAP
double CMapNode::gaussPDF(double x, double mean, double sigma){
	double b = sigma*(-2.);
	return exp(((x-mean)*(x-mean))/b);
}

void CMapNode::featurePDF(int n, int mean, double sigma, Eigen::Matrix3d& m){
	
	if(n >= mean - g_scale*sigma) return ;
	double y = gaussPDF(n, mean, sigma*sigma);
	// cout<<"feature: "<<n <<" param y: "<<1-y<<endl;
	Eigen::Matrix3d add_ = (1- y)*CMapNode::max_sigma;
	// showMatrix(add_,"add: ");
	m+=add_;
}
#endif
// call this function to finish fusing all the feature points
void CMapNode::finishReduction(){
	bool bfused = false;
	m_featurePoints.resize(fvector.size(), NULL);
	int index = 0;
	for(int i=0;i<rvector.size();i++){
		// m_featurePoints[i]=new InterestPoint(*fvector[i]);
		m_featurePoints[index]=new InterestPoint(*fvector[i]);
		// dense LMS151 1 sparse LMS211 0
		if(rvector[i].size()==0) // 1
			continue;
		bfused = true;
		float fx=0;
		float fy=0;
		for(int j=0;j<rvector[i].size();j++){
			fx+=rvector[i][j]->getPosition().x;
			fy+=rvector[i][j]->getPosition().y;
		}
		fx = fx/(float)(rvector[i].size());
		fy = fy/(float)(rvector[i].size());
		// m_featurePoints[i]->setPosition(OrientedPoint2D(fx,fy,fvector[i]->getPosition().theta));
		m_featurePoints[index]->setPosition(OrientedPoint2D(fx,fy,fvector[i]->getPosition().theta));
		index++;
	}
	m_featurePoints.resize(index);
	/*if(bfused) cout<<"fuse succeed!"<<endl;
	else cout<<"fail to fuse fp!"<<endl;*/

	// set ready flag
	m_IsReady = true;

#ifdef USE_PMAP
	// cout<<"start building PMAP!"<<endl;
	// computePMAP();
	// cout<<"finish building PMAP!"<<endl;
#endif
	// compute information matrix of the edge between adjacent MapNodes
	// Eigen::Matrix3d delta1_Cov = m_incremental_Cov;
	// Eigen::Matrix3d delta2_Cov = m_final_Cov - m_root_Cov;
	// showMatrix(delta1_Cov,"incremetal_cov");
	// showMatrix(delta2_Cov,"minus_cov");

	m_covariance = m_incremental_Cov;

	/*for(int i=0;i<3;i++)
		for(int j=0;j<3;j++){
			if(i!=j) m_covariance(i,j) = 0;
		}
	*/
#ifdef USE_PMAP
	// featurePDF(m_iNum_of_unique_fps,m_covariance);
#endif
	m_information = m_covariance.inverse();
	
	// cout<<"MapNode range: x["<<m_ld_corner->x<<","<<m_ru_corner->x<<"], y["<<m_ld_corner->y<<","<<m_ru_corner->y<<"]"<<endl;

	return ;
}

bool CMapNode::fuseFeature(InterestPoint* fp){
	// static vector<InterestPoint*> fvector;
	// static map<int, vector<InterestPoint*> > rvector;
	
	double min_dis = 1e17;
	double min_des = 1e17;

	int index = findCloestFeature(fvector,fp,min_dis,min_des);
	
	if(index >= 0 && min_des < g_matching_threshold && min_dis < g_distance_threshold){
		rvector[index].push_back(fp);
		return true;
	}

	/*for(int i=0;i<fvector.size();i++){
		if(featureEqual(fp,fvector[i])){ // this feature can be fused
			// TODO: fvector[i] should be fused with fp
			// fvector[i] = average(fvector[i],fp)
			rvector[i].push_back(fp);
			return true;
		}
	}*/
	// a new feature point
	vector<InterestPoint*> new_vec;
	new_vec.push_back(fp);
	cur_fp.push_back(fp);
	// fvector.push_back(fp);
	rvector.push_back(new_vec);

	return false;
}

// Incremetally add pose-node into map-node until the features of this map-node
// arrives to certain threshold
bool CMapNode::addPoseNodeIncrementally(CFliterNode* new_node, int feature_thre){
	bool feature_added = false; 
	// static int n_valid_points = 0;
	if(new_node->m_featurePointsLocal.size()<=0)
		return false;
	// set root info for this MapNode
	if(m_iNum_of_unique_fps<=0){
		*m_rootPose = new_node->m_pose;
		*m_rootPoseBack = new_node->m_pose;
		m_root_id = new_node->m_id;
		m_psyn = new_node->m_psyn;
		// copy uncertainty to root_node
		m_root_Cov = new_node->m_covariance;
		m_last_Cov = m_root_Cov;
		m_incremental_Cov = m_last_Cov - m_last_Cov;
		}

	CFliterNode* pcurNode = new_node;
	for(int j=0;j<pcurNode->m_featurePointsLocal.size();j++){
		InterestPoint* fp = pcurNode->m_featurePointsLocal[j];
		InterestPoint* p = new InterestPoint(*fp);
		p->setPosition(m_rootPose->ominus(pcurNode->m_pose.oplus(fp->getPosition()))); // (FP+Pi-Pr)
		updateObsRange(p->getPosition());
		if(!fuseFeature(p)) // succeed to fuse with this Feature Point
		{
			m_iNum_of_unique_fps++;
			feature_added = true;
		}
	}
	
	// add feature into MapNode
	fvector.insert(fvector.end(),cur_fp.begin(),cur_fp.end());
	cur_fp.clear();

	if(!feature_added){
		return false;
	}
	// 3 record trajectory
	m_trajectory.push_back(new OrientedPoint2D(m_rootPose->ominus(new_node->m_pose)));
	m_abs_trajectory.push_back(new OrientedPoint2D(new_node->m_pose - *m_rootPose));

	// 4 record observation
	addObsOfPoseNode(new_node);
	
	// 5 record COV
	m_final_Cov = new_node->m_covariance;
	Eigen::Matrix3d delta_Cov = new_node->m_covariance - m_last_Cov;
	
	if(delta_Cov(0,0) >0 && delta_Cov(1,1) >0 ){
		 // if( delta_Cov(0,0) < 0.5 || delta_Cov(1,1) < 0.5)
			m_incremental_Cov += delta_Cov;
	}
	m_last_Cov = m_final_Cov;
	if(m_iNum_of_unique_fps>=feature_thre){
		return true;
	}
	return false;
} 

void CMapNode::getUncertainty(float& px,float& py ,float& pth, float& a, float& b)
{
	px = (float)m_rootPose->x;
	py = (float)m_rootPose->y;
	pth = (float)m_rootPose->theta;
	a = m_dis_mean_x;
	b = m_dis_mean_y;
	return ;
}

void CMapNode::addObsOfPoseNode(CFliterNode* new_node)
{

	vector<float> bearing(new_node->m_pFMatch->m_pParam->pm_l_points);
	float* pb = new_node->m_pScan->r;
	for(int i=0;i<bearing.size();i++,pb++)
		bearing[i] = *pb;
	m_bearing.push_back(bearing);
	/*vector<float> fx;
	vector<float> fy;
	translate2GlobalFrame(new_node->m_pScan->r,fx,fy,new_node->m_pose.x,new_node->m_pose.y,new_node->m_pose.theta);
	m_obs_x.insert(m_obs_x.end(),fx.begin(),fx.end());
	m_obs_y.insert(m_obs_y.end(),fy.begin(),fy.end());*/
}

void CMapNode::getOneObs(int index, vector<float>& fx, vector<float>& fy,float& rx, float& ry, float& th)
{
	if(index>= m_bearing.size() || index<0){
		cout<<"error in getOneObs()"<<endl;
		return ;
	}
	OrientedPoint2D cur_pose = m_rootPose->oplus(*(m_trajectory[index]));
	translate2GlobalFrame(&m_bearing[index][0],fx,fy,cur_pose.x,cur_pose.y,cur_pose.theta);
	rx = (float)(cur_pose.x);
	ry = (float)(cur_pose.y);
	th = (float)(cur_pose.theta);
	return ;
}

void CMapNode::calculateAllObs(vector<float>& fx, vector<float>& fy)
{
	OrientedPoint2D root_pose = *m_rootPose;
	OrientedPoint2D cur_pose;
	if(m_trajectory.size()!=m_bearing.size()){
		cout<<"big error in caculateAllObs()!"<<endl;
		return ;
	}
	for(int i=0;i<m_trajectory.size();i++){
		vector<float> tx;
		vector<float> ty;
		cur_pose = root_pose.oplus(*(m_trajectory[i]));
		// cur_pose = root_pose + *(m_trajectory[i]);
		translate2GlobalFrame(&m_bearing[i][0],tx,ty,cur_pose.x,cur_pose.y,cur_pose.theta);
		fx.insert(fx.end(),tx.begin(),tx.end());
		fy.insert(fy.end(),ty.begin(),ty.end());
	}	
	return ;
}



void CMapNode::translate2GlobalFrame(float* bearing, vector<float>& fx, vector<float>& fy, double rx, double ry, double th)
{

#define PI 3.141592654

/* LMS151 */
// [-45, 225] 270  

#define BEAR_NUM 541
#define PI 3.141592654
#define MIN_ANGLE -(PI/4.)
#define MAX_RANGE 50
#define FOV (1.5*PI)

/* LMS211 */
// [-90, 90] 180  
/*
#define BEAR_NUM 181
#define MAX_RANGE 50
#define MIN_ANGLE -(PI/2.)
#define FOV PI
*/

/* LMS511 */
// [-90, 90] 361  
/*
#define BEAR_NUM 361
#define MAX_RANGE 50
#define MIN_ANGLE -(PI/2.)
#define FOV PI
*/
const float bad_range = -100000; 
	static bool initAngle = false;
	static float cosAngle[BEAR_NUM];
	static float sinAngle[BEAR_NUM];
	static float Angle[BEAR_NUM];
	if(!initAngle){
		initAngle = true;
		static float minAngle =  MIN_ANGLE;  // 0.0;
		static float dfi = (float)((FOV)/(BEAR_NUM-1)); // dfi 0.5' 
		for(int i=0;i<BEAR_NUM;i++){
			Angle[i]=minAngle + dfi*i;
			cosAngle[i] = cosf(Angle[i]);
			sinAngle[i] = sinf(Angle[i]);
		}
		//cout<<"minAngle: "<<minAngle<<"; maxAngle "<<Angle[BEAR_NUM-1]<<endl;
	}

	/*if(bearing.size()!=BEAR_NUM){
		cout<<"something is wrong!"<<endl;
	}*/

	if(fx.size()!=BEAR_NUM){
		fx.resize(BEAR_NUM);
		fy.resize(BEAR_NUM);
	}

	float x,y,tx,ty,nIdx,nIdy;
	float fcos = cosf(th);
	float fsin = sinf(th);
	int index=0;
	for(int i=0;i<BEAR_NUM;i++){
		if(bearing[i]>MAX_RANGE*100){ // this is bad range 
			// fx[i]=fy[i]=bad_range;
			continue;
		}
		x=(bearing[i]/100.0)*cosAngle[i];
		y=(bearing[i]/100.0)*sinAngle[i];
		tx=fcos*x-fsin*y+rx;
		ty=fsin*x+fcos*y+ry;

		fx[index] = tx;
		fy[index] = ty;
		index++;
	}
	fx.resize(index);
	fy.resize(index);
}

int CMapNode::addPoseNode(CFliterNode* new_node)
{
	int last_num = this->m_iNum_of_unique_fps;
	addPoseNodeIncrementally(new_node);
	return (this->m_iNum_of_unique_fps-last_num);
}

void CMapNode::reduceIntoMapNode(const vector<CFliterNode*>& node_set, int duo_size)
{
	if(node_set.size()<=0){
		cout<<"node_set is empty!"<<endl;
		return ;
	}
	int overlap = duo_size > g_threshold_duo_size? duo_size:g_threshold_duo_size;
	// 1 set rootNode		
	*m_rootPose = node_set[0]->m_pose;
	// 2 filter duplicate features
	vector< vector<InterestPoint*> >totalF(node_set.size());
	for(int i=0; i<node_set.size();i++){
		CFliterNode* pcurNode = node_set[i];
		for(int j=0;j<pcurNode->m_featurePointsLocal.size();j++){
			InterestPoint* fp = pcurNode->m_featurePointsLocal[j];
			InterestPoint* p = new InterestPoint(*fp);
			p->setPosition(m_rootPose->ominus(pcurNode->m_pose.oplus(fp->getPosition()))); // (FP+Pi-Pr)
			// m_featurePoints.push_back(p);
			totalF[i].push_back(p);
		}
	}
	vector< vector<bool> > flag(totalF.size());
	for(int i=0;i<totalF.size();i++){
		flag[i].resize(totalF[i].size(),false);
	}
	for(int i=0;i<totalF.size();i++){
		for(int j=0;j<totalF[i].size();j++){
			if(flag[i][j]) continue;
			// flag[i][j] = true;
			bool Add2PreSet=false;
			bool Add2PosSet=false;
			if(i<g_threshold_duo_size)
				Add2PreSet = true;
			if(i>=totalF.size()-g_threshold_duo_size)
				Add2PosSet = true;
			InterestPoint* psearch = totalF[i][j];
			vector<InterestPoint*> similarset;
			similarset.push_back(psearch);
			// search for the rest features
			for(int k=i+1;k<totalF.size();k++){
				for(int l=0;l<totalF[k].size();l++){
					if(flag[k][l]) continue;
					double descriptor_distance = psearch->getDescriptor()->distance(totalF[k][l]->getDescriptor());
					double position_distance = calPositionDis(psearch->getPosition(),totalF[k][l]->getPosition());
					if(descriptor_distance < g_matching_threshold && position_distance < g_distance_threshold)
					{
						if(k>=totalF.size()-g_threshold_duo_size)
							Add2PreSet = true;
						similarset.push_back(totalF[k][l]);
						flag[k][l] = true;
					}
				}
			}
			InterestPoint * rp = new InterestPoint(*psearch);
			// find similar feature points
			if(similarset.size()>=2){ 
				flag[i][j] = true;
				float tx=0,ty=0;
				for(int m=0;m<similarset.size();m++)
				{
					tx+=similarset[m]->getPosition().x;
					ty+=similarset[m]->getPosition().y;
				}
				tx/=similarset.size();
				ty/=similarset.size();
				rp->setPosition(OrientedPoint2D(tx,ty,rp->getPosition().theta));
			}
			// if this feature only be detected once, it may not be included
			// we can take experiment later
			m_featurePoints.push_back(rp);
			if(Add2PreSet)
				m_fpre.push_back(m_featurePoints.size()-1);
			if(Add2PosSet)
				m_fpos.push_back(m_featurePoints.size()-1);
		}
	}
	// delete totalF
	for(int i=0;i<totalF.size();i++)
		for(int j=0;j<totalF[i].size();j++)
			{
				delete totalF[i][j];
				totalF[i][j] = NULL;
			}
	// 3 record trajectory
	m_trajectory.resize(node_set.size());
	m_abs_trajectory.resize(node_set.size());
	for(int i=0;i<node_set.size();i++)	
	{
		m_trajectory[i]= new OrientedPoint2D(m_rootPose->ominus(node_set[i]->m_pose));
		m_abs_trajectory[i] = new OrientedPoint2D(node_set[i]->m_pose - *m_rootPose);
	}
	// 4 set the ready flag
	m_IsReady = true;
}
int CMapNode::matchNodePairGT(CMapNode* pref, MatchingResult& mr){
	if(!pref->m_IsReady || !this->m_IsReady)
	{
		std::cout<<"either map-node is not ready!"<<std::endl;
		return -1;
	}
	OrientedPoint2D transform;
	pair<int,double> match_result = m_pFMatch->matchFeaturePoints(pref->m_featurePoints,this->m_featurePoints,transform);
	// actually this can be achieved much better, 
	// currently, we simply use the original features as threshold T{50, 0.5}
	static double match_ratio = 0.7; //0.7 
	// static int match_least = 110; 
	int match_threshold = this->m_featurePoints.size()*match_ratio;
	// match_threshold = match_threshold < match_least? match_threshold : match_least;
	if(match_result.first >= match_threshold) // successfully matched
	{
		std::cout<<"Succeed, matched number: "<<match_result.first<<std::endl;
		m_pFMatch->fromOrientedPoint2SE2(transform,mr.m_edge);
		mr.id1 = pref->m_id;
		mr.id2 = this->m_id;
		Eigen::Matrix3d information;
		information.fill(0.);
		double inf = 1./match_result.second;
		for(int i=0;i<3;i++)
			information(i,i) = inf; // TODO: weighted according to the observed times
		mr.m_edge.setInformation(information);
	}
	else{
		mr.id1 = -1;
		mr.id2 = -1;
	}
	return match_result.first;
}

void CMapNode::dumpToFile()
{
#ifdef USE_PMAP
	stringstream sid;
	sid<<m_id<<".log";
	string ss;
	sid>>ss;
	ofstream outf(ss.c_str());
	for(int i=0;i<m_obs_x.size();i++)
	{
		outf<<m_obs_x[i]<<" "<<m_obs_y[i]<<endl;
	}
	outf.close();
#endif
}

void CMapNode::getICPResult(int& n_iter, float& goodness, float& quality){
	#ifdef USE_PMAP
		m_pICP->getMatchQuality(n_iter, goodness, quality);
	#else
		cout<<"MapNode.cpp: Error, USE_PMAP not defined!"<<endl;
	#endif
}

int CMapNode::matchNodePairICP(CMapNode* pref, MatchingResult& mr, OrientedPoint2D* initPose)
{
	
	#ifdef USE_PMAP
		pref->computePMAP();
		this->computePMAP();
		double p[3];
		double cov[6];
		m_pICP->ICPMatch(pref->m_obs_x,pref->m_obs_y,m_obs_x,m_obs_y,initPose->x,initPose->y,initPose->theta);
		m_pICP->getResult(p,cov);
		OrientedPoint2D transform(p[0],p[1],p[2]);
		m_pFMatch->fromOrientedPoint2SE2(transform,mr.m_edge);
		
		float scale = 1;
		Eigen::Matrix3d covariance;
		covariance.fill(0.); // this is very important!
		covariance(0,0) = cov[0]*scale;
		covariance(0,1) = covariance(1,0) = cov[1];
		covariance(0,2) = covariance(2,0) = cov[2];
		covariance(1,1) = cov[3]*scale;
		covariance(1,2) = covariance(2,1) = cov[4];
		covariance(2,2) = cov[5];
		
		// covariance += max_sigma;

		Eigen::Matrix3d information = covariance.inverse();
		mr.m_edge.setInformation(information);
	#else
		cout<<"MapNode.cpp: Error, USE_PMAP not defined!"<<endl;
	#endif
}

int CMapNode::matchNodePair(CMapNode* pref, MatchingResult& mr){
	if(!pref->m_IsReady || !this->m_IsReady)
	{
		std::cout<<"either map-node is not ready!"<<std::endl;
		return -1;
	}
	static double match_ratio = 0.65; //0.75; // 0.7; //0.7
	static int match_least =  60; //25
	if(pref->m_featurePoints.size() < match_least || m_featurePoints.size() < match_least)
	{
		mr.id1 = mr.id2 = -1;
		return 0;
	}
	OrientedPoint2D transform;
	pair<int,double> match_result = m_pFMatch->matchFeaturePoints(pref->m_featurePoints,this->m_featurePoints,transform);
	// cout<<"matched num between "<<m_id<<" and "<< pref->m_id<<" : "<<match_result.first<<endl;
	// actually this can be achieved much better, 
	int match_threshold = this->m_featurePoints.size()*match_ratio;
	match_threshold = match_threshold > match_least? match_threshold : match_least;
	
	if(match_result.first >= match_threshold) // successfully matched
	{
		// std::cout<<"Succeed, matched number: "<<match_result.first<<std::endl;
		m_pFMatch->fromOrientedPoint2SE2(transform,mr.m_edge);
		mr.id1 = pref->m_id;
		mr.id2 = this->m_id;
		Eigen::Matrix3d information;
		information.fill(0.); // this is very important!
		double inf = 1./match_result.second;
		/*for(int i=0;i<3;i++)
			information(i,i) = inf; // TODO: weighted according to the observed times
		*/
		for(int i=0;i<3;i++)
			information(i,i) = match_result.first*1000;
		mr.m_edge.setInformation(information);
	}
	else{
		mr.id1 = -1;
		mr.id2 = -1;
	}
	return match_result.first;
}

#ifdef USE_PMAP
#define MAX_VAL 10000
#define RESOLUTION 0.05 // 5cm
void CMapNode::computeGlobalScans(){
	OrientedPoint2D* curr_pose;
	vector<float> obs_x;
	vector<float> obs_y;
	for(int i=0;i<m_trajectory.size();i++){
		curr_pose = m_trajectory[i];
		translate2GlobalFrame(&m_bearing[i][0],obs_x,obs_y,curr_pose->x,curr_pose->y,curr_pose->theta);
		g_obs_x.push_back(obs_x);
		g_obs_y.push_back(obs_y);
	}
}
void CMapNode::computeBoundary(float & min_x, float &max_x, float& min_y, float& max_y)
{
	min_x = min_y = MAX_VAL; max_x = max_y = -MAX_VAL; 
	for(int i=0;i<g_obs_x.size();i++){
		for(int j=0;j<g_obs_x[i].size();j++){
			if(g_obs_x[i][j]<min_x) min_x = g_obs_x[i][j];
			if(g_obs_x[i][j]>max_x) max_x = g_obs_x[i][j];
			if(g_obs_y[i][j]<min_y) min_y = g_obs_y[i][j];
			if(g_obs_y[i][j]>max_y) max_y = g_obs_y[i][j];
		}	
	}
}
void CMapNode::computePMAP(){
	if(m_bPMapReady)
		return;
	float min_x, max_x, min_y, max_y;
	int size_x, size_y;
	// cout<<"start computeGlobalScans()!"<<endl;
	computeGlobalScans();
	// cout<<"start computeBoundary()!"<<endl;
	computeBoundary(min_x,max_x,min_y,max_y);
	size_x =( (max_x - min_x + 0.05) / RESOLUTION )*2;
	size_y =( (max_y - min_y + 0.05) / RESOLUTION )*2;
	// cout<<"boundary: "<<min_x<<" - "<<max_x<<","<<min_y<<" - "<<max_y<<endl;
	// cout<<"start initializeMap(): size: "<<size_x<<" * "<<size_y<<endl;
	m_pPMAP->initializeMap(size_x,size_y,size_x/2,size_y/2, 0,0,RESOLUTION);
	OrientedPoint2D* curr_pose;
	// cout<<"start updateMap()!"<<endl;
	for(int i=0;i<g_obs_x.size();i++){
		curr_pose = m_trajectory[i];
		m_pPMAP->updateMap(&g_obs_x[i][0],&g_obs_y[i][0],g_obs_x[i].size(),curr_pose->x,curr_pose->y,curr_pose->theta);
	}
	// cout<<"start computeProbsOfMap()!"<<endl;
	m_pPMAP->computeProbsOfMap();
	m_pPMAP->getPointCloud(m_obs_x,m_obs_y);
	m_bPMapReady = true;
	// delete allocated memory
	{
		delete m_pPMAP;
		m_pPMAP = NULL;
		vector< vector<float> > tmpx;
		vector< vector<float> > tmpy;
		g_obs_x.swap(tmpx);
		g_obs_y.swap(tmpy);
	}
}
#endif
