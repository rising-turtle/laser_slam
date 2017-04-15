#ifndef MAP_NODE_H
#define MAP_NODE_H

#include "./preheader.h"
#include "FlirterNode.h"

class CFliterNode;
class MatchingResult;
class OrientedPoint2D;
class Point2D;

#define USE_PMAP

#ifdef USE_PMAP
class CPMap;
class CICPWarpper;
#endif

// reduce a session of pose-nodes into a map-node
class CMapNode{
public:
	CMapNode();
	CMapNode(vector<CFliterNode*>& , int duo_size=-1);
	~CMapNode();
	bool m_IsReady;
public:
	// Reduce pose nodes into a Single Map node
	void reduceIntoMapNode(const vector<CFliterNode*>& , int duo_size );
	double calPositionDis(const OrientedPoint2D& , const OrientedPoint2D& );
public:
	// Incremetnally add pose-node into map-node, until the number of its features
	// arrives to certain threshold, default 100
	bool addPoseNodeIncrementally(CFliterNode* new_node, int feature_thre=100);
	// add the new_node into map_node, return the number of newly added features 
	int addPoseNode(CFliterNode* new_node);
	void finishReduction(); // to register all the feature points
	void dumpToFile();
public:
	OrientedPoint2D* m_rootPose;	// root node of this submap
	OrientedPoint2D* m_rootPoseBack; // root pose back up
	vector<OrientedPoint2D*> m_trajectory; // relative trajectory
	vector<OrientedPoint2D*> m_abs_trajectory; // abs trajectory
	vector<InterestPoint*> m_featurePoints; // features relative to root
	vector<int> m_fpre; // duo-graph pre set
	vector<int> m_fpos; // duo-graph pos set
public:
	// add the observation of new_pose
	void translate2GlobalFrame(float* bearing, vector<float>&fx,vector<float>& fy, double rx, double ry, double th);
	void addObsOfPoseNode(CFliterNode*);
	void calculateAllObs(vector<float>& , vector<float>&);
	void getOneObs(int, vector<float>&, vector<float>&,float&,float&,float& );
	vector< vector<float> > m_bearing; // observation
	vector<float> m_obs_x; // observation points x
	vector<float> m_obs_y; // observation points y
public:
	int matchNodePairGT(CMapNode* pref,MatchingResult& mr);
	int matchNodePair(CMapNode* pref, MatchingResult& mr);
	void getICPResult(int&, float&, float&);
	int matchNodePairICP(CMapNode* pref, MatchingResult& mr, OrientedPoint2D* initPose);
	CFliterNode* m_pFMatch;	// used to match feature points
	int m_id;
	int m_root_id;
	int m_psyn;		// number for tcp syn
public:
	static int g_threshold_duo_size;
	static float g_matching_threshold;
	static float g_distance_threshold;
private:
	CMapNode(const CMapNode&);
	CMapNode& operator=(const CMapNode&);
private: 
public:
	int getNumofFeatures(){return m_iNum_of_unique_fps;}
public:
	bool featureEqual(InterestPoint* p1, InterestPoint* p2);
	int findCloestFeature(vector<InterestPoint*>& , InterestPoint*, double&, double&);
	bool fuseFeature(InterestPoint* fp);

	int m_iNum_of_unique_fps; 
	vector<InterestPoint*> fvector;
	vector<InterestPoint*> cur_fp;
	vector<vector<InterestPoint*> > rvector;
public:
	void getUncertainty(float&,float&,float&,float&,float&);
	Eigen::Matrix3d m_covariance;
	Eigen::Matrix3d m_information;
	
	Eigen::Matrix3d m_incremental_Cov;
	Eigen::Matrix3d m_last_Cov;
	Eigen::Matrix3d m_root_Cov;
	Eigen::Matrix3d m_final_Cov;

	float m_dis_mean_x;
	float m_dis_mean_y;
	
	// observation area
	void updateObsRange(const OrientedPoint2D&);
	Point2D * m_ld_corner; // the left-down corner 
	Point2D * m_ru_corner; // the right-up corner

#ifdef USE_PMAP
public:
	double gaussPDF(double,double,double);
	void featurePDF(int,int,double, Eigen::Matrix3d&);
	void computeGlobalScans();
	void computeBoundary(float& min_x,float &max_x, float& min_y, float& max_y);
	void computePMAP();
	vector< vector<float> > g_obs_x;
	vector< vector<float> > g_obs_y;
	volatile bool m_bPMapReady;
	CPMap * m_pPMAP;
	CICPWarpper* m_pICP;
	static Eigen::Matrix3d max_sigma;
	static double s_sigma;
	static int s_mean;
#endif
};

#endif
