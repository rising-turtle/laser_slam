#ifndef GRAPH_MANAGER_H
#define GRAPH_MANAGER_H

#include "preheader.h"
#include <fstream>
#include "FlirterNode.h"

// #include "g2o/core/graph_optimizer_sparse.h"
// #include "g2o/core/hyper_dijkstra.h"
namespace g2o{
class SparseOptimizer;
class SparseOptimizerOnline;
}
//#include "ZHPolar_Match.h"

// #define ONLINE
// #define SAVE_PMAP

class AbstractReading;
class MatchingResult;
class CPolarMatch;
class CMapNode;

class CMatchedPair{
public:
	CMatchedPair(int, float, OrientedPoint2D& , MatchingResult*);
	~CMatchedPair();
public:
	int m_id;
	float m_good;
	OrientedPoint2D* m_pose;
	MatchingResult* m_mr;
public:
	CMatchedPair(const CMatchedPair&);
	CMatchedPair& operator=(const CMatchedPair&);
};

typedef vector<CMatchedPair> CMPSet; 

// contains all the pose-node in a graph , 
// optimize this graph using Hogman or g2o
class CMapGraph{
public:
	CMapGraph(string laser_name="LMS151");
	~CMapGraph();

	ofstream m_record_matched;

	// read Sick laser data
	bool readSicklog(string logfile,std::vector<AbstractReading*>& log);
	bool runSicklog();
	
	bool constructMapGraphGTIncrementally();

	// test ground-truth : intel-lab
	bool constructPoseGraphGT();
	bool constructMapGraphGT();
	bool addMapNodeGT(CMapNode* new_node);
	void testGT(string outfile,int num_of_frame=-1); 
	void testGT1(string outfile, int num_of_frame=-1);

	// test map-node
	bool constructPoseGraph();
	bool constructMapGraph();

	bool addMapNode(CMapNode* new_node);
	bool isAvaliableArea(CMapNode*, OrientedPoint2D&);
	bool isLoopyArea(CMapNode* cur_node, CMapNode* pre_node, float);
	bool isOverlappedArea(CMapNode*, CMapNode*);
	void recordTraj(string outfile);
	// void recordFuseMap(string outfile);
	void recordMap(string outfile);
	void recordG2O();
	CMapNode* getLastNode(); // return the last mapNode
	void synPoseAfterOpt(); // syn pose after optimization
	std::map<int, CMapNode*> m_mapGraph;

	// for constant covariance
	Eigen::Matrix3d m_cov_adjnode;
	Eigen::Matrix3d m_cov_loopnode;
	Eigen::Matrix3d m_cov_corridor;
	bool m_bUseConstantCov;

	// for delayed G2O optimization
	bool addMapNodeCov(CMapNode* new_node, double[], bool&);
	void deleteMisMatch(CMPSet&);
	void calculateWeightedPose(CMPSet&, OrientedPoint2D&);
	int calculateFinalPose(CMPSet&, OrientedPoint2D& ,double*);
	vector<MatchingResult*> m_loopEdges;
	vector<bool> m_bLargeEdges;
	bool b_delayed_g2o; // whether to construct g2o offline
	bool constructDelayedG2o(); // construct delayed g2o structure
	
	bool calCovAdded(); // calculate the added covariance according to eature distribution
	bool calFeatureDistribute(vector<int>& ); // calculate the distribution of the features
	double m_f_sigma;
	int m_f_mean;

	static int g_session_size;
	
	// read and run ground-truth data in carmon format 
	bool readGTCarmon(string filename);
	void runGTCarmon(int run_num=-1);

	// read and run Our carmon data
	bool readOurCarmon(string filename);
	void runOurCarmon(int run_num=-1);
	void runComparison(int run_num=-1); // compare frontend with flirt
	
	// read Our RawSeed data
	bool readOurRawSeed(string filename);

	// read log file using groundtruth
	bool readlog(string logfile);
	bool runlog();

	void recordPoseTraj(string outfile);

	// record trajectory
	void recordTrajectory(string outfile);
	bool IsNoisyMove(OrientedPoint2D& transform);
public:
	CPolarMatch* m_pPSM; // used to read our carmon and store all datas
public:
	std::vector<AbstractReading*> m_log;	// read log files
	map<int, CFliterNode*> m_graph; // contains the trajectory of robot
	// map<int, OrientedPoint2D*> m_before_opt; // contains the trajectory of the robot before optimization
public:
#ifndef ONLINE
	g2o::SparseOptimizer* optimizer_; // g2o optimization
#else
	g2o::SparseOptimizerOnline* optimizer_; // g2o online optimizer
#endif
	bool m_reset_request;
public:
	void testbug();
public:
	// our Randomly matching Algorithm to compare Previous Nodes
	bool comparePreviousNodes(CFliterNode* new_node, int max_targets, int window_size);
	bool validVerify(CFliterNode* new_node, std::vector<int> index, OrientedPoint2D& fpose);
	bool addEdgeFromOriented(int id1,int id2,int weight,OrientedPoint2D& trans,bool bigedge, bool reset);

	
	// for test g2o algorithm
	void dumpG2oTrajectory(std::string filename);
	void runG2olog();
	
	void resetGraph();
	void optimizeGraph(int iter);

	//! Add new node to the graph.
	/// Node will be included, if a valid transformation to one of the former nodes
	/// can be found. If appropriate, the graph is optimized
	/// graphmanager owns newNode after this call. Do no delete the object
	/// \callergraph
	bool addNode(CFliterNode* newNode);
	bool addNodeImproved(CFliterNode* newNode);
	/// The parameter max_targets determines how many potential edges are wanted
	/// max_targets < 0: No limit
	/// max_targets = 0: Compare to first frame only
	/// max_targets = $: Compare to previous frame only
	/// max_targets > $: Select intelligently
	std::set<int> getPotentialEdgeTargets(CFliterNode* new_node, int max_targets);

	//std::vector<int> getPotentialEdgeTargetsFeatures(const Node* new_node, int max_targets);
	bool addEdgeToG2O(MatchingResult& mr, bool large_edge, bool set_estimate=false);
	bool isLargeEdge(OrientedPoint2D&);
	bool isSmallEdge(OrientedPoint2D&);
	bool isBigAngleDiff(OrientedPoint2D*, OrientedPoint2D*);
};




#endif
