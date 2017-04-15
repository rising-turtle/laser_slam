#ifndef FLIRTER_NODE_H
#define FLIRTER_NODE_H

#include "preheader.h"
// #include <utils/HistogramDistances.h>
#include <geometry/point.h>

#include <Eigen/Core>  
#include <Eigen/Geometry>

#include <utility>
// #include "MatchingResult.h"
// #include "ZHPolar_Match.h"
#include <stdint.h>
typedef uint64_t TTimeStamp;

class InterestPoint;
class LaserReading;
class CPolarMatch;
class CCanonicalMatcher;

struct _PMScan;
typedef struct _PMScan PMScan;
namespace g2o{
class EdgeSE2;
}

class MatchingResult;

// detectors 
class CurvatureDetector;
class NormalBlobDetector;
class NormalEdgeDetector;
class RangeDetector;
class Detector;

// distance
template<class N>
class HistogramDistance;
template<class N>
class EuclideanDistance;
template<class N>
class Chi2Distance;
template<class N>
class SymmetricChi2Distance;
template<class N>
class BatthacharyyaDistance;
template<class N>
class KullbackLeiblerDistance; 
template <class N>
class JensenShannonDistance;

// peakFinders
class SimpleMinMaxPeakFinder;


// generators 
class BetaGridGenerator;
class ShapeContextGenerator;
class DescriptorGenerator;

// feature matchers
class RansacFeatureSetMatcher;

// used for session node
class CMapNode;

#define TEST_PERFORMANCE

class CFliterNode{
	friend class CMapNode;
public:
	explicit CFliterNode(LaserReading* lread, std::string laser_name="LMS151"); // construct node with carmon log from Flirter Lib
	explicit CFliterNode(PMScan* pmscan, CPolarMatch* laser_wrapper); // construct node with carmon log from our experiment
	explicit CFliterNode(PMScan* pmscan, CPolarMatch* laser_wrapper, TTimeStamp t); // add timestamp to node;
	
	~CFliterNode();
	void InitFliter();	// Init all the static parameters
	void InitNode(LaserReading* lread); // Init parameters of this node
	void UnitNode(); // delete all the features of this node
	// detect peak points
	// describe peak points 
	// match with previous nodes

	// identify whether all the parameters have been initialized
	static bool g_IsInit;
	static void resetInit();

	// peakFinders
	static SimpleMinMaxPeakFinder* g_peakMinMax;

	// detectors
	static string g_detector_name;
	static CurvatureDetector *g_detectorCurvature;	
	static NormalBlobDetector *g_detectorNormalBlob;
	static NormalEdgeDetector *g_detectorNormalEdge;
	static RangeDetector *g_detectorRange;
	static Detector *g_detector; 

	// distance
	static string g_distance_name;
	static HistogramDistance<double> *g_dist ;

	// descriptors
	static string g_descriptor_name;
	static BetaGridGenerator *g_betaGenerator;
	static ShapeContextGenerator *g_shapeGenerator;
	static DescriptorGenerator *g_descriptor ;
	
	// feature matcher
	static RansacFeatureSetMatcher *g_ransac;


	// parameters for Flirter Node
	typedef struct _Parameters{
		unsigned int scale;
		unsigned int dmst;
		unsigned int window;
		unsigned int detectorType;		// detector type	
		unsigned int descriptorType;		// descriptor type 
		unsigned int distanceType;			// distance type
		unsigned int matchStrategy;		// feature match strategy
		double baseSigma;
		double sigmaStep;
		double minPeak;
		double minPeakDistance;
		double acceptanceSigma; 
		double success;
		double inlier;
		double matchingThreshold;
		bool useMaxRange;
		explicit _Parameters();
	}FliterParameters;

	static FliterParameters& getParameter();
public:
	// compute marginal covariance for this node
	void computeMarginal(CFliterNode*);
	// information for this pose-node
	Eigen::Matrix3d m_covariance;
	Eigen::Matrix3d m_information; 
	void setCov(vector<double>&);
	float m_dis_mean_x ;
	float m_dis_mean_y ; 
	std::vector<InterestPoint *> m_featurePoints;	// feature points
	std::vector<InterestPoint*> m_featurePointsLocal; // feature points in local reference
	OrientedPoint2D m_gtpose; 					// gt's pose
	OrientedPoint2D m_pose;					// pose
	OrientedPoint2D m_relpose;					// relative pose to previous nodes
	PMScan* m_pScan;
	LaserReading * m_pLaserRead;				// laser reading info
	int m_id;			// id of this node
	int m_psyn; 		// num for tcp syn
	CPolarMatch* m_pFMatch;	// frontend PSM or ICP Matching
	CCanonicalMatcher* m_pFMatch2; // frontend CSM(PLICP) Matching
	TTimeStamp m_timestamp;
public:
	void fromSE22OrientedPoint(OrientedPoint2D& trans, g2o::EdgeSE2& ret);
	void fromOrientedPoint2SE2(OrientedPoint2D& trans, g2o::EdgeSE2& ret);
	pair<int,double> matchNodePairLocal( CFliterNode* refNode, OrientedPoint2D& transform);
	pair<int,double> matchNodePairGlobal( CFliterNode* refNode, OrientedPoint2D& transform);
	pair<int,double> matchFeaturePoints(std::vector<InterestPoint*>& fpref,std::vector<InterestPoint*>& 					fpcur,OrientedPoint2D& transform);
	int matchNodePair(CFliterNode* pref, MatchingResult& mr);
	int matchNodeFrontend(CFliterNode* pref); // match with previous Node using PSM + ICP
	int matchNodeFrontend2(CFliterNode* pref); // match with previous Node using CSM
	bool matchNodeFrontendGT(CFliterNode* pref, int& ); // match with previous Node with GT data
private:
	// transfrom between lread and pmscan
	LaserReading* fromPM2LR(PMScan* pmscan);
	PMScan* fromLR2PM(LaserReading* lread);
private:
	CFliterNode(std::string laser_name="LMS151"); // this construct can only be called by its friend class
};



#endif
