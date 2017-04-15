#ifndef MATCHING_RESULT_H
#define MATCHING_RESULT_H

#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/vertex_point_xy.h"
#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/slam2d/edge_se2_pointxy.h"
#include "g2o/math_groups/se2.h"

using namespace g2o;
//using namespace g2o::tutorial;

// most important is the edge information after the match

class MatchingResult{	
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	MatchingResult(){}
	~MatchingResult(){}
public:
	int id1,id2;  // id of the matched nodes
	g2o::EdgeSE2 m_edge;
	int m_inlier_number;	// indicate the number of inlier from this match
};


#endif
