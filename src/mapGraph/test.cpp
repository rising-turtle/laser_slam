
#include "preheader.h"

#include "MapGraph.h"

#include <Eigen/Core>
#include <Eigen/Dense>

namespace{
	string indoor = "../data/indoor.txt";
	string outname="/mnt/hgfs/SharedFold/log/hg2o.log";
};

int main(int argc, char* argv[]){
	CMapGraph graph;
	// 1 read our indoor data
	graph.readOurCarmon(indoor);
	// 2 construct pose-graph
	graph.constructPoseGraph();
	std::cout<<"finish constructPoseGraph()"<<std::endl;
	//graph.testbug();
	//graph.recordTrajectory(outname);
	// 3 construct map-graph
	graph.constructMapGraph();
	std::cout<<"finish constructMapGraph()"<<std::endl;
	// 4 record trajectory
	graph.recordTraj(outname);
	cout<<"finished!"<<endl;
	return 0;
}
