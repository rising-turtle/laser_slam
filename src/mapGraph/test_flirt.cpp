#include "preheader.h"

#include "MapGraph.h"

#include "FlirterNode.h"

namespace{
	string indoor = "/mnt/hgfs/SharedFold/dataset/lenovo/LMS151/20130108_t1.txt";
	string outname="/mnt/hgfs/SharedFold/log/hg2o.log";
};

int main(int argc, char* argv[]){
	CMapGraph graph;
	// 1 read our indoor data
	graph.readOurRawSeed(indoor);
	// 2 construct pose-graph
	graph.constructPoseGraph();
	std::cout<<"finish constructPoseGraph()"<<std::endl;
	return 0;
}
