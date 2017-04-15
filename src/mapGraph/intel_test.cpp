#include "preheader.h"
#include "MapGraph.h"
#include "ZHPolar_Match.h"

#include <Eigen/Core>
#include <Eigen/Dense>

namespace{
	string intel_log = "../data/intel-lab.log";
	string outname="/mnt/hgfs/SharedFold/log/intel_g2o.log";
	string outname1="/mnt/hgfs/SharedFold/log/intel_g2o_p.log";

};

int main(int argc, char* argv[]){
	//
	int n_of_frames = -1;
	CMapGraph graph("LMS211");
	graph.readGTCarmon(intel_log);
	graph.constructPoseGraphGT();
	cout<<"finish constructPoseGraphGT()"<<endl;
	// graph.constructMapGraphGT();
	graph.constructMapGraphGTIncrementally();
	cout<<"finish constructMapGrpah()"<<endl;
	graph.recordTraj(outname);
	graph.recordPoseTraj(outname1);

	// graph.testGT(outname,n_of_frames);
	
	/*cout<<"psm"<<endl;
	CPolarMatch mypsm("LMS211");
	mypsm.runFlirtFile(intel_log,n_of_frames);*/
	

	// graph.runGTCarmon(100);
	/*graph.constructPoseGraphGT();
	cout<<"finish constructPoseGraphGT()"<<endl;
	graph.constructMapGraphGT();
	cout<<"finish constructMapGrpah()"<<endl;
	graph.recordTraj(outname);*/
	cout<<"finished!"<<endl;
	return 0;
}
