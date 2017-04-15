#include "slam_v1.h"
#include <iostream>
#include <fstream>
#include <string>

using namespace std;

ofstream outf("test_slamv1.log");

int cbSLAM(float x,float y,float th){
	static int cnt = 0;
	cout<<"pose "<<cnt++<<": "<<x<<" "<<y<<" "<<th<<endl;	
	outf<<x<<" "<<y<<" "<<th<<endl;
}

int cbSICK(vector<float>& bearing)
{
	static int cnt = 0;
	cout<<"scan "<<cnt++<<": "<<bearing[0]<<" "<<bearing[bearing.size()-1]<<endl;
}

SLAM_CallBack cbSets;

int main(int argc, char* argv[])
{
	cbSets.cbDataFusionResult = cbSLAM;
	cbSets.cbSICKA = cbSICK;
	CSlamV1 slamv1;
	slamv1.init(&cbSets,NULL);
	slamv1.m_work_model = 1;
	slamv1.setSystem();
	cout<<"start!"<<endl;
	slamv1.start();
	slamv1.startEventLoop(argc,argv);
	outf.close();

	return 0;
}
