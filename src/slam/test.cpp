#include "slam.h"
#include <iostream>
#include <fstream>
#include <string>

using namespace std;

typedef void (*CallBack_SLAM)(double,double,double);

ofstream outf("test.log");
string input_file("/home/lyxp/work/mkproj/SVN/PFG/trunk/data/LMS151/t2/SICK_up.txt");

void cbSLAM(double x,double y,double th){
	static int cnt = 0;
	cout<<"pose "<<cnt++<<": "<<x<<" "<<y<<" "<<th<<endl;	
	outf<<x<<" "<<y<<" "<<th<<endl;
}

int main(int argc, char* argv[])
{
	CSlam slam;
	slam.InitLog(input_file,cbSLAM);
	cout<<"start!"<<endl;
	slam.start();

	slam.startEventLoop(argc,argv);
	outf.close();

	return 0;
}
