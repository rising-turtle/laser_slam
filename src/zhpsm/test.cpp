#include <string>
#include <iostream>
#include "ZHPolar_Match.h"

using namespace std;
string file1("/home/lyxp/work/exprdata/data/intel-lab.log");
string file2("indoor.txt");
string file3("../data/Bicocca_2009-02-26a-SICK_FRONT.log");
string file4("/mnt/hgfs/SharedFold/dataset/lenovo/LMS511_data/lms511-2_output.txt");
string file5("SICK511.txt");
string file6("/mnt/hgfs/SharedFold/dataset/lenovo/LMS511_data/lms511-2_output.txt");
int main(int argc, char* argv[])
{
	// CPolarMatch mypsm(_BEARING_181);
	// mypsm.runFlirtFile(file3,100);

	// mypsm.testAccuracy(file1);

	// CPolarMatch mypsm(_BEARING_541);
	// mypsm.runOurFile(file2);
	
	CPolarMatch mypsm("LMS511");
	mypsm.run511Data(file6, -1);

	cout<<"finished!"<<endl;
	return 0;
}
