#include "ZHCanonical_Matcher.h"
#include <iostream>

using namespace std;

string rawfile1("/mnt/hgfs/SharedFold/dataset/rawseed/Bicocca_2009-02-25b/Bicocca_2009-02-25b-SICK_FRONT.csv");

string lenovot1("/home/lyxp/work/mkproj/SVN/PFG/trunk/data/LMS151/t1/SICK_up.txt");

string lenovo511("/mnt/hgfs/SharedFold/dataset/lenovo/LMS511_data/lms511-2_output.txt");

string lenovo511_1("/mnt/hgfs/SharedFold/dataset/lenovo/LMS511_data/SICK511.txt");


string lenovot4("/mnt/hgfs/SharedFold/dataset/lenovo/static151/t2.txt");

int main(int argc, char* argv[])
{	
	CCanonicalMatcher cs_matcher;
	// cs_matcher.testKeyFrameMatch(rawfile1,100);
	// cs_matcher.compareCSM2PSM_RawSeed(rawfile1,5000);
	cs_matcher.compareCSM2PSM_RawSeed(lenovot4,1000);
	// cs_matcher.compareCSM2PSM_Lenovo(lenovot4,1000);
	// cs_matcher.compareCSM2PSM_511(lenovo511,-1);
	cout<<"Complete!"<<endl;
	return 0;
}

