#include "stdio.h"
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <algorithm>
#include "string.h"


using namespace std;

string fuseMap(string map, string path)
{
	string ret("result.log");
	ofstream result(ret.c_str());
	ifstream fmap(map.c_str());
	ifstream fpath(path.c_str());

	char line[4096];
	char line2[1024];
	int tag;
	float x,y,th;
	while(fmap.getline(line,4096)){
		tag = atoi(strtok(line," "));
		if(!fpath.getline(line2,1024))
		{
			cout<<"not matched with trajectory!"<<endl;
			break;
		}
		sscanf(line2,"(%f,%f,%f)",&x,&y,&th);
		// sscanf(line2,"%f %f %f",&x,&y,&th);
		result<<tag<<" "<<x<<" "<<y<<" "<<th<<endl;
		fmap.getline(line,4096);
		result<<line<<endl;
	}
	return ret;
}
      

