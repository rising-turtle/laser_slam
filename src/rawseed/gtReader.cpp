#include "gtReader.h"
#include <iostream>
#include <algorithm>
#include <string.h>
#include <cmath>
#include <fstream>

GTReader::GTReader(){}
GTReader::~GTReader(){}


bool GTReader::readGT(const char* file)
{
	ifstream inf(file);
	if(!inf.is_open()){
		cout<<"failed to open file: "<<file<<endl;
		return false;
	}
	cout<<"succeed to open file: "<<file<<endl;
	char line[4096];
	string debug; int cnt=0;
	vector<double> tmp_co(9);
	while(inf.getline(line,4096)){
		timestamp.push_back(atof(strtok(line," ")));
		
		x.push_back(atof(strtok(NULL," ")));
		y.push_back(atof(strtok(NULL," ")));
		th.push_back(atof(strtok(NULL," ")));
		/*for(int i=0;i<9;i++)
			tmp_co[i] = atof(strtok(NULL," "));
		covariance.push_back(tmp_co);*/
	}
	cout<<"obtain "<<x.size()<<" items!"<<endl;
	return true;
}
