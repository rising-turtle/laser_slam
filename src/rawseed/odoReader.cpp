#include "odoReader.h"
#include <iostream>
#include <algorithm>
#include <string.h>
#include <cmath>
#include <fstream>

ODOReader::ODOReader(){}
ODOReader::~ODOReader(){}


bool ODOReader::readODO(const char* file)
{
	ifstream inf(file);
	if(!inf.is_open()){
		cout<<"failed to open file: "<<file<<endl;
		return false;
	}
	cout<<"succeed to open file: "<<file<<endl;
	char line[4096];
	while(inf.getline(line,4096)){
		timestamp.push_back(atof(strtok(line," ")));
		count.push_back(atoi(strtok(NULL," ")));
		left.push_back(atoi(strtok(NULL," ")));
		right.push_back(atoi(strtok(NULL," ")));
		x.push_back(atof(strtok(NULL," ")));
		y.push_back(atof(strtok(NULL," ")));
		th.push_back(atof(strtok(NULL," ")));
	}
	cout<<"obtain "<<x.size()<<" items!"<<endl;
	return true;
}
