#ifndef GTREADER_H
#define GTREADER_H

#include <vector>
#include <string>

using namespace std;

class GTReader{
public:
	GTReader();
	~GTReader();
public:	
	bool readGT(const char*);
	vector<double> timestamp;
	vector<double> x;
	vector<double> y;
	vector<double> th;
	vector<vector<double> >covariance;
};


#endif

