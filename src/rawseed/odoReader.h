#ifndef ODOREADER_H
#define ODOREADER_H

#include <vector>
#include <string>

using namespace std;

class ODOReader{
public:
	ODOReader();
	~ODOReader();
public:	
	bool readODO(const char*);
	vector<double> timestamp;
	vector<int> count;
	vector<int> left;
	vector<int> right;
	vector<double> x;
	vector<double> y;
	vector<double> th;
};


#endif

