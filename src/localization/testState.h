#ifndef TEST_STATE_H
#define TEST_STATE_H
#include <string>

using namespace std;

struct _PMScan;
class CPolarMatch;

extern bool constructPSMfromRawSeed(char*, struct _PMScan&, CPolarMatch*);
extern int frontObject(struct _PMScan&, float, float, float&);
extern string g_scan_file;
/*
* Return : 
* 	-1 error happened
*	0 speed up
*	1 stop -> turn angle
*	2 stop -> not turn angle
*	3 slow -> turn angle
* 	4 slow -> not turn angle
*/ 
extern int Look4Window2(struct _PMScan& ls, float& angle, int start, int end, float slow_dis, float stop_dis);

#endif
