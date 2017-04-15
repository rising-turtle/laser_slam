/*
 * test_sick.cpp
 *
 *  Created on: Jan 25, 2013
 *      Author: liu
 */


#include "CSICK.h"
#include "CObs2DScan.h"

#include <iostream>
#include <fstream>
#include <string>
#include <string.h>

#define PI 3.1415926;
#define D2R 0.01745

using namespace std;

int main() {

	string SICK_ip = "192.168.1.2";
	int SICK_port = 2112;
	CSICK laser(SICK_ip, SICK_port);

	if(!laser.turnOn())
	{
		cout<<"Laser can not turn on!"<<endl;
		return 0;
	}

	bool isOutObs, hardwareError;
	CObs2DScan outObs;
	laser.doProcessSimple(isOutObs, outObs, hardwareError);

	ofstream saveData;
	//save in rawseed format
	saveData.open("saveData.txt");
	int offset = 0;
	while(1)
	{
		TTimeStamp t_s = getCurrentTime();
		laser.doProcessSimple(isOutObs, outObs, hardwareError);
		if(isOutObs && outObs.scan.size() > 0)
		{
			saveData<<format("%.7f", (double)((int64_t)outObs.timestamp/10000000.0))<<", "<<outObs.scan.size()<<", "<<offset;
			for(int i=0;i<outObs.scan.size();i++)
			{
				saveData<<", "<<outObs.scan[i];
			}
			saveData<<endl;
		}
		TTimeStamp t_e = getCurrentTime();
		cout<<"Time: "<<timeDifference(t_s, t_e)*1000<<endl;
	}

	cout << "!!!Hello World!!!" << endl; // prints !!!Hello World!!!
	return 0;
}
