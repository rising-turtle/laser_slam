/*
 * frontendSICK.h
 *
 *  Created on: Jan 11, 2013
 *      Author: liu
 */

#ifndef FRONTENDSICK_H_
#define FRONTENDSICK_H_

#include "clientFrontend.h"

//namespace zhpsm{class OrientedPoint2D;}
//using namespace std;

class CFrontendSICK: public CClientFrontend
{
	Q_OBJECT
public:
	CFrontendSICK();
	~CFrontendSICK();

public Q_SLOTS:
	void runSick();
	void runRS(); // rawseed

public:
	void setSickIP(string, unsigned int);
	void setSickName(string);
	void setOriInRobot();//zhpsm::OrientedPoint2D);

private:
	string sick_name;
	string sick_ip;
	unsigned int sick_port;
	//zhpsm::OrientedPoint2D a;
	//zhpsm::OrientedPoint2D oriInRobot;
};



#endif /* FRONTENDSICK_H_ */
