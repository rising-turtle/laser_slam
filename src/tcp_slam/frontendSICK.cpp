/*
 * frontendSICK.cpp
 *
 *  Created on: Jan 11, 2013
 *      Author: liu
 */


#include "frontendSICK.h"

//using namespace zhpsm;

CFrontendSICK::CFrontendSICK():
sick_ip(""),
sick_port(0),
sick_name("")
{}
CFrontendSICK::~CFrontendSICK()
{}

void CFrontendSICK::runSick()
{
	runSICK(sick_ip, sick_port);
}

void CFrontendSICK::runRS()
{
	runRawSeed();
}

void CFrontendSICK::runCM()
{
	runCarmon();
}


void CFrontendSICK::setSickIP(string ip, unsigned int port)
{
	sick_ip = ip;
	sick_port = port;
}
void CFrontendSICK::setSickName(string name)
{
	sick_name = name;
}
void CFrontendSICK::setOriInRobot()//OrientedPoint2D p)
{
	//oriInRobot = p;

}

