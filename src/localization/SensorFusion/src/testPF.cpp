/*
 * testFastSLAM.cpp
 *
 *  Created on: Mar 5, 2013
 *      Author: liu
 */

#include "testPF.h"

int main (int argc, char* argv[])
{
	// Global setup for test output
	std::cout.flags(std::ios::fixed); std::cout.precision(4);

	unsigned nParticles = 1000;
	if (argv[1])
	{
    	try {
			nParticles = boost::lexical_cast<unsigned>(argv[1]);
		}
		catch (boost::bad_lexical_cast) {
			// ignore error and use default
		}
	}
	std::cout << "nParticles = " << nParticles << std::endl;

	// Create test and run experiments
	try {
		SLAMDemo test(nParticles);
		test.pfExp();
		//test.InformationLossExperiment();
	}
	catch (const BF::Filter_exception& ne)
	{
		std::cout << ne.what() << std::endl;
	}
}
