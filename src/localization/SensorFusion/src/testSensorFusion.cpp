//============================================================================
// Name        : SensorFusion.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C, Ansi-style
//============================================================================

#include <stdio.h>
#include <stdlib.h>

#include "unsFlt.hpp"
#include <iostream>
#include <boost/numeric/ublas/io.hpp>

#include "config.hpp"

using namespace std;
using namespace Bayesian_filter;
using namespace Bayesian_filter_matrix;



/*
 * Filter a simple example
 */


int main(void) {
	// Global setup for test output
	cout.flags(ios::fixed); cout.precision(4);

	// Construct simple Prediction and Observation models

	Robot_predict robot_predict;
	SICK_observe sick_observe_1;
	SICK_observe sick_observe_2;
	GPS_observe gps_observe;

	// Use an 'Unscented' filter scheme with one state
	Unscented_scheme my_filter(3);

	// Setup the initial state and covariance
	Vec x_init (3); SymMatrix X_init (3, 3);
	x_init[0] = 0.;		// Start at 10 with no uncertainty
	x_init[1] = 0.;
	x_init[2] = 0.;
	X_init = ublas::zero_matrix<double>(3,3);
	my_filter.init_kalman (x_init, X_init);

	cout << "Initial  " << my_filter.x << my_filter.X << endl;

	// Predict the filter forward
	my_filter.predict (robot_predict);
	my_filter.update();		// Update the filter, so state and covariance are available

	cout << "Predict  " << my_filter.x << my_filter.X << endl;

	// Make an observation
	Vec z_1(3);
	z_1[0] = 3.;				// Observe that we should be at 11
	z_1[1] = 2.;
	z_1[2] = 1.;
	my_filter.observe (sick_observe_1, z_1);
	my_filter.update();		// Update the filter to state and covariance are available

	cout << "Filtered " << my_filter.x << my_filter.X << endl;

	my_filter.observe (sick_observe_2, z_1);
	my_filter.update();		// Update the filter to state and covariance are available
	cout << "Filtered " << my_filter.x << my_filter.X << endl;

	//test GPS
	Vec z_2(1);
	z_2[0] = 23.;				// Observe that we should be at 11

	my_filter.observe (gps_observe, z_2);
	my_filter.update();		// Update the filter to state and covariance are available

	cout << "GPS Filtered " << my_filter.x << my_filter.X << endl;

	puts("!!!Hello World!!!");
	return EXIT_SUCCESS;
}
