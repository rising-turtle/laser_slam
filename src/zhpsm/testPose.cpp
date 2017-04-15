#include "point.h"
#include <iostream>
#include <cmath>

using namespace std;
using namespace zhpsm;

OrientedPoint2D p1(-21.2825,20.3314,1.765); // position
OrientedPoint2D p2(-8.23395,13.2828,1.13656);
OrientedPoint2D t1(9.7678,1.09852,-6.2241); // translation 
OrientedPoint2D t2(-1.2,3.01,-2.1);

void change(OrientedPoint2D& cor,OrientedPoint2D& rel, OrientedPoint2D& abs){
	float fcos = cos(cor.theta);
	float fsin = sin(cor.theta);
	abs.x = rel.x * fcos - rel.y * fsin;
	abs.y = rel.x * fsin + rel.y * fcos;
	abs.theta = rel.theta;
}

int main(){
	
	float fcos = cos(p1.theta);
	float fsin = sin(p1.theta);
	OrientedPoint2D t1_d;
	t1_d.x = t1.x * fcos - t1.y * fsin;
	t1_d.y = t1.x * fsin + t1.y * fcos;
	t1_d.theta = t1.theta;
	
	OrientedPoint2D r12 = p1.ominus(p2);
	OrientedPoint2D r12_d;
	change(p1,r12,r12_d);

	OrientedPoint2D p11 = p1.oplus(t1);

	OrientedPoint2D p22 = p2 + t1_d;
	OrientedPoint2D p23 = p11 + r12_d;
	cout<<"p22: "<<p22<<endl;
	cout<<"p23: "<<p23<<endl;
	
	
	/*OrientedPoint2D p11 = p1.oplus(t1);
	OrientedPoint2D p12 = p11.oplus(t2);
	OrientedPoint2D t13 = p1.ominus(p12);
	
	OrientedPoint2D p21 = p2.oplus(t1);
	OrientedPoint2D p22 = p21.oplus(t2);
	OrientedPoint2D p23 = p2.oplus(t13);

	cout<<"1 p22: "<<p22<<endl;
	cout<<"2 p23: "<<p23<<endl;
	*/

	return 0;
}
