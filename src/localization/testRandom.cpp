#include <boost/random.hpp>
#include <iostream>

using namespace std;

int main(int argc, char* argv[])
{
	boost::mt19937 eng();
	boost::normal_distribution<> norm(0,1);
	for(int i=0;i<1000;i++)
	{
		// cout<< norm(eng)<<endl;
	}
	return 0;
}
