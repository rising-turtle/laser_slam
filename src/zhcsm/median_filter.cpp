#include <iostream>
#include <stdlib.h>
#include <vector>

using namespace std;

void median_filter(vector<float>& ls)
{
	const int HALF_WINDOS = 2; 
	const int WINDOW = 2*HALF_WINDOS + 1;
	float r[WINDOW];
	float w;
	vector<float> ret(ls.size(),0);
	/*for(int i=0; i < ret.size();i++)
	{
		ret[i] = 0;
	}
	cout<<"after testing!"<<endl;*/

	int i,j,k,l;
	int N = ls.size();
	
	for( i=0 ; i<N; i++)
	{
		k = 0;
		for( j = i-HALF_WINDOS; j <= i+HALF_WINDOS; j++)
		{
			l = ((j>=0) ? j:0);
			r[k] = ls[(l<N)?l:(N-1)];
			k++;
		}
		for( j = (WINDOW-1); j>0; j-- )
		{
			for(k=0; k<j; k++)
			{
				if(r[k]>r[k+1])
				{
					w = r[k];
					r[k] = r[k+1];
					r[k+1] = w;
				}
			}
		}
		ret[i] = r[HALF_WINDOS]; 
		// ret[i] = r[i];
	}
	ret.swap(ls);
}

int main(int argc, char* argv[])
{
	srand(time(0));
	vector<float> r(541);
	for(int i=0; i< r.size(); i++)
	{
		r[i] = rand()%10;
		cout<<r[i]<<" ";
	}
	cout<<"after filtering!"<<endl;
	median_filter(r);
	for(int i=0; i< r.size(); i++)
	{
		cout<<r[i]<<" ";
	}
}
