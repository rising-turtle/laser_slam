/*
	2013.2.25 H Zhang
	PMap Interface
	input:  vector< vector<float> > x,y; // global points
		vector<float> px,py,pth; // trajectory
	        float prob_thre;  // probability threshold
	        float cell_size;  // grid cell size
	output: vector<float> ox, oy; 
		whose pro of cell is bigger than prob_thre;
	example:
		CPMap2D tmp(0.9,0.02); // prob: 0.09, cell_size: 2cm
		tmp.computePMAP(x,y,px,py,pth,ox,oy);
*/
#include <vector>
#include <iostream>
using namespace std;

class CPMap;
class CPMap2D {
public:
	CPMap2D(float prob_thre = 0.8, float cell_size = 0.05);
	~CPMap2D();
	void computePMAP(vector<vector<float> >& , \
		vector<vector<float> >&, \
		vector<float>&,vector<float>&,vector<float>&,\
		vector<float>&, vector<float>&);
	void computeBoundary(float&, float&, float&, float&, \
		vector<vector<float> >&, vector<vector<float> >&);
	CPMap * m_pPMap; 
	float m_prob_thre;
	float m_cell_size;
private:
	CPMap2D(const CPMap2D&);
	CPMap2D operator=(const CPMap2D&);
};
