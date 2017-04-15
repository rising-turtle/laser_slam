#include "PMap2D.h"

namespace{
	const double MAX_VAL = 1000000;
}
CPMap2D::CPMap2D(float prob_thre, float cell_size):
m_prob_thre(prob_thre),
m_cell_size(cell_size)
{}

CPMap2D::~CPMap2D(){}

void CPMap2D::computePMAP(vector<vector<float> >& obs_x, \
		vector<vector<float> >& obs_y,\
		vector<float>& px, vector<float>& py, vector<float>& pth,  
		vector<float>& ox, vector<float>& oy)
{
	size_t sx = obs_x.size();
	if(sx != obs_y.size() || sx != px.size() || sx != py.size() \
		|| sx != pth.size()){
		cout<<"wrong input data size!"<<endl;
		return ;
	}

	m_pPMAP = new CPMap(prob_thre);
	float min_x, max_x, min_y, max_y;
	int size_x, size_y;
	computeBoundary(min_x,max_x,min_y,max_y, obs_x, obs_y);
	size_x =( (max_x - min_x + 0.05) / m_cell_size )*2;
	size_y =( (max_y - min_y + 0.05) / m_cell_size )*2;
	// cout<<"boundary: "<<min_x<<" - "<<max_x<<","<<min_y<<" - "<<max_y<<endl;
	// cout<<"start initializeMap(): size: "<<size_x<<" * "<<size_y<<endl;
	m_pPMAP->initializeMap(size_x,size_y,size_x/2,size_y/2, 0,0,m_cell_size);
	// cout<<"start updateMap()!"<<endl;
	for(int i=0;i<obs_x.size();i++){
		m_pPMAP->updateMap(&obs_x[i][0],&obs_y[i][0],obs_x[i].size(),px[i],py[i],pth[i]);
	}
	// cout<<"start computeProbsOfMap()!"<<endl;
	m_pPMAP->computeProbsOfMap();
	m_pPMAP->getPointCloud(ox,oy);
	delete m_pPMAP;
	m_pPMAP = 0;
}

void CPMap2D::computeBoundary(float & min_x, float &max_x, float& min_y, float& max_y, vector<vector<float> >& g_obs_x, vector<vector<float> >& g_obs_y)
{
	min_x = min_y = MAX_VAL; max_x = max_y = -MAX_VAL; 
	for(int i=0;i<g_obs_x.size();i++){
		for(int j=0;j<g_obs_x[i].size();j++){
			if(g_obs_x[i][j]<min_x) min_x = g_obs_x[i][j];
			if(g_obs_x[i][j]>max_x) max_x = g_obs_x[i][j];
			if(g_obs_y[i][j]<min_y) min_y = g_obs_y[i][j];
			if(g_obs_y[i][j]>max_y) max_y = g_obs_y[i][j];
		}	
	}
}

