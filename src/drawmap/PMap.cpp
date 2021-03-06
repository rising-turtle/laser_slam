#include "PMap.h"
namespace{
	const float little_tail = 0.0005;
	const double std_pro = 0.3;
}
CPMap::CPMap(float prob_thre): m_prob_thre(prob_thre)
{
	// default_init(); // init the map using default parameters
}
CPMap::~CPMap(){}

void CPMap::default_init(){
	initializeMap(2000,2000,400,300,0,0,0.05); // 5cm cell size
}

void CPMap::initializeMap(int size_x, int size_y, int start_x, int start_y,float offset_x, float offset_y, double resolution)
{
	m_size_x = size_x;
	m_size_y = size_y;
	m_center_x = start_x;
	m_center_y = start_y;
	m_offset_x = offset_x;
	m_offset_y = offset_y;
	m_resolution = resolution;
	m_max_line_lens = 4*max(m_size_x,m_size_y);

	m_maphit.resize(size_x, vector<float>(size_y,0) );
	m_mapsum.resize(size_x, vector<float>(size_y,0) );
	m_mapprob.resize(size_x, vector<float>(size_y,0));

	/*m_maphit = (float**)malloc(size_x*size_y*sizeof(float*));
	m_mapsum = (float**)malloc(size_x*size_y*sizeof(float*));
	m_mapprob = (float**)malloc(size_x*size_y*sizeof(float*));
	
	for(int i=0;i<size_x;i++)
		for(int j=0;j<size_y;j++)
		{
			m_maphit[i*size_y + j] = 0;
			m_mapsum[i*size_y + j] = 0;
			m_mapprob[i*size_y + j] = 0;
		}*/
}
void CPMap::uninitMap(){}

// input the global cordinates of the scans and current pose
void CPMap::updateMap(float* gx, float* gy, int n, float rx, float ry, float th)
{
	static vector<ivector2d>    	 line;
	int 				 cnt; // number of this line
	int                            i, j, x, y;
	ivector2d            		 start, end;
	
	// n == -1 indicate finishing sending the map info
	if(n<0){
		cout<<"Finally send PMAP!"<<endl;
		computeProbsOfMap();
		// draw this map 
		// sendMap(this);
		return; 
	}
	
	// cout<<"$$$rece update map!"<<endl;

	static int nframe = 0;

	if(line.size()<=0) line.resize(m_max_line_lens);

	// caculate start grid
	mapPosfromPoint(Point2D(rx,ry),start);
	for(int i=0;i<n;i++){
		mapPosfromPoint(Point2D(*(gx+i),*(gy+i)),end);
		if( end.x>=0 && end.x < m_size_x \
				&& end.y>=0 && end.y < m_size_y){
			m_maphit[end.x][end.y]++;	
		}

		// line update model
		grid_line( start, end, line, cnt );
		for (int j=0;j<cnt/*line.size()*/;j++) {
			x = line[j].x;
			y = line[j].y;
			if ( x>=0 && x<m_size_x &&
					y>=0 && y<m_size_y ) {
				if (j>/*line.size()*/cnt-2) {
					m_maphit[x][y]++;
				}
				m_mapsum[x][y]++;
			} 
		}
	}
}
void CPMap::mapPosfromPoint(float px, float py, int& ix, int& iy)
{
	Point2D point(px,py);
	ivector2d pos;
	mapPosfromPoint(point,pos);
	ix = pos.x;
	iy = pos.y;
}
void CPMap::mapPosfromPoint(Point2D point, ivector2d& pos)
{
	if(point.x < m_offset_x){
		pos.x = (int) (m_center_x+
				((point.x-m_offset_x +little_tail )/(double)m_resolution)- 1);
	}else{
		pos.x = (int) (m_center_x + 
				((point.x-m_offset_x + little_tail)/(double)m_resolution));
	}
	if(point.y < m_offset_y){
		pos.y = (int) (m_center_y+
				((point.y-m_offset_y +little_tail )/(double)m_resolution)- 1);

	}else{
		pos.y = (int) (m_center_y + 
				((point.y-m_offset_y+little_tail)/(double)m_resolution));
	}
}
// compute probability of each grid in this map
void CPMap::computeProbsOfMap()
{
	int  x, y;
	for (x=0;x<m_size_x;x++) {
		for (y=0;y<m_size_y;y++) {
			if (m_mapsum[x][y]>0) {
				m_mapprob[x][y] =
					( m_maphit[x][y] /
					  (double) ( /*m_maphit[x][y] +*/ m_mapsum[x][y] ) );
				if(m_mapprob[x][y]>1.0) m_mapprob[x][y] = 1.0;
			} else {
				m_mapprob[x][y] = std_pro; // undetected points
					/*settings.global_map_std_val*/;
			}
		}
	}

}

void CPMap::getPointCloud(vector<float>&x, vector<float>& y )
{
	for(int i=0;i<m_size_x;i++){
		for(int j=0;j<m_size_y;j++){
			if(m_mapprob[i][j]>=m_prob_thre)
			{
				x.push_back((i-m_center_x)*m_resolution);
				y.push_back((j-m_center_y)*m_resolution);
			}
		}
	}
}
