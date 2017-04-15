#include "drawmap.h"
#include "ZHPolar_Match.h"
#include "PMap.h"
#include <iostream>
#include <fstream>
#include <QPainter>


#define BIG_N 100000
CDrawMap::CDrawMap( string laser, double cell_size):
m_pLaser(new CPolarMatch(laser.c_str())),
m_pPMAP(new CPMap),
m_cell_size(cell_size)
{
	max_y = max_x = -BIG_N; 
	min_y = min_x = BIG_N;
}
CDrawMap::~CDrawMap(){}

bool CDrawMap::readLog(CPolarMatch* m_pl, string file )
{
	char line[4096];
	ifstream in(file.c_str());
	if(!in.is_open()){
		cout<<"failed to open file: "<<file<<endl;
		return false;
	}
	int tag;
	vector<float> bearing(m_pl->m_pParam->pm_l_points);
	float p;
	cout<<"begin to read Log!"<<endl;
	while(in.getline(line,4096)){
		tag = atoi(strtok(line," "));
		if(tag == m_pl->m_pParam->pm_l_points)
		{
			for(int i=0;i<tag;i++)
			{
				bearing[i] = atof(strtok(NULL," "));
			}
			// translate?
			translate2GlobalFrame(bearing,*(m_px.rbegin()),*(m_py.rbegin()),*(m_pth.rbegin()));
		}
		else if(tag == 1 || tag ==0 ){
			p = atof(strtok(NULL," "));
			m_px.push_back(p);
			p = atof(strtok(NULL," "));
			m_py.push_back(p);
			p = atof(strtok(NULL," "));
			m_pth.push_back(p);
			m_bIsRoot.push_back(tag==1);
		}else{
			cout<<"unknown tag: "<<tag<<endl;
			return false;
		}
	}
	cout<<"total obtain : "<<m_px.size()<<" scans!"<<endl;
	return true;
}
void CDrawMap::translate2GlobalFrame(vector<float>& bearing, float rx, float ry, float th)
{
	int BEAR_NUM = m_pLaser->m_pParam->pm_l_points;
	vector<float> fx(BEAR_NUM);
	vector<float> fy(BEAR_NUM);

	float x,y,tx,ty,nIdx,nIdy;
	float fcos = cosf(th);
	float fsin = sinf(th);
	int index=0;
	for(int i=0;i<BEAR_NUM;i++){
		if(bearing[i] >= m_pLaser->m_pParam->pm_max_range || 
			bearing[i] <= m_pLaser->m_pParam->pm_min_range ||
			bearing[i] >= 1500){ // this is bad range 
			continue;
		}
		x=(bearing[i]/100.0)*m_pLaser->pm_co[i];
		y=(bearing[i]/100.0)*m_pLaser->pm_si[i];

		tx=fcos*x-fsin*y+rx;
		ty=fsin*x+fcos*y+ry;

		if(tx < min_x) min_x = tx;
		if(tx > max_x) max_x = tx;
		if(ty < min_y) min_y = ty;
		if(ty > max_y) max_y = ty;

		fx[index] = tx;
		fy[index] = ty;
		index++;
	}
	fx.resize(index);
	fy.resize(index);
	m_obs_x.push_back(fx);
	m_obs_y.push_back(fy);
	// cout<<"finish translate!!"<<endl;
}
void CDrawMap::drawLog(string input, string out){
	// 1 read all these data and translate to global coordinates
	if(!readLog(m_pLaser,input)){
		cout<<"failed to read log: "<<input.c_str();
		return ;
	}
	cout<<"after read Log!"<<endl;
	cout<<"(minx,miny): ("<<min_x<<","<<min_y<<")"<<endl;
	cout<<"(maxx,maxy): ("<<max_x<<","<<max_y<<")"<<endl;
	cout<<"resolution is: "<<m_cell_size<<endl;

	// 2 calculate PMAP
	m_size_x = ((max_x - min_x + 0.05)/m_cell_size+1)*1;
	m_size_y = ((max_y - min_y + 0.05)/m_cell_size+1)*1;
	
	m_pPMAP->initializeMap(m_size_x,m_size_y,0,0,(int)min_x,(int)min_y,m_cell_size);
	cout<<"after initilizeMap!"<<endl;
	for(int i=0;i<m_obs_x.size();i++){
		m_pPMAP->updateMap(&m_obs_x[i][0],&m_obs_y[i][0],m_obs_x[i].size(),m_px[i],m_py[i],m_pth[i]);
	}
	m_pPMAP->computeProbsOfMap();
	cout<<"after compute PMAP!"<<endl;
	// 3 draw this PMAP
	cout<<"QImage: "<<m_size_x<<"*"<<m_size_y<<endl;
	m_pImg = new QImage(m_size_x,m_size_y,8,256);
	m_pPix = new QPixmap(m_size_x,m_size_y);
	for(int i=0;i<256;i++){
		m_pImg->setColor(i,qRgb(i,i,i));
	}
	m_pImg->setColor(0,qRgb(200,200,255));
	m_pImg->setColor(1,qRgb(255,0,0));
	cout<<"after set QPixmap!"<<endl;

	draw2DMap(m_pPMAP);
	m_pPix->convertFromImage(*m_pImg);
	drawTrajectory(m_pPMAP);
	m_pPix->save(out.c_str(),"PNG");
	cout<<"finish save PNG!"<<endl;
}


void CDrawMap::draw2DMap(CPMap* map)
{
	int maxx = map->m_size_x;
	int maxy = map->m_size_y;

	for(int x=0;x<maxx;x++){
		for(int y=0;y<maxy;y++){
			if(map->m_mapsum[x][y]>0){
				/*if(map->m_mapsum[x][y]>0.95){
				  m_pm->setPixel(x,y,(int)(1));
				  }else{*/
				m_pImg->setPixel(x,maxy-y-1,
						(int)(255-253*map->m_mapprob[x][y]));
			}else{  
				m_pImg->setPixel(x,maxy-y-1,0);
			}
			}
		}
}

void CDrawMap::drawTrajectory(CPMap* map){
	cout<<"start paint!"<<endl;
	QBrush mapbrush = QBrush(qRgb(0,0,255));
	QBrush nodebrush = QBrush(qRgb(255,0,0));
	QPen nodePen(nodebrush,4);
	QPainter imgPt;
	imgPt.begin(m_pPix);

	float x,y,th;
	int curr_x, curr_y;
	int last_x, last_y;
	// save ellipse vector
	vector<int> mx, my;
	
	int maxx = map->m_size_x;
	int maxy = map->m_size_y;

	for(int i=0;i<m_bIsRoot.size();i++){
		x = m_px[i];
		y = m_py[i];
		map->mapPosfromPoint(x,y,curr_x,curr_y);
		curr_y = maxy - 1 - curr_y;
		if(m_bIsRoot[i]) // draw mapNode
		{
			imgPt.setPen(qRgb(0,0,255));
			imgPt.setBrush(mapbrush);
			imgPt.drawEllipse(curr_x,curr_y,8,8);
			mx.push_back(curr_x);
			my.push_back(curr_y);
		}
		// imgPt.setPen(nodePen);
		// imgPt.drawPoint(curr_x,curr_y);
		if(i>0){
			imgPt.setPen(nodePen);
			imgPt.drawLine(last_x,last_y,curr_x,curr_y);
		}
		last_x = curr_x;
		last_y = curr_y;
	}
	cout<<"start drawImage!"<<endl;
	imgPt.end();
}

