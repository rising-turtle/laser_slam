#include "testState.h"
#include "PolarParameter.h"
#include "ZHPolar_Match.h"
#include "string.h"
#include <string>
#include <cmath>
#include <vector>
#include <iostream>
#include <fstream>

double segment_Score3(PMScan& ls, vector<float>& f, vector<int>& segs, int seg, int& index, int shift, int&, int&);
int segment_Scan2(vector<float>&, vector<int>&);
double segment_Score2(vector<float>& f, vector<int>& segs, int seg, int& index, int shift);

double seg_filtr_score(vector<float>& f, int& st, int &et );
int segment_Scan(vector<float>&, vector<int>&);
double segment_Score(PMScan&, vector<float>& f, vector<int>& segs, int seg, int& index, int shift);
void record(vector<float>& , vector<int>&);

string g_scan_file("/home/lyxp/work/mkproj/SVN/PFG/trunk/src/localization/work_circles_2.txt");

// string inifile("/home/lyxp/work/mkproj/SVN/PFG/trunk/src/localization/work_circles_2.txt");

// bool constructPSMfromRawSeed(char*, PMScan&, CPolarMatch*);
// int frontObject(PMScan&, float, float, float& );
/*
int main(){
	CPolarMatch * m_pPSM = new CPolarMatch("LMS151");
	PMScan ls(541);
	ifstream inf(inifile.c_str());
	if(!inf.is_open()){
		cout<<"in runCarmon() failed to open file: "<<inifile<<endl;
		return -1;
	}
	char line[8192];
	int cnt = 0;

	float stop_dis = 0.5;
	float slow_dis = 1.5;
	float angle; 
	int step = 0;
	while(!inf.eof() && inf.getline(line,8192))
	{
		// if(!constructPSMfromCarmon(line,ls,m_pPSM))
		if(!constructPSMfromRawSeed(line,ls,m_pPSM)){
			cout<<"error!"<<endl;
		}
		frontObject(ls, stop_dis, slow_dis, angle);
		if(++step > 1000)
			break;
	}
}
*/

// RIGHT -> LEFT
#define LEFT_OCC 60
#define RIGHT_OCC 40
#define ANGLE_THRE 10 
#define MIN_DIST 1 // 
#define ROBOT_LEN 0.8
#ifndef PM_R2D
#define PM_R2D (180./M_PI)
#define PM_D2R (M_PI/180.)
#endif
/*
* [start - end) [input] check area :[180-360)
* [RIGHT_OCC LEFT_OCC] occlusion area
* slow_dis [input] occlusion dis to slow : 3 m,
* stop_dis [input] occlusion dis to stop : 1.5 m, 
* win_range [input] search width range to forward : 5 almost 5'
* angle [output] angle to rotate [left +, right -]
* Return : 
* 	-1 error happened
*	0 speed up
*	1 stop -> turn angle
*	2 stop -> random angle
*	3 slow -> turn angle
* 	4 slow -> not turn angle
*/ 
int Look4Window2(PMScan& ls, float& angle, int start, int end, float slow_dis, float stop_dis)
{
	int N = end - start;
	int ret_Status = 0;
	cout<<"Look4Window2: N: "<<N<<endl;
	vector<float> frontpts(N,0);
	cout<<"after frontpts!"<<endl;
	vector<float> weights(N,0);
	vector<float> bweights(N,0);
#define MAX_R 10000
#define MIN_R 0.0001
	float min_r = MAX_R;
	float max_r = MIN_R;
	float c_r;

	int single_d = N/2; // current robot 's orientation
	static bool once = true;
	if(once){
		cout<<"start: "<<start<<" end: "<<end<<" single_d: "<<single_d<<endl;
		cout<<"right: "<<single_d-RIGHT_OCC<<" left: "<<single_d+LEFT_OCC<<endl;
		cout<<"stop_dis: "<<stop_dis<<endl;;
		once = false;
	}
	for(int i=0;i<N;i++){
		// c_r = m_stSickARange[i+start].fDis;
		c_r = ls.r[i+start]/100.;
		if(c_r == 0) continue;
		if( single_d - RIGHT_OCC < i && i < single_d + LEFT_OCC )
		{
			if(c_r <= stop_dis)
			{
				ret_Status = 1; // stop 
				//cout<<"actually need stop!"<<endl;
			}
			else if(ret_Status !=1 && c_r <= slow_dis) 
			{	
				ret_Status =3; // slow
			}
		}
		frontpts[i] = c_r;
		if(c_r > max_r) max_r = c_r;
		if(c_r < min_r) min_r = c_r;
	}
	if(min_r == MAX_R || max_r == MIN_R){
		cout<<"IOA.cpp: this scan is not valid!!"<<endl;
		return -1;
	}

	vector<int> segs;
	int n_of_seg = segment_Scan2(frontpts,segs); // segment the scan
	record(frontpts, segs);
	// int n_of_seg = segment_Scan(frontpts,segs); // segment the scan

	cout<<"testState.cpp: number of segs: "<<n_of_seg<<endl;

	// vector<float> score(n_of_seg);
	vector<int> index(n_of_seg+1,0);
	vector<float> score(n_of_seg+1,0);
	vector<int> win_st(n_of_seg+1,0);
	vector<int> win_et(n_of_seg+1,0);
	float max_score = MIN_R;
	// float score;
	int ret_index = 0;
	for(int i=1;i<=n_of_seg;i++)
	{
		// score = segment_Score2(frontpts, segs, i, index[i], start);
		score[i] = segment_Score3(ls, frontpts, segs, i, index[i], start, win_st[i], win_et[i]);
		//  score = 0;
		// score = segment_Score(ls, frontpts, segs, i, index[i], start);
	
		/*score = segment_Score(frontpts,segs,i,index[i], start);*/
		if(score[i] > max_score) 
		{
			max_score = score[i];
			ret_index = index[i];
		}
	}
	/*if(max_score < 1.5){
		cout<<"testState.cpp: No available area! ";
		cout<<"Max_score: "<<max_score<<endl;
		return 1;
	}*/
	cout<<"testState.cpp: max_score: "<<max_score<<endl;
	if(max_score == MIN_R || max_score < ROBOT_LEN){
		cout<<"testState.cpp: No available area! "<<endl;
		// cout<<"ret_index: "<<ret_index<<" "<<endl;
		return 2;
	}
	
	if(ret_index == 0){
		cout<<"IOA.cpp: No nice segment!"<<endl;
		return 2;
	}

	int min_change_angle = MAX_R;
	for(int i=1; i<=n_of_seg; i++)
	{
		int change = abs(index[i] - single_d);
		if(change < min_change_angle && score[i] >= ROBOT_LEN)
		{
			min_change_angle = change;
			ret_index = index[i];
		}
	}

	for(int i=1; i<=n_of_seg;i++)
	{
		if(i==ret_index){
			printf("best: {%d, %d} score: %f\n", win_st[i], win_et[i], score[i]);
		}else{
			printf("option: [%d, %d] score: %f\n", win_st[i], win_et[i], score[i]);
		}
	}
	/*// calculate weights for each laser line
	for(int i=0; i< N; i++){
		weights[i] = (frontpts[i] - min_r)/(max_r - min_r);
	}
	int size_w = win_range;
	float norm = (1./((2*size_w)+1));
	int ret_index = 0; // the max weight orientation
	float max_w = 0;  
	for(int i=size_w; i < N-size_w-1; i++){
		float w = 0;
		for(int j=i-size_w; j<=i+size_w; j++){
			w+=weights[i];
		}
		w*=norm;
		if(w>max_w) {max_w = w; ret_index = i;}
	}
	*/
	// find the angle to rotate
	int angle_change = ret_index - single_d;
	angle = PM_D2R*((float)(angle_change)*0.5); 
	
	if( ret_Status !=0 && (angle_change > -ANGLE_THRE && angle_change < ANGLE_THRE ))
        {
		cout<<"small angle "<<PM_R2D*angle<<", no need rotation!"<<endl;
		return ++ret_Status;
	}
	cout<<"index: "<< ret_index <<" rotation : "<<PM_R2D*angle<<endl;
	return ret_Status;
}

void record(vector<float>& f, vector<int>& segs)
{
	ofstream rec("tmp.log");
	for(int i=0;i<f.size();i++)
	rec<<f[i] << " "<<segs[i]<<endl;
}

int segment_Scan2(vector<float>& f, vector<int>& segs)
{
	segs.clear();
	segs.resize(f.size(),0);
	const float MAX_DIST = MIN_DIST;//max range diff between conseq. points in a seg 
	double   dr;
	int       seg_cnt = 0;
	int       i,cnt;
	bool      break_seg;
	if(f.size()<2)
	{
		cout<<"IOA.cpp error in segment_Scan!"<<endl;
		return -1;
	}

	seg_cnt = 1;
	
	cnt = 0;
	for(int i=0;i<f.size();i++){
		if(f[i] > MAX_DIST || fabs(f[i] - MAX_DIST) < 0.2){
			segs[i] = seg_cnt;
			cnt++;
		}else{
			// seg_cnt ++ ;
			segs[i] = 0;
			if(i>1 && cnt <=1 ){
				segs[i-1] = 0;
			}else if(cnt > 1){
				seg_cnt ++;
			}
			cnt = 0 ;
		}
	}
	
	return seg_cnt;
}

double segment_Score3(PMScan& ls, vector<float>& f, vector<int>& segs, int seg, int& index, int shift, int& vst, int& vet)
{
	double sum = 0;
	int cnt = 0;
	int start = 0;
	int end ;
	bool bfirst = true;
	bool found = false;

	int ac_st = 0;
	int ac_et = f.size()-1;
	int i=0;
	while(f[i]==0){ i++; ac_st = i; }
	i=f.size()-1;
	while(f[i]==0){ i--; ac_et = i; }
	
	start = ac_st;
	end = ac_et;

	for(int i=ac_st;i<=ac_et;i++)
	{
		if(segs[i] == seg)
		{
			found = true;
			if(bfirst) 
			{
				bfirst = false;
				start = i;
			}
			end = i;
			++cnt;
			// sum+=f[i];	
		}else if(segs[i] != seg && !bfirst)
		{
			break;
		}		
	}
	
	if(cnt <= 10 || !found)  return 0;

	int st, et;
	if(start == ac_st || end == ac_et)
	{
		st = start;
		et = end;
	}else{
		st = start - 1;
		et = end + 1;
		while(st > ac_st && (f[st] == 0 || f[st] > MIN_DIST))
			st -- ;
		while(et < ac_et && (f[et] == 0 || f[et] > MIN_DIST))
			et++;
		/*if(f[st] > MIN_DIST || f[et] > MIN_DIST){
			cout<<"testState.cpp: must have error!"<<endl;
		}*/
	}
	
	// angle displacement
	index = st+(et-st)/2;
	cout<<"st: "<<st<<" et: "<<et<<" index: "<<index<<endl;
	
	const double delta_r = 0.19; // 20cm
	const double delta_l = 0.44; // 44cm
	if(f[et] == 0 ) --et;
	if(f[st] == 0 ) ++st;
	double r = f[st];
	double l = f[et];
	double theta_r, theta_l;
	if(delta_r >= r)
		theta_r = M_PI/2;
	else 
		theta_r = asin(delta_r/r);
	if(delta_l >= l) 
		theta_l = M_PI/2;
	else 
		theta_l = asin(delta_l/l);

	int shift_r = (theta_r/M_PI)*360;
	int shift_l = (theta_l/M_PI)*360;
	// printf("r=%f, l=%f, tr=%f, tl=%f, sr=%f, sl=%f\n",r,l,theta_r,theta_l,shift_r,shift_l);
	vst = st + shift_r + 2;
	vet = et - shift_l - 2;	
	// cout<<"testState.cpp: valid window "<<vst<<" : "<<vet<<endl;
	if(vet < vst)
	{
		cout<<"testState.cpp Not valid window!"<<endl;
		return 0;
	}
	if(et - st > 180)
	{
		return 2.0;
	}
	double tangle = sin(PM_D2R*(et-st)*0.5*0.5);
	if(tangle < 0) tangle *=-1.;
	double line = f[st] < f[et] ? f[st] : f[et];
	double score = line*tangle*2;

	// double score = f[st]*tangle + f[et]*tangle;
	double x1,y1,x2,y2;
	x1 = ls.x[shift+st]/100.;  y1 = ls.y[shift+st]/100.;
	x2 = ls.x[shift+et]/100.;  y2 = ls.y[shift+et]/100.;
	cout<<"p1: " <<f[st]<<" "<<x1<<","<<y1<<" p2: "<<f[et]<<" "<<x2<<","<<y2<<" ";
	// double score = sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
	printf("tan: %f, line: %f, score: %f\n",tangle,line,score);
	// cout<<"score: "<<score<<endl;

	return score;
}

double segment_Score2(vector<float>& f, vector<int>& segs, int seg, int& index, int shift)
{
	double sum = 0;
	int cnt = 0;
	int start = 0;
	int end = f.size()-1;
	bool bfirst = true;
	for(int i=0;i<f.size();i++){
		if(segs[i] == seg){
			if(bfirst) {
				bfirst = false;
				start = i;
			}
			end = i;
			++cnt;
			sum+=f[i];	
		}else if(segs[i] > seg && !bfirst){
			break;
		}		
	}
	
	// angle displacement
	index = start+(end-start)/2;
	double tangle = tan(PM_D2R*(end-start)*0.5*0.5);
	if(cnt<3) {return 0;}
	
	double dis = sum/cnt;
	return (dis*2.*tangle);
}

int segment_Scan(vector<float>& f, vector<int>& segs)
{
	segs.clear();
	segs.resize(f.size(),0);
	const float   MAX_DIST = 0.2;//max range diff between conseq. points in a seg 
	double   dr;
	int       seg_cnt = 0;
	int       i,cnt;
	bool      break_seg;
	if(f.size()<2){
		cout<<"IOA.cpp error in segment_Scan!"<<endl;
		return -1;
	}

	seg_cnt = 1;

	//init:
	if ( fabsf ( f[0]-f[1] ) < MAX_DIST ) //are they in the same segment?
	{
		segs[0] = seg_cnt;
		segs[1] = seg_cnt;
		cnt        = 2;    //2 points in the segment
	}
	else
	{
		segs[0] = 0; //point is a segment in itself
		segs[1] = seg_cnt;
		cnt        = 1;
	}
	for(i=2;i<f.size();i++){
		break_seg = false;
		if(f[i] == 0){
			break_seg = true;
			segs[i] = 0;
		}else{
			 // dr = f[i] - (2.0*f[i-1] - f[i-2]); 
			/*if(fabs(f[i]-f[i-1]) < MAX_DIST || \
				((segs[i-1] == segs[i-2]) && fabsf(dr) < MAX_DIST))*/
				if(fabs(f[i]-f[i-1]) < MAX_DIST)
				{
					cnt++; 
					segs[i] = seg_cnt;
				}else{
					break_seg = true;
				}
		}

		if(break_seg){
			if(cnt==1){
				dr = f[i] - (2.0*f[i-1] - f[i-2]);
				if(segs[i-2] ==0 && f[i]!=0 && f[i-1] != 0 \
				&& f[i-2] !=0 && fabsf(dr) < MAX_DIST)
				{
					segs[i] = seg_cnt;
					segs[i] = seg_cnt;
					segs[i] = seg_cnt;
					cnt = 3;
				}else{
					segs[i-1] = 0;
					segs[i] = seg_cnt;
					cnt =1;
				}
			}else{
				++seg_cnt; 
				segs[i] = seg_cnt;
				cnt = 1;
			}
		}
	}
	return seg_cnt;
}

double seg_filtr_score(vector<float>& f, int& st, int &et )
{
	double max_sum = 0;
	int max_cnt = 0;
	double sum = 0;
	int start = -1; 
	int end = f.size()-1;
	int cnt = 0;
	for(int i=0;i<f.size();i++){
		if(f[i] > 3.0){
			if(start < 0)
				start = i;
			sum += f[i];
			cnt++;
		}else
		{
			if(start >= 0 && cnt >= 3)
			{
				end = i;
				if(sum > max_sum){
					st = start;
					et = end;
					max_sum = sum;
					max_cnt = cnt;
				}
				// cout<<"max_cnt: "<<max_cnt<<" max_sum: "<<max_sum<<endl;
			}
			start = -1;
			cnt = 0;
			sum = 0;
		}
	}
	if(cnt >= 0 )
	{
		if(sum > max_sum){
			st = start;
			et = end;
			max_sum = sum;
			max_cnt = cnt;
		}
	}
	// cout<<"max_cnt: "<<max_cnt<<" max_sum: "<<max_sum<<endl;

	if(max_cnt > 0)
	{
		max_sum/=max_cnt;
		return max_sum;
	}
	return 0;
}

double segment_Score(PMScan& ls, vector<float>& f, vector<int>& segs, int seg, int& index, int shift)
{
	double max_sum = 0;
	double sum = 0;
	int cnt = 0;
	int start = 0;
	int end = f.size()-1;
	bool bfirst = true;

	vector<float> tmpf;

	for(int i=0;i<f.size();i++){
		if(segs[i] == seg){
			tmpf.push_back(f[i]);
			if(bfirst) {
				bfirst = false;
				start = i;
			}
			end = i;
			++cnt;
			// sum+=f[i];	
		}else if(segs[i] > seg && !bfirst){
			break;
		}		
	}
	if(cnt < 5) {return 0;}
	int st,et;
	sum = seg_filtr_score(tmpf, st, et);
	// cout<<"testState.cpp: sum: "<<sum<<" st: "<<st<<" et: "<<et<<endl;
	if(sum == 0) return 0;

	// distance to robot 
	// sum /= cnt;
	// if(sum < 2) return 0; 
	// length of the segment
	/*float sx = ls.x[start+shift]/100.;
	float sy = ls.y[start+shift]/100.;
	float ex = ls.x[end+shift]/100.;
	float ey = ls.y[end+shift]/100.;
	double length = sqrt((ex-sx)*(ex-sx) + (ey-sy)*(ey-sy));*/
	
	double win_len = tan((et-st)*0.5*0.5)*2;
	if(win_len < 0) win_len*=-1.;
	win_len = sqrt(win_len);

	// angle displacement
	int middle = f.size()/2;
	end = start + et;
	start += st;
	index = start+(end-start)/2;
	double angle_dis = cos(PM_D2R*(index - middle)*0.5);
	double ret =  (sum*sum*win_len*angle_dis);
	// cout<<"testState.cpp: segment: "<<seg<<" dis: "<<sum<<"angle_dis: "<<angle_dis<<"win_len: "<<win_len<<" score: "<<ret<<endl;
	return sum*angle_dis;
}


// 1 stop 
// 2 continue
// 3 rotate
int frontObject(PMScan& ls, float stop_dis, float slow_dis, float& angle)
{
	int start = 180;
	int end = 360;
	int N = 180;
	vector<float> frontpts(N,0);
	vector<float> weights(N,0);
	vector<float> bweights(N,0);
	float min_r = 10000;
	float max_r = 0.0001;
	float c_r ;

	int ret_Status = 0; // 1: stop, 2: slow
	int single_d = N/2;
	int slow_win = 20;

	ofstream pts("pts.log");
	ofstream wei("weights.log");

	for(int i=0;i<N;i++)
	{
		c_r = ls.r[i+start]/100.;
		if(c_r == 0) continue;

		if(c_r <= stop_dis) {
			cout<<"must stop!"<<endl;
			return 1;
		}else if(c_r <= slow_dis && (single_d - slow_win < i && i < single_d + slow_win))
		{
			// cout<<"must slow down!"<<endl;
			ret_Status = 3;
		}
		frontpts[i] = c_r;
		if(c_r > max_r) max_r = c_r;
		if(c_r < min_r) min_r = c_r;
	}
	for(int i=0;i<N;i++){
		weights[i] = (frontpts[i] - min_r)/(max_r - min_r);
		// wei<<weights[i]<<" "<<endl;
		// pts<<frontpts[i]<<" "<<endl;
	}

	int size_w = 3;
	float norm = (1./(2*size_w));
	int ret_index = 0;
	float max_w = 0;

	for(int i=size_w;i<N-size_w-1;i++)
	{	float w = 0;
		for(int j=i-size_w; j<i+size_w; j++){
			w+=weights[j];
		}
		w *= norm;
		if(w>max_w) {max_w = w; ret_index = i;}
	}

	if(ret_index == 0){
		cout<<"nothing happened!, sth error!"<<endl;
		return 2;
	}
	
	// find the angle to rotate
	static float max_angle = PM_D2R*(45);
	static float min_angle = -max_angle;
	static float angle_diff= PM_D2R*0.5;
	static float angle_thre = PM_D2R*5;
	angle = min_angle + ret_index*angle_diff;
	if( ret_Status !=3 && fabs(angle) <= angle_thre  )
	{
		cout<<"small angle "<<PM_R2D*angle<<", no need rotation!"<<endl;
		return 2;
	}
	// cout<<"index: "<< ret_index <<" rotation : "<<PM_R2D*angle<<endl;
	return 3;
}

bool constructPSMfromRawSeed(char *line, PMScan& ls, CPolarMatch* m_pPSM)
{
	double timestamp; //must be double
	int num_points;
	int offset;
	char* ptr;
	string delim(" ,");

	timestamp = atof(strtok_r(line,delim.c_str(),&ptr));
	num_points = atoi(strtok_r(NULL,delim.c_str(),&ptr));
	offset = atoi(strtok_r(NULL,delim.c_str(),&ptr));

	// printf("CClientFrontend::constructPSMfromRawSeed: t=%.7f, pm_l_points=%d, offset=%d \n", timestamp,num_points,offset);

	if(m_pPSM->m_pParam->pm_l_points!=num_points){

		if(m_pPSM->m_pParam->pm_l_points==num_points + 1)
		{}
		else
		{
			cout<<"num_points: "<<num_points<<" PARAM: "<<m_pPSM->m_pParam->pm_l_points<<endl;
			cout<<"not enough points in file!"<<endl;
			return false;
		}
	}
	ls.rx=ls.ry=ls.th=0; // its position must be 0,0,0
	ls.t = timestamp;

	char* p;

	for(int i=0;i<num_points;i++){
		p = strtok_r(NULL,delim.c_str(),&ptr);
		if(!p) return false;
		ls.r[i] = atof(p)*100.;
		// ls.r[i] = atof(strtok_r(NULL,delim.c_str(),&ptr))*100.0; // from [m] 2 [cm]
		ls.x[i] = ls.r[i] * m_pPSM->pm_co[i];
		ls.y[i] = ls.r[i] * m_pPSM->pm_si[i];
		ls.bad[i] = 0;
		if(ls.r[i] <= m_pPSM->m_pParam->pm_min_range){
			ls.r[i] = 0; //m_pPSM->m_pParam->pm_max_range+1; //set it to a value larger than the max
			ls.bad[i] = 1;
		}
	}
	if(m_pPSM->m_pParam->pm_l_points==num_points + 1)
	{
		ls.r[num_points] =  m_pPSM->m_pParam->pm_max_range+1;
		ls.x[num_points] = 0;
		ls.y[num_points] = 0;
		ls.bad[num_points] = 1;
	}

	return true;
}


