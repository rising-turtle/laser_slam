#include "FlirterNode.h"
#include "MatchingResult.h"
#include "ZHPolar_Match.h"
#include "ZHCanonical_Matcher.h"

#include <feature/Detector.h>
#include <feature/ShapeContext.h>
#include <feature/BetaGrid.h>
#include <feature/RangeDetector.h>
#include <feature/CurvatureDetector.h>
#include <feature/NormalBlobDetector.h>
#include <feature/NormalEdgeDetector.h>
#include <feature/RansacFeatureSetMatcher.h>
#include <feature/RansacMultiFeatureSetMatcher.h>
#include <sensorstream/CarmenLog.h>
#include <sensorstream/LogSensorStream.h>
#include <sensorstream/SensorStream.h>
#include <utils/SimpleMinMaxPeakFinder.h>
#include <utils/HistogramDistances.h>
#include <utility>

#ifdef TEST_PERFORMANCE
	#include <sys/time.h>
#endif 

using namespace g2o;
//using namespace g2o::tutorial;

const int g_match_threshold = 15;

bool CFliterNode::g_IsInit = false;
SimpleMinMaxPeakFinder* CFliterNode::g_peakMinMax = NULL;

// detectors
string CFliterNode::g_detector_name = "";
Detector* CFliterNode::g_detector = NULL;
CurvatureDetector * CFliterNode::g_detectorCurvature = NULL;
NormalBlobDetector * CFliterNode::g_detectorNormalBlob = NULL;
NormalEdgeDetector* CFliterNode::g_detectorNormalEdge = NULL;
RangeDetector* CFliterNode::g_detectorRange = NULL;

//distance
string CFliterNode::g_distance_name="";
HistogramDistance<double> * CFliterNode::g_dist = NULL;

// descriptor
string CFliterNode::g_descriptor_name = "";
ShapeContextGenerator* CFliterNode::g_shapeGenerator=NULL;
DescriptorGenerator* CFliterNode::g_descriptor =NULL;
BetaGridGenerator* CFliterNode::g_betaGenerator = NULL;

// match strategy
RansacFeatureSetMatcher* CFliterNode::g_ransac = NULL;

namespace{
#define SMALL_RELIABLE_VALUE 1e-6
#ifdef TEST_PERFORMANCE
	ofstream timelog("time_cost.log");
#endif
}



// This construct can only be called by CMapNode
CFliterNode::CFliterNode(std::string laser_name):m_pLaserRead(NULL),m_id(-1),m_pFMatch(new CPolarMatch(laser_name)), m_pScan(NULL), m_pFMatch2(0)
{
}
// just for adding a timestamp to this function
CFliterNode::CFliterNode(PMScan* pmscan, CPolarMatch* laser_wrapper, TTimeStamp t):m_pLaserRead(NULL),m_id(-1),m_pFMatch(laser_wrapper), m_pScan(NULL), m_timestamp(t)
{
	if(m_pFMatch == NULL){
		std::cerr<<"Laser Wrapper is NULL: "<<std::endl;
		throw 1;
	}
	m_pFMatch2 = new CCanonicalMatcher(m_pFMatch->m_pParam);
	if(!CFliterNode::g_IsInit)
	{
		InitFliter();
		g_IsInit = true;
	}
	if(pmscan == NULL){
		std::cerr<<"pmscan is NULL!"<<std::endl;
		throw 1;
	}
	// these actually a duplicated value for the laser scans
	m_pScan = new PMScan(*pmscan);
	m_pLaserRead = fromPM2LR(pmscan);
	if(m_pLaserRead == NULL){
		std::cout<<"lread is NULL!"<<std::endl;
		throw 1;
	}
	InitNode(m_pLaserRead);
}

// 2012.10.11this is where the bug exists! CPolarMatch(laser_name) maybe not compatible with this scan PMScan, thus, using specified CPolarMatch 
// but it is still dangerous when the outer laser_wrapper is changed!
CFliterNode::CFliterNode(PMScan* pmscan, CPolarMatch* laser_wrapper/*std::string laser_name*/):m_pLaserRead(NULL),m_id(-1),m_pFMatch(laser_wrapper), m_pScan(NULL)
{
	if(m_pFMatch == NULL){
		std::cerr<<"Laser Wrapper is NULL: "<<std::endl;
		throw 1;
	}
	m_pFMatch2 = new CCanonicalMatcher(m_pFMatch->m_pParam);
	// cout<<"m_pFMatch2: "<<m_pFMatch2<<endl;
	// cout<<"m_pParam: "<<m_pFMatch2->m_pParam<<endl;

	if(!CFliterNode::g_IsInit)
	{
		InitFliter();
		g_IsInit = true;
	}
	if(pmscan == NULL){
		std::cerr<<"pmscan is NULL!"<<std::endl;
		throw 1;
	}
	// these actually a duplicated value for the laser scans
	m_pScan = new PMScan(*pmscan);
	m_pLaserRead = fromPM2LR(pmscan);
	if(m_pLaserRead == NULL){
		std::cout<<"lread is NULL!"<<std::endl;
		throw 1;
	}
	InitNode(m_pLaserRead);	
}
// this constructor has not been realized
CFliterNode::CFliterNode(LaserReading* lread, std::string laser_name):m_pLaserRead(NULL),m_id(-1),m_pFMatch(new CPolarMatch(laser_name)),m_pScan(NULL)
{
	if(m_pFMatch == NULL){
		std::cerr<<"Wrong Laser: "<<laser_name<<std::endl;
		throw 1;
	}
	m_pFMatch2 = new CCanonicalMatcher(m_pFMatch->m_pParam);
	if(!CFliterNode::g_IsInit)
	{
		InitFliter();
		g_IsInit = true;
	}
	if(lread == NULL){
		std::cout<<"lread is NULL!"<<std::endl;
		throw 1;
	}
	// this is dangerous for deleting lread outside
	m_pLaserRead = new LaserReading(*lread);
	m_pScan = fromLR2PM(m_pLaserRead);
	if(m_pScan == NULL){
		std::cout<<"Scan is NULL!"<<std::endl;
		throw 1;
	}
	InitNode(m_pLaserRead);
}
CFliterNode::~CFliterNode(){
	UnitNode();
	// Here, if delete m_pFMatch or m_pFMatch2 
	// then, error will happen
	// because not overload the CFliterNode(CFliterNode&),
	// thus its member *m_pFMatch is not copied!
	/*if(m_pFMatch !=0) 
	{
		cout<<"before delete m_pFMatch!"<<endl;
		delete m_pFMatch;
		m_pFMatch = 0;
		cout<<"after delete m_pFMatch!"<<endl;
	}
	if(m_pFMatch2 !=0 )
	{
		delete m_pFMatch2;
		m_pFMatch2 = 0;
	}*/
}

void CFliterNode::InitNode(LaserReading* lread)
{
#ifdef TEST_PERFORMANCE
	struct timeval st,et;
	gettimeofday(&st,0);
#endif
	// detect feature points
	int n = g_detector->detect(*lread,m_featurePoints);
#ifdef TEST_PERFORMANCE
	gettimeofday(&et,0);
	timelog<<n<<" "<<((et.tv_sec - st.tv_sec)*1000+(et.tv_usec - st.tv_usec)/1000)<<" ";
#endif

	m_pose = lread->getLaserPose();
	m_gtpose = m_pose;
#ifdef TEST_PERFORMANCE
	gettimeofday(&st,0);
#endif
	// describe feature points
	for(int i=0;i<m_featurePoints.size();i++)
		m_featurePoints[i]->setDescriptor(g_descriptor->describe(*m_featurePoints[i],*lread));
#ifdef TEST_PERFORMANCE
	gettimeofday(&et,0);
	timelog<<((et.tv_sec - st.tv_sec)*1000+(et.tv_usec - st.tv_usec)/1000)<<endl;
#endif
	m_featurePointsLocal.resize(m_featurePoints.size(),NULL);
	// calculate local feature points
	for(int i=0;i<m_featurePoints.size();i++)
	{
		InterestPoint* plocal = new InterestPoint(*m_featurePoints[i]);
		plocal->setPosition(m_pose.ominus(plocal->getPosition()));
		m_featurePointsLocal[i] = plocal;
	}
	
	// init covariance and information 
	m_covariance.fill(0.);
	m_covariance(0,0) = SMALL_RELIABLE_VALUE;
	m_covariance(1,1) = SMALL_RELIABLE_VALUE;
	m_covariance(2,2) = SMALL_RELIABLE_VALUE;
	m_information = m_covariance.inverse();
	m_dis_mean_x = 0;
	m_dis_mean_y = 0;
	
	// init num for tcp syn
	m_psyn = 0;
	return ;
}
void CFliterNode::UnitNode()
{
	if(m_pScan!=NULL)
		delete m_pScan;
	if(m_pLaserRead!=NULL)
		delete m_pLaserRead;
	for(int i=0;i<m_featurePoints.size();i++){
		if(m_featurePoints[i]!=NULL)
		{
			delete m_featurePoints[i];
			m_featurePoints[i]=NULL;
		}
	}
	for(int i=0;i<m_featurePointsLocal.size();i++){
		if(m_featurePointsLocal[i]!=NULL)
		{
			delete m_featurePointsLocal[i];
			m_featurePointsLocal[i]=NULL;
		}
	}
}
void CFliterNode::fromSE22OrientedPoint(OrientedPoint2D& trans, EdgeSE2& ret){
	double d[3];
	ret.getMeasurementData(d);
	trans=OrientedPoint2D(d[0],d[1],d[2]);
	return ;
}
void CFliterNode::fromOrientedPoint2SE2(OrientedPoint2D& trans, EdgeSE2& ret){
	SE2 tmpSE(trans.x,trans.y,trans.theta);
	ret.setMeasurement(tmpSE);
	ret.setInverseMeasurement(tmpSE.inverse());
	return ;
}

// status = 1 : big angle
// status = 2 : big err
// status = 0 : right
bool CFliterNode::matchNodeFrontendGT(CFliterNode* pref, int& status){
	// threshods for angle change, dis change, error value
	static double odometry_thre = 0.6982; // almost 40 degree
	static double dis_thre = 10000; // 1m
	static float  err_thre = 0.1;//10; 
	static int cnt=0;
	cnt++;
	/*else
	{
		//cout<<"cur th: "<<this->m_gtpose.theta<<" last th: "<<pref->m_gtpose.theta<<endl;
		cout<<"cur: "<<this->m_gtpose<<"\t"<<"last: "<<pref->m_gtpose<<endl;	
	}*/
	static bool bfirst = true; 
	pair<float,float> err = m_pFMatch->FMatch(pref->m_pScan,m_pScan,bfirst); // for the first match
	if(bfirst) bfirst = false;
	
	// 1 if the translate angle is too big, then just use gt data
	if(fabs(this->m_gtpose.theta-pref->m_pose.theta/*pref->m_gtpose.theta*/) > odometry_thre){
		m_relpose = pref->m_gtpose.ominus(this->m_gtpose);
		m_pose = pref->m_pose.oplus(m_relpose);
		status = 1; // big angle
		// cout<<"odo: ("<<cnt<<","<<cnt-1<<") : "<<m_pose<<endl;
		return true;
	}

	if(err.first<0 && err.second<0){
		std::cout<<"Failed in FMatch!"<<std::endl;
		// return false;
	}
	// 2 if the registration error or translate dis is too big,
	
	bool err_f = ((err.first > err_thre) || (err.second > err_thre) || (err.first<0));
	bool dis_f = (((m_pScan->rx*m_pScan->rx)+(m_pScan->ry*m_pScan->ry)) > dis_thre);
	if(err_f || dis_f){
		m_relpose = pref->m_gtpose.ominus(this->m_gtpose);
		m_pose = pref->m_pose.oplus(m_relpose);
		status = 2; // big err
		return true;
	}	
	
	// 3 else use the result from PSM + ICP // From [cm] to [m]
	m_relpose = OrientedPoint2D((m_pScan->rx/100.0),(m_pScan->ry/100.0),m_pScan->th);
	m_pose = pref->m_pose.oplus(m_relpose);
	// cout<<"matched "<<cnt<<" : "<<m_pose<<endl;
	status = 0; // right match
	return true;
}

void CFliterNode::setCov(vector<double>& m){
	m_covariance(0,0) = m[0]; m_covariance(0,1) = m[1]; m_covariance(0,2) = m[2];
	m_covariance(1,0) = m[1]; m_covariance(1,1) = m[3]; m_covariance(1,2) = m[4];
	m_covariance(2,0) = m[2]; m_covariance(2,1) = m[4]; m_covariance(2,2) = m[5];
	m_information = m_covariance.inverse();
}

void CFliterNode::computeMarginal(CFliterNode* pre)
{
	// according to page.282 in Probabilistic Robotics
	// TODO: To compute Covariance_XY using Odometry function
	// Here to simplize, set Covariance_XY as I,
	
	/*
	m_information = m_information + pre->m_covariance;
	m_covariance = m_information.inverse();
	*/
	
	m_covariance = m_covariance + pre->m_covariance;
	m_information = m_covariance.inverse();
	
	m_dis_mean_x = sqrtf(m_covariance(0,0));
	m_dis_mean_y = sqrtf(m_covariance(1,1));
	// cout<<"dis_x: "<<m_dis_mean_x<<"; dis_y: "<<m_dis_mean_y<<endl;
}

namespace{
#define DIS_STEP_NUM 5
	int m_dis_step[DIS_STEP_NUM] = {1,2,4,8,16};
}
// 1 CSM succeed
// 0 CSM failed
int CFliterNode::matchNodeFrontend2(CFliterNode* pref)
{
	//cout<<"XXXXXXXXXXXXXXXXXXXXXX CSM"<<endl;
	static bool bfirst = true;
	float err = m_pFMatch2->FMatch(pref->m_pScan, m_pScan,bfirst);
	if(err == -1){
		cout<<"CSM failed! "<<endl;
		return 0;
	}
	if(bfirst) bfirst = false;
	// From [cm] to [m]
	m_relpose = OrientedPoint2D(m_pScan->rx/100.0,m_pScan->ry/100.0,m_pScan->th);
	m_pose = pref->m_pose.oplus(m_relpose);
	return 1;
}

// 1 PSM succeed
// 2 PSM failed ICP succeed
// 0 PSM and ICP failed
// match previous node using PSM or ICP
int CFliterNode::matchNodeFrontend(CFliterNode* pref)
{
	static bool bfirst = true;
	static pair<float,float> err_max = make_pair(0.4,0.4);
	static int index = 0;
	int ret = 0;
	pair<float,float> err = m_pFMatch->FMatch(pref->m_pScan,m_pScan,bfirst);
	if(bfirst) bfirst = false;
	if(err.first<0 && err.second<0){
		std::cout<<"Failed in FMatch!"<<std::endl;
		return 0;
	}
	// From [cm] to [m]
	m_relpose = OrientedPoint2D(m_pScan->rx/100.0,m_pScan->ry/100.0,m_pScan->th);
	m_pose = pref->m_pose.oplus(m_relpose);

	// Update marginal covariance
	if(err.first >0 && err.second > 0) // PSM succeed
	{
		m_covariance(0,0) = err.first;
		m_covariance(1,1) = err.second;
		if(index>0) index--;
		ret = 1;
		/*err_max.first = err_max.first < err.first? err.first : err_max.first;
		err_max.second = err_max.second < err.second? err.second : err_max.second;*/
	}else{	
		// PSM failed
		cout<<"lalalala! PSM failed!"<<endl;
		m_covariance(0,0) = err_max.first*m_dis_step[index];
		m_covariance(1,1) = err_max.second*m_dis_step[index];
		index++;
		if(index >= DIS_STEP_NUM) index--;
		ret = 2;
	}
	// m_information = m_covariance.inverse();
	computeMarginal(pref);
	return ret;
}

int CFliterNode::matchNodePair(CFliterNode* pref, MatchingResult& mr)
{
	OrientedPoint2D transform;
	pair<int,double> ret = matchNodePairLocal(pref,transform);
	
	// threshold for successfully matching
	static int match_threshold=35;
	static float match_ratio = 0.3;

	if(ret.first >= match_threshold && ret.second >= 0 ) // success to match
	{
		fromOrientedPoint2SE2(transform,mr.m_edge);
		//mr.m_edge.vertices()[0] = pref->m_id;
		//mr.m_edge.vertices()[1] = this->m_id;
		mr.id1 = pref->m_id;
		mr.id2 = this->m_id;
		Eigen::Matrix3d information;
		information.fill(0.);
		double inf = (1./ret.second);
		for(int i=0;i<3;i++)
			information(i,i) = inf;//match_num*match_num;
		//mr.m_edge.information() = Eigen::Matrix2d::Identity()*(match_num*match_num);
		mr.m_edge.setInformation(information);
	}
	else{
		mr.id1 = -1; // mr.m_edge.vertices()[0] = -1;
		mr.id2 = -1; //mr.m_edge.vertices()[1] = -1;
	}
	return ret.first;//mr;
}

pair<int,double> CFliterNode::matchNodePairLocal( CFliterNode* refNode, OrientedPoint2D& transform)
{
	pair<int,double> ret ;
	if(refNode ==NULL){
			return make_pair(0,-1);
	}	
	ret = matchFeaturePoints(refNode->m_featurePointsLocal,m_featurePointsLocal,transform);
	if(ret.first > 0 && ret.second >=0) // succeed
	{
		//this->m_pose = refNode->m_pose + (transform);
		this->m_relpose = transform;
	}
	else
	{
		this->m_relpose = refNode->m_pose.ominus(m_pose);
		transform = this->m_relpose;
		return make_pair(0,-1);
	}
	return ret;
}
pair<int,double> CFliterNode::matchNodePairGlobal( CFliterNode* refNode, OrientedPoint2D& transform)
{
	if(refNode == NULL)
		return make_pair(0,-1);
	pair<int,double> ret= matchFeaturePoints(refNode->m_featurePoints,m_featurePointsLocal,transform);
	if(ret.first >0 && ret.second >=0) // successfully matched
	{	
		//this->m_pose = transform;
		this->m_relpose = transform - refNode->m_pose;
	}
	else	// failed to match
	{
		this->m_relpose = m_pose - refNode->m_pose;
	}
	//this->m_relpose = m_pose - (refNode->m_pose);
	transform = this->m_relpose;
	return ret;
}

pair<int,double> CFliterNode::matchFeaturePoints(std::vector<InterestPoint*>& fpref,std::vector<InterestPoint*>& fpcur,
								OrientedPoint2D& transform)
{
	std::vector< std::pair<InterestPoint*, InterestPoint* > > correspondences;
	double result;
	result=g_ransac->matchSets(fpref, fpcur, transform, correspondences);

	if(correspondences.size()<2)
	{
		//cout<<"correspondences are too few!"<<endl;
		return make_pair(0,-1);
	}
	if(result >=1e17){
		cout<<"result error is too big!"<<endl;
		return make_pair(0,-1);
	}
	// return (int)(correspondences.size());
	return make_pair((correspondences.size()),result);
}


void CFliterNode::resetInit(){
	CFliterNode::g_IsInit =false;
}

void CFliterNode::InitFliter(){
	CFliterNode::g_peakMinMax = new SimpleMinMaxPeakFinder(getParameter().minPeak, getParameter().minPeakDistance);
	// set detectorType
	switch(getParameter().detectorType){
		case 0:
			g_detectorCurvature = new CurvatureDetector(g_peakMinMax,getParameter().scale, 
				getParameter().baseSigma,getParameter().sigmaStep,getParameter().dmst);
			g_detectorCurvature->setUseMaxRange(getParameter().useMaxRange);
			g_detector = g_detectorCurvature;
			g_detector_name = "curvature";
			break;
		case 1:
			g_detectorNormalEdge = new NormalEdgeDetector(g_peakMinMax,getParameter().scale,getParameter().baseSigma, 
				getParameter().sigmaStep,getParameter().window);
			g_detectorNormalEdge->setUseMaxRange(getParameter().useMaxRange);
			g_detector = g_detectorNormalEdge;
			g_detector_name = "edge";
			break;
		case 2:
			g_detectorNormalBlob = new NormalBlobDetector(g_peakMinMax,getParameter().scale,getParameter().baseSigma, 
				getParameter().sigmaStep,getParameter().window);
			g_detectorNormalBlob->setUseMaxRange(getParameter().useMaxRange);
			g_detector = g_detectorNormalBlob;
			g_detector_name = "blob";
			break;
		case 3:
			g_detectorRange = new RangeDetector(g_peakMinMax,getParameter().scale,getParameter().baseSigma,getParameter().sigmaStep);
			g_detectorRange->setUseMaxRange(getParameter().useMaxRange);
			g_detector = g_detectorRange;
			g_detector_name = "range";
			break;
		default:
			cerr<<"wrong detecor type!"<<endl;
			exit(-1);
	}
	// set distance types
	switch(getParameter().distanceType){
	case 0:
		g_dist =  new EuclideanDistance<double>(); 
		g_distance_name = "euclid";
		break;
	case 1:
		g_dist = new Chi2Distance<double>();
		g_distance_name = "chi2";
		break;
	case 2:
		g_dist = new SymmetricChi2Distance<double>();
		g_distance_name = "symchi2";
		break;
	case 3:
		g_dist = new BatthacharyyaDistance<double>();
		g_distance_name = "batt";
		break;
	case 4:
		g_dist = new KullbackLeiblerDistance<double>();
		g_distance_name = "kld";
		break;
	case 5:
		g_dist = new JensenShannonDistance<double>();
		g_distance_name = "jsd";
		break;
	default:
		cerr<<"wrong distance type!"<<endl;
		exit(-1);
	}
	// set descriptors 
	switch(getParameter().descriptorType){
	case 0:
		g_betaGenerator =  new BetaGridGenerator(0.02, 0.5, 4, 12);
		g_betaGenerator->setDistanceFunction(g_dist);
		g_descriptor = g_betaGenerator;
		g_descriptor_name = "beta";
		break;
	case 1:
		g_shapeGenerator = new  ShapeContextGenerator(0.02, 0.5, 4, 12);
		g_shapeGenerator->setDistanceFunction(g_dist);
		g_descriptor = g_shapeGenerator;
		g_descriptor_name = "shape";
		break;
	default:
		cerr<<"wrong descriptor type!"<<endl;
		exit(-1);
	}
	// set match strategy
	switch(getParameter().matchStrategy){
		case 0:
			g_ransac = new RansacFeatureSetMatcher(getParameter().acceptanceSigma * getParameter().acceptanceSigma * 5.99, 
				getParameter().success, getParameter().inlier,  getParameter().matchingThreshold, getParameter().acceptanceSigma * getParameter().acceptanceSigma * 3.84, false);
			break;
		case 1:
			g_ransac = new RansacMultiFeatureSetMatcher(getParameter().acceptanceSigma * getParameter().acceptanceSigma * 5.99, 
				getParameter().success, getParameter().inlier,  getParameter().matchingThreshold, getParameter().acceptanceSigma * getParameter().acceptanceSigma * 3.84, false);
			break;
		default:
			cerr<<"wrong match strategy!"<<endl;
			exit(-1);
	}
}
CFliterNode::_Parameters::_Parameters(){
	scale = 5; /*5*/ 
	dmst = 2; 
	window = 3; 
	detectorType = 2; // 0 Curature 1 Edge Detector 2 Blob 3 Range
	descriptorType = 0; 
	distanceType = 2; 
	matchStrategy = 0;
	baseSigma = 0.2; 
	sigmaStep = 1.4; 
	minPeak = 0.34; 
	minPeakDistance = 0.001; 
	acceptanceSigma = 0.1; 
	success = 0.95; 
	inlier = 0.4; 
	matchingThreshold = 0.4; // 
	useMaxRange = false;
}

CFliterNode::FliterParameters& CFliterNode::getParameter(){
	static CFliterNode::FliterParameters g_singleParamter;
	return g_singleParamter;
}

PMScan* CFliterNode::fromLR2PM(LaserReading* lread){
	std::cout<<"Oops, This function has not been defined!"<<std::endl;
	return NULL;
}

LaserReading* CFliterNode::fromPM2LR(PMScan* pmscan)
{
	if(pmscan==NULL){
		std::cout<<"pmscan is NULL"<<std::endl;
		return NULL;
	}
	if(pmscan->np != m_pFMatch->m_pParam->pm_l_points){
		std::cout<<"error: this scan is not compatible with Laser: "<<m_pFMatch->m_pParam->pm_laser_name<<std::endl;
		return NULL;
	}
	std::vector<double> rho(pmscan->np); // range value
	std::vector<double> phi(pmscan->np); // angle value
	std::vector<double> remission; 

	std::string sensorName(m_pFMatch->m_pParam->pm_laser_name);
	std::string robotName("");
	double timestamp = pmscan->t;
	for(int i=0;i<rho.size();i++)
	{
		rho[i] = pmscan->r[i]/100.0; // [cm] 2 [m]
		// NOTICE: FLIRT use [m] while PSM use [cm]
		phi[i] = m_pFMatch->pm_fi[i];
	}

	LaserReading * ret = new LaserReading(phi,rho,timestamp,sensorName,robotName);
	ret->setMaxRange(m_pFMatch->m_pParam->pm_max_range);
	// This maybe a BUG!
	ret->setLaserPose(OrientedPoint2D(pmscan->rx/100.,pmscan->ry/100.,pmscan->th));
	return ret;
}


