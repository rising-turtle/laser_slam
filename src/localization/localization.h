#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <QObject>
#include <QImage>
#include <vector>
#include <fstream>
#include "globaldef.h"

using namespace std;
class OrientedPoint2D;
class CPolarMatch;
class CPMap;
class CVPmap;
class CParticles;
class CCanonicalMatcher;
struct _PMScan;

class threadLocalization : public QObject
{
	Q_OBJECT
public:
	threadLocalization();
	virtual ~threadLocalization();
	void randomPose(OrientedPoint2D&);
	void globalization(struct _PMScan*, OrientedPoint2D&);
	void topK(vector<int>&, vector<float>&, int k=g_num_of_particles);
	bool getCurrScan();
	bool runPFLocalization(OrientedPoint2D&);
	bool runScanMatch(OrientedPoint2D&);
public Q_SLOTS:
	bool receivePMAP(QImage*);
	void randomScan();
	void ranParticles();
	void globalize(int step = 5);
	void localize();
	void localize2();
	void localization1(int,int,float);
	void localization2(int,int,float);
	void enableSM();
	void simulateSM(int,int);
Q_SIGNALS:
	void updatePMAP(CPMap*);
	void send2Scan(int*, int*, int, int*, float);
	void send2Robot(int,int,float);
	void sendParticles(int*, int*, int);
	void sendParticle(int,int);
	void paintReady();
	void sendSimScan(float*,float*,int,double*,double*,double*);
	void sendRelScan(float*,float*,int,double*,double*,double*);
public:
	void setScanFile(string);
	ifstream * m_scanFile;
	CPolarMatch * m_pPSM;
	CCanonicalMatcher* m_pCSM;
	CVPmap* m_PMap;
	CParticles* m_Particles;
	unsigned int m_x_range;
	unsigned int m_y_range;
	bool m_bDrawLocalization;
	bool m_bPFLocalization;
	bool m_bScanMatchSim;
	struct _PMScan* m_cur_scan;
	struct _PMScan* m_sim_scan;
};


#endif
