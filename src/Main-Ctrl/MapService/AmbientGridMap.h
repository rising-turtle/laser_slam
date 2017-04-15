#ifndef AGM_H_
#define AGM_H_
#include "../MainCtrl_Define.h"
#include "pthread.h"
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <vector>

using namespace std;

typedef struct RawDataInfo
{
	char * pcData;
	int nDataLen;
	bool bRecv;
}RawDataInfo;


typedef struct ProbabilityGraphNode
{
	ProbabilityGraphNode **ppstNeighbour;
	int nBorderXMin;
	int nBorderXMax;
	int nBorderYMin;
	int nBorderYMax;
	float *pfProbabilityGraphData;
}ProbabilityGraphNode;


class AmbientGridMap
{
public:
	AmbientGridMap();
	~AmbientGridMap();

	int AGMInit();
	int AGMRun();
	int AGMStop();
	int AGMUninit();

	static char *m_pcRawDataPtr;
	static char *m_pcRawDataBuff1;
	static char *m_pcRawDataBuff2;
	static char m_cCurRawData;
	static pthread_t m_hThreadWriteRawData2File;
	static int Wrtie2RawDataBuff(vector<float> vctBearing);
	static int Wrtie2RawDataBuff(char *pcRawData,int nDatalen);
	static int WriteRawData2File(char *pcData,int nDatalen);
	static int m_nRawDataFileIdxW,m_nRawDataFileIdxR;
	static void* ThreadWriteRawData2File(void* lpParam);
	

	//FastProbabilityGraph Module
	int FastProbabilityGraphInit();
	int FastProbabilityGraphRun();
	int FastProbabilityGraphDataEntrance();
	ProbabilityGraphNode *m_pstProbabilityGraphBaseNode;


	static vector<vector<float> > m_vctBuff1,m_vctBuff2;

	static int nCurCount;


	static int SendStoreData2RS();
	static pthread_t m_hThreadSendStoreData2RS;
	static void* ThreadSendStoreData2RS(void* lpParam);

	static CallBack_NetUpload m_cbSend2RS_B;
};
#endif
