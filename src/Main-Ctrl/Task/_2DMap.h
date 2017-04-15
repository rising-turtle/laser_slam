#pragma once

#include "../MainCtrl_Define.h"
#include	<pthread.h>
#include "BMPLib.h"
//Params for Real time map for IOA
#define RLM_WIDTH 10
#define RLM_HEIGHT 100
#define MAP_COPY_NUM 5

typedef struct _2DGridMapData
{
	int n2DGridMapW;
	int n2DGridMapH;
	int n2DGridMapIdx;
	int n2DGridMapDataLen;
	unsigned char *puc2DGridMapData;

	float fResolution;
	float fOriginInGlobalX;
	float fOriginInGlobalY;
}_2DGridMapData;



class _2DMap
{
public:
	_2DMap(void);
	~_2DMap(void);

	int _2DMapInit();

	static _2DGridMapData *m_pst2DGridMapData;
	static int AddNew2DGridMap(char *pcData);
	static int ShowPathInBMP(_2DGridMapData *pst2DGridMapData);

	/*static int AddNew2DGridMap(char *pcData);
	static int Update2DGridMap(char *pcData);

	static _2DGridMapData * GetAMapCopy();
	static int DestroyMapCopy();

	int MarkObstacleOnMapCopy(const IOA_Pos *pstSICKView,const Pos_f *pstCurPos,_2DGridMapData * pstMapCopy);
	CallBack_LogFile m_cbLogFile;

private:

#define MAX_MAP_NUM 32
	char m_cOccupyFlag[MAX_MAP_NUM];
	static _2DGridMapData *m_pst2DGridMapData;

	//static _2DGridMapData * m_pstMapCopy;
	bool m_bLockMapCopy;

//IOA local map
	float m_fLocalMapA[RLM_WIDTH*RLM_HEIGHT];
	float m_fLocalMapB[RLM_WIDTH*RLM_HEIGHT];

	pthread_mutex_t m_MapCopyMutex;*/
};
