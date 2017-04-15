#include "_2DMap.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
_2DGridMapData *_2DMap::m_pst2DGridMapData;
//_2DGridMapData * _2DMap::m_pstMapCopy;
_2DMap::_2DMap(void)
{
//	pthread_mutex_init(&m_MapCopyMutex,0);
}

_2DMap::~_2DMap(void)
{
}
/*
int _2DMap::MarkObstacleOnMapCopy(const IOA_Pos *pstSICKView,
				const Pos_f *pstCurPos,_2DGridMapData * pstMapCopy)
{
	int i,nCurPosInMapX,nCurPosInMapY,nX,nY;
	nCurPosInMapX=(pstCurPos->fX-pstMapCopy->fOriginInGlobalX)*10;
	nCurPosInMapY=(pstCurPos->fX-pstMapCopy->fOriginInGlobalX)*10;

	
	for(i=0;i<SICK_LINES;i++)
	{
		nX=cos(pstCurPos->fTheta)*pstSICKView[i].-sin(pstCurPos->fTheta)*y+nCurPosInMapX;
		nY=sin(pstCurPos->fTheta)*pstSICKView[i].+cos(pstCurPos->fTheta)*y+nCurPosInMapY;

		if(nX>=5&&nX<=pstMapCopy->n2DGridMapW-5&&nY>=5&&nY<=pstMapCopy->n2DGridMapH)
		{
			pstMapCopy->m_pst2DGridMapData[nY*pstMapCopy->nWidth+nX]=MAP_TMP_OBSTACLE;
			pstMapCopy->m_pst2DGridMapData[nY*pstMapCopy->nWidth+nX]=MAP_TMP_OBSTACLE;
			pstMapCopy->m_pst2DGridMapData[nY*pstMapCopy->nWidth+nX]=MAP_TMP_OBSTACLE;
			pstMapCopy->m_pst2DGridMapData[nY*pstMapCopy->nWidth+nX]=MAP_TMP_OBSTACLE;
			pstMapCopy->m_pst2DGridMapData[nY*pstMapCopy->nWidth+nX]=MAP_TMP_OBSTACLE;
			pstMapCopy->m_pst2DGridMapData[nY*pstMapCopy->nWidth+nX]=MAP_TMP_OBSTACLE;
			pstMapCopy->m_pst2DGridMapData[nY*pstMapCopy->nWidth+nX]=MAP_TMP_OBSTACLE;
			pstMapCopy->m_pst2DGridMapData[nY*pstMapCopy->nWidth+nX]=MAP_TMP_OBSTACLE;
		}
	}
	return 0;
}
*/

int _2DMap::_2DMapInit()
{
	m_pst2DGridMapData=new _2DGridMapData;
	printf("m_pst2DGridMapData Addr  :%d \n",m_pst2DGridMapData);
	//memset(m_pst2DGridMapData,0,sizeof(_2DGridMapData));
	//memset(m_cOccupyFlag,0,MAX_MAP_NUM);

	//m_pstMapCopy=new _2DGridMapData[MAX_MAP_NUM];
	//memset(m_pstMapCopy,0,sizeof(_2DGridMapData)*MAX_MAP_NUM);

	return RTN_OK;
}


//cha yi ge chu shi wei zhi
/*
int _2DMap::Update2DGridMap(char *pcData)
{
	int nCx,nCy,nWidth,nHeight,nMarkValue,nMapIdx,x,y,nIdx;

	memcpy(&nCx,pcData,4);
	memcpy(&nCy,pcData+4,4);
	memcpy(&nWidth,pcData+8,4);
	memcpy(&nHeight,pcData+16,4);
	memcpy(&nMarkValue,pcData+20,4);
	memcpy(&nMapIdx,pcData+24,4);


	int nXS=nCx-nWidth/2,nYS=nCy-nHeight/2;

	nIdx=nYS*nWidth+nXS;
	for (y=-nHeight/2;y<nHeight/2+1;y++)
	{
		for (x=-nWidth/2;x<nWidth/2+1;x++)
		{
			m_pst2DGridMapData[nMapIdx].puc2DGridMapData[nIdx]=100;
			nIdx++;
		}
		nIdx+=m_pst2DGridMapData[nMapIdx].n2DGridMapW;
	}

	return RTN_OK;
}

_2DGridMapData * _2DMap::GetAMapCopy()
{
	if( m_bLockMapCopy)
	return NULL;
	else return m_pstMapCopy;
}

int _2DMap::DestroyMapCopy()
{
	if(!m_bLockMapCopy)return -1;
	else
	{
		memcpy(m_pstMapCopy->puc2DGridMapData,
			m_pst2DGridMapData[0].puc2DGridMapData,
				m_pst2DGridMapData[0].n2DGridMapDataLen);
		m_bLockMapCopy=false;
		return 0;
	}
	return RTN_OK;
}
*/


int _2DMap::AddNew2DGridMap(char *pcData)
{
	int n2DGridMapW;
	int n2DGridMapH;
	int n2DGridMapIdx;
	int n2DGridMapDataLen;
	float fResolution;

printf("AddNew  run...%d\n",pcData);
	memcpy(&n2DGridMapW,pcData,4);
	memcpy(&n2DGridMapH,pcData+4,4);
	memcpy(&n2DGridMapIdx,pcData+8,4);
	memcpy(&n2DGridMapDataLen,pcData+12,4);
	memcpy(&fResolution,pcData+16,4);

	m_pst2DGridMapData->n2DGridMapW=n2DGridMapW;
	m_pst2DGridMapData->n2DGridMapH=n2DGridMapH;
	m_pst2DGridMapData->n2DGridMapIdx=n2DGridMapIdx;
	m_pst2DGridMapData->n2DGridMapDataLen=n2DGridMapDataLen;
	m_pst2DGridMapData->fResolution=fResolution;

	m_pst2DGridMapData->puc2DGridMapData=new unsigned char[n2DGridMapDataLen];

	memcpy(m_pst2DGridMapData->puc2DGridMapData,
			pcData+20,n2DGridMapDataLen);


	//ShowPathInBMP(m_pst2DGridMapData);
	//memcpy(m_pstMapCopy,pcData+24,n2DGridMapDataLen);

	return RTN_OK;
}

int _2DMap::ShowPathInBMP(_2DGridMapData *pst2DGridMapData)
{
	BMPLib CBMP;
	int nW=pst2DGridMapData->n2DGridMapW;
	int nH=pst2DGridMapData->n2DGridMapH;
	int i,j;
	unsigned char *pucSaveImg=new unsigned char [nW*nH*3];
	memset(pucSaveImg,255,nW*nH*3);
	int idx=0;


	for (j=0;j<nH;j++)
	{
		for (i=0;i<nW;i++)
		{
			idx=j*nW+i;
			if(pst2DGridMapData->puc2DGridMapData[idx]==MAP_FREE_AREA)
			{
					pucSaveImg[idx*3]=0;
					pucSaveImg[idx*3+1]=255;
					pucSaveImg[idx*3+2]=0;
			}
			else if (pst2DGridMapData->puc2DGridMapData[idx]==MAP_UNFREE_AREA)
			{
				pucSaveImg[idx*3]=0;
				pucSaveImg[idx*3+1]=0;
				pucSaveImg[idx*3+2]=255;
			}
			else if (pst2DGridMapData->puc2DGridMapData[idx]==MAP_TMP_OBSTACLE)
			{
				pucSaveImg[idx*3]=255;
				pucSaveImg[idx*3+1]=0;
				pucSaveImg[idx*3+2]=0;
			}
			else
			{
				//	pucSaveImg[idx*3]=125;
				//	pucSaveImg[idx*3+1]=125;
				//	pucSaveImg[idx*3+2]=125;
				pucSaveImg[idx*3]=255;
				pucSaveImg[idx*3+1]=255;
				pucSaveImg[idx*3+2]=255;
			}




		}
	}
	CBMP.BMPSave("Obstacles.bmp",pucSaveImg,nW,nH);

	delete [] pucSaveImg;
	return 0;
}
