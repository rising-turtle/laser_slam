#include "AmbientGridMap.h"
#include <stdio.h>


char *AmbientGridMap::m_pcRawDataBuff1;
char *AmbientGridMap::m_pcRawDataBuff2;

char *AmbientGridMap::m_pcRawDataPtr;
char AmbientGridMap::m_cCurRawData;
pthread_t AmbientGridMap::m_hThreadWriteRawData2File;
int AmbientGridMap::m_nRawDataFileIdxW;
int AmbientGridMap::m_nRawDataFileIdxR;

vector<vector<float> > AmbientGridMap::m_vctBuff1;
vector<vector<float> > AmbientGridMap::m_vctBuff2;

int AmbientGridMap::nCurCount;

CallBack_NetUpload AmbientGridMap::m_cbSend2RS_B;
pthread_t AmbientGridMap::m_hThreadSendStoreData2RS;


pthread_mutex_t g_MutexAmbientGridMapBuff;
AmbientGridMap::AmbientGridMap()
{
	pthread_mutex_init(&g_MutexAmbientGridMapBuff,0);
}

AmbientGridMap::~AmbientGridMap()
{
}

int AmbientGridMap::AGMInit()
{
	m_pcRawDataBuff1=new char[_100M];
	m_pcRawDataBuff2=new char[_100M];

	memset(m_pcRawDataBuff1,0,_100M);
	memset(m_pcRawDataBuff2,0,_100M);

	m_pcRawDataPtr=m_pcRawDataBuff1;
	m_cCurRawData=1;

	m_nRawDataFileIdxW=0;
	m_nRawDataFileIdxR=0;
	return 0;
}

int AmbientGridMap::Wrtie2RawDataBuff(vector<float> vctBearing)
{

	return 0;
}

int AmbientGridMap::Wrtie2RawDataBuff(char *pcRawData,int nDatalen)
{
	static int nCount=0;
	pthread_mutex_trylock(&g_MutexAmbientGridMapBuff);
	if(nCount+nDatalen<_100M)
	{
		memcpy(m_pcRawDataPtr,pcRawData,nDatalen);
		m_pcRawDataPtr+=nDatalen;
		nCount+=nDatalen;
	}
	else
	{
		if(m_cCurRawData==1)
		{
			m_pcRawDataPtr=m_pcRawDataBuff2;
			m_cCurRawData=2;
			WriteRawData2File(m_pcRawDataBuff1,nCount);
			nCount=0;
		}
		else
		{
			m_pcRawDataPtr=m_pcRawDataBuff1;
			m_cCurRawData=1;
			WriteRawData2File(m_pcRawDataBuff2,nCount);
			nCount=0;
		}
		memcpy(m_pcRawDataPtr,pcRawData,nDatalen);
		m_pcRawDataPtr+=nDatalen;
		nCount+=nDatalen;
	}

	nCurCount=nCount;
	pthread_mutex_unlock(&g_MutexAmbientGridMapBuff);
	return 0;
}

int AmbientGridMap::WriteRawData2File(char *pcData,int nDatalen)
{
	RawDataInfo stRawDataInfo;
	stRawDataInfo.pcData=pcData;
	stRawDataInfo.nDataLen=nDatalen;
	stRawDataInfo.bRecv=false;
	while(pthread_create(&m_hThreadWriteRawData2File,NULL,ThreadWriteRawData2File,&stRawDataInfo)!=0)
	{
		printf("hThreadWriteRawData2File  Create Thread Failed!!!! \n");
		usleep(1000);
	}
	while(!stRawDataInfo.bRecv)
	{
		usleep(10);
	}
	return 0;
}

void* AmbientGridMap::ThreadWriteRawData2File(void* lpParam)
{
	pthread_detach(pthread_self());
	RawDataInfo stRawDataInfo;
	memcpy(&stRawDataInfo,lpParam,sizeof(RawDataInfo));
	stRawDataInfo.bRecv=true;

	FILE *pFile;
	
	char cFileName[20];
	memset(cFileName,0,20);
	sprintf(cFileName,"%d.RawData",m_nRawDataFileIdxW);
	pFile=fopen(cFileName,"wb");
	fwrite(stRawDataInfo.pcData,1,stRawDataInfo.nDataLen,pFile);
	fclose(pFile);
	m_nRawDataFileIdxW++;
	if(MAX_RAWDATA_FILE_NUM<=m_nRawDataFileIdxW)m_nRawDataFileIdxW=0;
	return RTN_OK;
}
int AmbientGridMap::SendStoreData2RS()
{
	while(pthread_create(&m_hThreadSendStoreData2RS,NULL,
			ThreadSendStoreData2RS,NULL)!=0)
	{
		printf("hThreadWriteRawData2File  Create Thread Failed!!!! \n");
		usleep(1000);
	}
	return 0;
}

void* AmbientGridMap::ThreadSendStoreData2RS(void* lpParam)
{
	pthread_detach(pthread_self());

	pthread_mutex_lock(&g_MutexAmbientGridMapBuff);
	char *pcTmp=NULL;
	char cSendData[ONE_LINE_LEN+4];
	int nCMD=OLD_PC_DATA;

	printf("ThreadSendStoreData2RS  Run!!!!!!!\n");
	printf("m_cCurRawData:%d, nCurCount:%d  \n",m_cCurRawData,nCurCount);

	int nNum,i;
	if(m_cCurRawData==1)
	{
		pcTmp=m_pcRawDataBuff1;
	}
	else
	{
		pcTmp=m_pcRawDataBuff2;
	}
	nNum=nCurCount/(ONE_LINE_LEN);


	memcpy(cSendData,&nCMD,4);

	for(i=0;i<nNum;i++)
	{
		memcpy(cSendData+4,pcTmp,ONE_LINE_LEN);
		if(m_cbSend2RS_B(cSendData,ONE_LINE_LEN+4)==-1)
		{
			break;
		}
		nCurCount-=ONE_LINE_LEN;
		pcTmp+=ONE_LINE_LEN;
		usleep(100);
	}

	printf("ThreadSendStoreData2RS  Finish!!!!!!!\n");
	pthread_mutex_unlock(&g_MutexAmbientGridMapBuff);

	return 0;
}

int AmbientGridMap::FastProbabilityGraphInit()
{
	int i;
/*	m_pstProbabilityGraphBaseNode=new ProbabilityGraphNode;
	m_pstProbabilityGraphBaseNode->ppstNeighbour=new ProbabilityGraphNode*[8];
	for(i=0;i<8;i++)
	{
		m_pstProbabilityGraphBaseNode->ppstNeighbour[i]=new ProbabilityGraphNode;
		m_pstProbabilityGraphBaseNode->ppstNeighbour[i]->ppstNeighbour=new ProbabilityGraphNode*[8];
	}

	m_pstProbabilityGraphBaseNode->nBorderXMin=0;
	m_pstProbabilityGraphBaseNode->nBorderXMax=PROB_MAP_W;
	m_pstProbabilityGraphBaseNode->nBorderYMin=0;
	m_pstProbabilityGraphBaseNode->nBorderYMax=PROB_MAP_H;
	m_pstProbabilityGraphBaseNode->pfProbabilityGraphData=new float[50*50];

	m_pstProbabilityGraphBaseNode->ppstNeighbour[i]->ppstNeighbour
*/


}





