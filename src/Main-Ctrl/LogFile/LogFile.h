#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include "pthread.h"

using namespace std;


#define MAX_CNT_LEN 1024
#define MODULE_NUM 30


typedef struct LogFormat
{
	char cDataTime[20];
	char cModuleName[20];
	char cContent[MAX_CNT_LEN];
}LogFormat;
class LogFile
{
public:
	LogFile(void);
	~LogFile(void);


	int LogFileInit(char *pcLogFilePath);
	int LogFileRun();
	int LogFileStop();
	int LogFileUninit();

	static int PushContent2LogStack(LogFormat *pstContent);
	static int PushContent2LogStack(char* pcContent,int nModuleIdx);

private:

	int Write2LogFile(LogFormat *pstContent);

//	HANDLE m_hThreadLogFileRun;
//	static UINT ThreadLogFileRun(LPVOID lpParam);


	bool m_bStopLog;
	ofstream m_LogFile;

	vector<LogFormat> m_vctLogContent;
	static LogFile *m_pCLogFile;

	char *m_pcReserveSpace;

	char *m_pcDataTimehead;
	char *m_pcModuleHead;
	char *m_pcContentHead;
	char *m_pcTail;
	char *m_pcEndL;

	char *m_pcLogFileName;

	char m_ccModuleTable[MODULE_NUM][20];
};
