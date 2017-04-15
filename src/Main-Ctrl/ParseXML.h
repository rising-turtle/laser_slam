#pragma once
#include <vector>
#include <iostream>   
#include <fstream>
#include <stdlib.h>
#include <string.h>

using namespace std;
class ParseXML
{
public:
	ParseXML(void);
	~ParseXML(void);


	int ParseXMLRun(const char *pcPath);
	int ParseIP(char *pcData,char *pcIP);
	vector<string> m_vctData;

private:

	char *m_pcFileBuff;
	int m_nFileLen;
	
	
	bool ReadWholeFile(const char *pcPath);
	bool Stat2Parse(char * pcBuff,int &nLen);
	bool GetContent(char * pcBuff,int &nLen,int nStyle);
};
