#pragma once
#include <vector>
#include <iostream>
#include <fstream>
#include <stdlib.h>
using namespace std;
class ReadSLAMResult
{
public:
	ReadSLAMResult(void);
	~ReadSLAMResult(void);
	vector<float> m_vctRobotPos;
	vector<vector<float> > m_vctPC;
	vector<vector<float> > m_vctPCLocal;

	int ReadSLAMResultInit();
	int ReadSLAMResultRun(const char *pcPath);

private:
	bool ReadTraFile(const char *pcPath);
	bool ReadCPFile(const char *pcPath);
	int TranslateCP();

	float m_fAngle[541][2];
	
	
};
