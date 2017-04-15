
#ifndef  LMB_H_
#define  LMB_H_
#pragma once
#include "AmbientGridMap.h"
class LocalMapBuilder
{
public:
	LocalMapBuilder();
	~LocalMapBuilder();
	static int MapStreamIn(char *pcMapData,int nMapDataLen);	
};

#endif
