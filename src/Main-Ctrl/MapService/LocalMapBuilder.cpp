

#include "LocalMapBuilder.h"
#include "AmbientGridMap.h"

LocalMapBuilder::LocalMapBuilder()
{
}

LocalMapBuilder::~LocalMapBuilder()
{
}



int LocalMapBuilder::MapStreamIn(char *pcMapData,int nMapDataLen)
{
	AmbientGridMap::Wrtie2RawDataBuff(pcMapData,nMapDataLen);
	return 0;
}


