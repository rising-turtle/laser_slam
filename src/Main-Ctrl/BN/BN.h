#ifndef BN_H_
#define BN_H_
#include "../MainCtrl_Define.h"
#include "BNpos.h"
#include <pthread.h>
typedef struct BNParams
{
	char cDev[100];
	unsigned int uiBaudrate;
	unsigned char ucRobotID;
}BNParams;
class BN
{
	public:
	BN();
	~BN();
	
	BNpos *m_pCBNpos;
	
	int Init(BNParams stParams);
	int Run();
	int Stop();
	int Uninit();

	static int BNLocation(float *pfLocation);
	static BN *m_PCBN;
};
#endif
