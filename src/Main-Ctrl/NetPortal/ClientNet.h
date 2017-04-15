#pragma once



#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>   

#include <arpa/inet.h>
#include <sys/types.h>
#include <netinet/tcp.h>

#include "pthread.h"


//#include <Winsock2.h>
#include <string.h>
//#include <stdio.h>
//#include <mstcpip.h>
#include "../MainCtrl_Define.h"
#define TRY_TIME 10
#pragma comment (lib,"wsock32.lib")
#pragma comment (lib,"ws2_32.lib")

typedef int (*CallBack_NetUpload)(char *pcData,int nDataLen);
class ClientNet
{
public:
	ClientNet(void);
	~ClientNet(void);

	int ClientNetInit(char *pucConfig);
	int ClientNetUninit();
	int SendSLAMData(char *pcData,int nDataLen);

	
	int Listen2Host();
	bool m_bStopListen;

	CallBack_NetUpload m_cbNetUpload;
	CallBack_LogFile m_cbLogFile;
private:


	int SetAddr(int nPort,unsigned char *pucIP,sockaddr_in &addr);
	int SetSocket(int nPort,unsigned char *pucIP,sockaddr_in &addr);
	int Try2CncHost();
	int RcvStream(int nGoalSize,char *pcTmpBuff,int sockfd);
	int SendStream(int nGoalSize,char *pcTmpBuff,int sockfd);


	struct sockaddr_in m_HostAddr;
	struct sockaddr_in m_ClientAddr;
	
	int m_nClientSock;

	int m_nQuit;


};
