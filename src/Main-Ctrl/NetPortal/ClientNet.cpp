#include "ClientNet.h"
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
ClientNet::ClientNet(void)
{
}

ClientNet::~ClientNet(void)
{
}
int ClientNet::ClientNetUninit()
{
	printf("close Client Sock  %d  \n",m_nClientSock);
	m_nQuit=1;
	close(m_nClientSock);
//	pthread_mutex_init(&m_MutexSendLock,0);
	return 0;
}

int ClientNet::ClientNetInit(char *pucConfig)
{

	int nHostPort,nClientPort;
	unsigned char ucHostIP[4],ucClientIP[4];
	int nJumpPort;

	m_nQuit=0;

	memcpy(ucClientIP,pucConfig,4);
	memcpy(&nClientPort,pucConfig+4,4);
	memcpy(ucHostIP,pucConfig+8,4);
	memcpy(&nHostPort,pucConfig+12,4);


	printf("IP: %d .  %d  .%d  .%d  \n",ucClientIP[0],ucClientIP[1],ucClientIP[2],ucClientIP[3]);
	printf("port:  %d  \n",nClientPort);

	printf("Host IP: %d .  %d  .%d  .%d  \n",ucHostIP[0],ucHostIP[1],ucHostIP[2],ucHostIP[3]);
	printf("Host port:  %d  \n",nHostPort);


	SetAddr(nHostPort,ucHostIP,m_HostAddr);

	nJumpPort=nClientPort;
	while((m_nClientSock=SetSocket(nJumpPort,ucClientIP,m_ClientAddr))<0)
	{
		//return -1;
		printf("Set Socket Now......\n");
		/*if(nJumpPort<nClientPort+1000)
			nJumpPort+=3;
		else nJumpPort=nClientPort;*/
		sleep(1);
	}
	Try2CncHost();

	return 0;
}
int ClientNet::SetAddr(int nPort,unsigned char *pucIP,sockaddr_in &addr)
{
	char cIPStr[16];
	addr.sin_family=AF_INET;
	addr.sin_port=htons(nPort);

	memset(cIPStr,0,16);
	sprintf(cIPStr, "%d.%d.%d.%d", pucIP[0],pucIP[1],pucIP[2],pucIP[3]);
	//printf("My IP  :%s port:  %d  \n",cIPStr,nPort);

	addr.sin_addr.s_addr=inet_addr(cIPStr);
	//printf("host ip1 :%d  port:  %d  \n  ",addr.sin_addr.s_addr,addr.sin_port);
	memset(addr.sin_zero,0,8);
	//printf("host ip2 :%d  port:  %d  \n  ",addr.sin_addr.s_addr,addr.sin_port);
	return 0;
}


int ClientNet::SetSocket(int nPort,unsigned char *pucIP,sockaddr_in &addr)
{
	int nSock,nRtn,nOn;
	char cIPStr[16];
	


	linger sLinger;
	sLinger.l_onoff=0;
	int nKeepAlive=1,nKeepIdle=1,nKeepInterval=1,nKeepCount=1;

	if ((nSock = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
		printf("set failed!!!!!!!\n");
		return -3;
	}
	setsockopt(nSock ,SOL_SOCKET,SO_LINGER,(void*)&sLinger,sizeof(linger));
	setsockopt(nSock ,SOL_SOCKET,SO_KEEPALIVE,(void*)&nKeepAlive,sizeof(nKeepAlive));
	setsockopt(nSock ,SOL_TCP,TCP_KEEPIDLE,(void*)&nKeepIdle,sizeof(nKeepIdle));
	setsockopt(nSock ,SOL_TCP,TCP_KEEPINTVL,(void*)&nKeepInterval,sizeof(nKeepInterval));
	setsockopt(nSock ,SOL_TCP,TCP_KEEPCNT,(void*)&nKeepCount,sizeof(nKeepCount));

	nOn=1;
	setsockopt(nSock,SOL_SOCKET,SO_REUSEADDR,&nOn,sizeof(nOn));

	addr.sin_family=AF_INET;

	addr.sin_port=htons(nPort);
		
	
	memset(cIPStr,0,16);
	sprintf(cIPStr, "%d.%d.%d.%d", pucIP[0],pucIP[1],pucIP[2],pucIP[3]);
	addr.sin_addr.s_addr=inet_addr(cIPStr);
	memset(addr.sin_zero,0, 8);

	if(bind(nSock,(struct sockaddr *)&addr,sizeof(struct sockaddr))==-1)
	{

		printf("port : %d   bind failed!!!!!!!\n",nPort);
		return -4;
	}
	return nSock;

}


int ClientNet::Try2CncHost()
{

	int nRtn=-1;
	int nFlags;
	while(nRtn==-1)
	{
		printf("trying to connect with host!!!!!!!!!!\n");
		if((nRtn=connect(m_nClientSock,(struct sockaddr *)&m_HostAddr,sizeof(struct sockaddr)))!=-1)
		{
			//m_cbLogFile("Connect with host Successfully!!!\n",LOG_NET);

			nFlags=fcntl(m_nClientSock,F_GETFL,0);
			fcntl(m_nClientSock,F_SETFL,nFlags|O_NONBLOCK);
			printf("Connect with host Successfully!!!\n");
			m_bStopListen=false;
			return 0;
		//	break;
		}
		else
		{
			//printf("try to connect with host falied reason:  %d!!!!\n",nRtn);
		}
		
		usleep(1000000);
	}
	return nRtn;
}


int ClientNet::RcvStream(int nGoalSize,char *pcTmpBuff,int sockfd)
{
	int nRecvCount,nLeftLen,nNumBytes;

	nLeftLen=nGoalSize;
	nRecvCount=0;
	//printf("nGoalSize:%d\n",nGoalSize);
	while (nRecvCount<nGoalSize&&m_nQuit==0)
	{
		nNumBytes=recv(sockfd, pcTmpBuff, nLeftLen, 0);
		//nNumBytes=read(sockfd,pcTmpBuff,nLeftLen);
		//printf("recv nNumBytes:%d ,err:%d,socket:%d, nLeftLen:%d \n",nNumBytes,errno,sockfd,nLeftLen);
		if (nNumBytes <=0)
		{
			if(errno==EAGAIN)
			{
				//printf("errno==EAGAIN\n");
				usleep(1);
			}
			if(errno!=EINTR&&errno!=EWOULDBLOCK)
			{
				printf("errno:%d \n",errno);
				return -1;
			}
		}
		else
		{
			nRecvCount+=nNumBytes;
			pcTmpBuff+=nNumBytes;
			nLeftLen=nGoalSize-nRecvCount;
			usleep(1);
		}

	}
	return 1;
}



int ClientNet::SendStream(int nGoalSize,char *pcTmpBuff,int sockfd)
{
	int nRecvCount,nLeftLen,nNumBytes;

	nLeftLen=nGoalSize;
	nRecvCount=0;
	//printf("SendStream0000 :%d\n",sockfd);
	while (nRecvCount<nGoalSize&&m_nQuit==0)
	{
		//printf("SendStream11111\n");
		nNumBytes=send(sockfd, pcTmpBuff, nLeftLen, 0);
		if (nNumBytes<=0)
		{
			if(errno!=EINTR&&errno!=EWOULDBLOCK&&errno!=EAGAIN)
			{
				//printf("errno:%d \n",errno);
				return -1;
			}
			else
			{
				//printf("Do not Jump errno:%d \n",errno);
			}
		}
		else
		{
			nRecvCount+=nNumBytes;
			pcTmpBuff+=nNumBytes;
			nLeftLen=nGoalSize-nRecvCount;
		}
		usleep(1);
	}

	return 1;
}

int ClientNet::SendSLAMData(char *pcData,int nDataLen)
{
	//printf("SendSLAMData0000:%d  \n",m_nClientSock);
	if(SendStream(nDataLen,pcData,m_nClientSock)==1)
	{
	//	printf("SendSLAMData1111\n");
		return 0;
	}

	else 
	{
	//	printf("SendSLAMData2222\n");
		return -1;
	}
}


int ClientNet::Listen2Host()
{
	int nRtn,nRecvDataLen;

	struct timeval timeout={0,200};
	bool bJump=false;

	char cTmp[1024*1024];
	while (!bJump&&!m_bStopListen)
	{
		fd_set fdR; 
		FD_ZERO(&fdR);
		FD_SET(m_nClientSock,&fdR);
		switch (select(m_nClientSock + 1, &fdR, NULL, NULL , &timeout)) 
		{
		case -1:
			bJump=1;
			printf("sock value :  %d  \n",m_nClientSock);
			printf("ThreadCmd Loss Connection with Host!!!!!\n");
			printf("Quit5555\n");
			break;
		case 0:   //no new data come
			break;
		default:
			nRtn=FD_ISSET(m_nClientSock,&fdR);
			//printf("AAAAAAAAAAAAAAAAAA!!!!!%d\n",nRtn);
			if (nRtn)
			{
				printf("Recv data!!!!!\n");
				if(RcvStream(4,(char*)&nRecvDataLen,m_nClientSock)!=-1)   
				{
				//	printf("Recv data len  %d \n",nRecvDataLen);
					if(RcvStream(nRecvDataLen,cTmp,m_nClientSock)!=-1)
					{
						//Parse Cmd
						m_cbNetUpload(cTmp,nRecvDataLen);
						//printf("B recv:%s \n",cTmp);
					}
					else
					{
						printf("Quit11111\n");
						bJump=true;
					}
				}
				else
				{
					printf("Quit2222\n");
					bJump=true;
				}
			}
			else
			{
				printf("Quit4444\n");
				bJump=true;
			}
			break;
		}

		usleep(20);
	}
	if (!bJump)
	{
		printf("Quit99999\n");
		return -1;
	}
	else return -1;
	
}
