#include "SerialCom.h"

SerialCom *SerialCom::m_pCSerialCom;

pthread_mutex_t g_SerialLock;


#include <time.h>

int SerialCom::SendClearOdo()
{
	unsigned char cData[2];
	cData[0]='@';
	cData[1]='|';
	WrtieData(cData,2);
	return 0;
}

int SerialCom::GetRobotOdo()
{
	unsigned char cData[2];
	cData[0]='@';
	cData[1]='^';
	WrtieData(cData,2);
	return 0;
}
int SerialCom::GetRobotStatus()
{
	unsigned char cData[2];
	cData[0]='@';
	cData[1]='!';
	WrtieData(cData,2);
	return 0;
}


void* SerialCom::threadFunction(void *arg)
{
	//m_pCSerialCom->listenUART();
}
int SerialCom::SendGetRobotStatus()
{
	unsigned char cData[2];
	JetFire::Send_GetStatus(cData,2);
	WrtieData(cData,2);
//	printf("SendGetRobotStatus:  %c  ,%c \n",cData[0],cData[1]);
	//write(m_pCSerialCom->fd,cData,sizeof(cData));

}
int SerialCom::SendControlCMD_Rot(float RotDegree, float Rot_Velocity)
{
	unsigned char cData[32];
	memset(cData,0,32);
	JetFire::JetFireRot_Pack(RotDegree,Rot_Velocity,cData);
	WrtieData(cData,sizeof(cData));
}
int SerialCom::SendControlCMD(float fVL,float fVR,int nLTime,int nRTime)
{
	//printf("Reach SendControlCMD!!!! fVL:%f ,fVR:%f ,nLTime:%d , nRTime:%d\n",fVL*1000,fVR*1000,nLTime,nRTime);
	unsigned char cData[32];
	memset(cData,0,32);
	JetFire::Send_JetFireCmd(fVL,fVR,nLTime,nRTime,cData);
	//write(m_pCSerialCom->fd,cData,32);
	WrtieData(cData,sizeof(cData));
}
int SerialCom::WrtieData(unsigned char *pcData,int nDataLen)
{
//	printf("Serial WrtieData@@@@@@@@@@@@@@@\n");
	pthread_mutex_lock(&g_SerialLock);
//	printf("pthread_mutex_lock\n");
	write(m_pCSerialCom->fd,pcData,nDataLen);
	pthread_mutex_unlock(&g_SerialLock);
//	printf("pthread_mutex_unlock\n");
}

int SerialCom::Run()
{
	/*int i,j;
	unsigned int recvtmp=0;
	char dev[] ={"/dev/ttyUSB1"};
	BNpos robotpos(dev,115200,0);
	unsigned char pcData[32];*/
	
int nTmp;
	if(pthread_create(&m_pCSerialCom->id,NULL,threadFunction,NULL)!=0)
	{
		printf("Create thread error!\n");
	}
	/*Send_JetFireCmd(1000,1000,100, 100,pcData);



	for(j=0;j<100;j++)
	{
		sleep(10);
		write(robotpos.fd,pcData,sizeof(pcData));
		printf("mgs send\n");
	}
	printf("done\n");*/
	//robotpos.closed();
	return 0;
}

SerialCom::SerialCom(char* dev,unsigned int Baudrate,int myRobotID)
{

	m_pCSerialCom=this;
	thread_close=0;
	recvcnt=0;
	recvcnt_ex=0;
	memset(&robot,0,sizeof(robot));

	speed_arr[0] = B115200;
	speed_arr[1] = B57600;
	speed_arr[2] = B38400;
	speed_arr[3] = B19200;
	speed_arr[4] = B9600;
	speed_arr[5] = B4800;
	speed_arr[6] = B2400;
	speed_arr[7] = B1200;
	speed_arr[8] = B300;
	speed_arr[9] = B115200;
	speed_arr[10] = B57600;
	speed_arr[11] = B38400;
	speed_arr[12] = B19200;
	speed_arr[13] = B9600;
	speed_arr[14] = B4800;
	speed_arr[15] = B2400;
	speed_arr[16] = B1200;
	speed_arr[17] = B300;

	name_arr[0] = 115200;
	name_arr[1] = 57600;
	name_arr[2] = 38400;
	name_arr[3] = 19200;
	name_arr[4] = 9600;
	name_arr[5] = 4800;
	name_arr[6] = 2400;
	name_arr[7] = 1200;
	name_arr[8] = 300;
	name_arr[9] = 115200;
	name_arr[10] = 57600;
	name_arr[11] = 38400;
	name_arr[12] = 19200;
	name_arr[13] = 9600;
	name_arr[14] = 4800;
	name_arr[15] = 2400;
	name_arr[16] = 1200;
	name_arr[17] = 300;
	start(dev,Baudrate);


	pthread_mutex_init(&m_BNposMutex,0);
	q1.empty();
	MyRobotID = myRobotID;
	MyAddressNow = 0;

	pthread_mutex_init(&g_SerialLock,0);

}

void SerialCom::set_speed(int fd, int speed)
{
  int   i;
  int   status;
  struct termios   Opt;
  tcgetattr(fd, &Opt);
  for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++)
   {
   	if  (speed == name_arr[i])
   	{
   	    tcflush(fd, TCIOFLUSH);
    	cfsetispeed(&Opt, speed_arr[i]);
    	cfsetospeed(&Opt, speed_arr[i]);
    	Opt.c_lflag &=~(ICANON|ECHO|ECHOE|ISIG);
    	Opt.c_oflag &=!OPOST;

    	Opt.c_cc[VTIME]=0;
    	Opt.c_cc[VMIN]=0;
    	status = tcsetattr(fd, TCSANOW, &Opt);
    	if  (status != 0)
            perror("tcsetattr fd1");
     	return;
     	}
   tcflush(fd,TCIOFLUSH);
   }
}

void SerialCom::closed()
{

	thread_close=1;
    	pthread_join(id,NULL);
	close(fd);
}

SerialCom::~SerialCom()
{
	if(thread_close==0)
	{
		thread_close=1;
    		pthread_join(id,NULL);
		close(fd);
	}
}


int SerialCom::set_Parity(int fd,int databits,int stopbits,int parity)
{
	struct termios options;
 if  ( tcgetattr( fd,&options)  !=  0)
  {
  	perror("SetupSerial 1");
  	return(0);
  }
  options.c_cflag &= ~CSIZE;
  switch (databits) /*è®Ÿçœ®æ°æ®äœæ°*/
  {
  	case 7:
  		options.c_cflag |= CS7;
  		break;
  	case 8:
		options.c_cflag |= CS8;
		break;
	default:
		fprintf(stderr,"Unsupported data size\n");
		return (0);
	}
  switch (parity)
  	{
  	case 'n':
	case 'N':
		options.c_cflag &= ~PARENB;   /* Clear parity enable */
		options.c_iflag &= ~INPCK;     /* Enable parity checking */
		break;
	case 'o':
	case 'O':
		options.c_cflag |= (PARODD | PARENB);  /* è®Ÿçœ®äžºå¥æéª*/
		options.c_iflag |= INPCK;             /* Disnable parity checking */
		break;
	case 'e':
	case 'E':
		options.c_cflag |= PARENB;     /* Enable parity */
		options.c_cflag &= ~PARODD;   /* èœ¬æ¢äžºå¶æéª*/
		options.c_iflag |= INPCK;       /* Disnable parity checking */
		break;
	case 'S':
	case 's':  /*as no parity*/
		options.c_cflag &= ~PARENB;
		options.c_cflag &= ~CSTOPB;
		break;
	default:
		fprintf(stderr,"Unsupported parity\n");
		return (0);
		}
  /* è®Ÿçœ®ååéæ­¢äœ?/
  switch (stopbits)
  	{
  	case 1:
  		options.c_cflag &= ~CSTOPB;
		break;
	case 2:
		options.c_cflag |= CSTOPB;
		break;
	default:
		fprintf(stderr,"Unsupported stop bits\n");
		return (0);
	}
  /* Set input parity option */
  if (parity != 'n')
  		options.c_iflag |= INPCK;
    options.c_cc[VTIME] = 150; // 15 seconds
    options.c_cc[VMIN] = 0;

	options.c_lflag &= ~(ICANON |ISIG);
	options.c_iflag &= ~(ICRNL|IGNCR);


  tcflush(fd,TCIFLUSH); /* Update the options and do it NOW */
  if (tcsetattr(fd,TCSANOW,&options) != 0)
  	{
  		perror("SetupSerial 3");
		return (0);
	}
  return (1);
 }

int SerialCom::OpenDev(char *Dev)
{

printf("OpenDev!!!\n");
int	fd = open( Dev, O_RDWR);// | O_NDELAY);
	if (-1 == fd)
		{ /*è®Ÿçœ®æ°æ®äœæ°*/
			perror("Can't Open Serial Port");
			return -1;
		}
	else
	return fd;

}

void SerialCom::start(char* portname,unsigned int Baudrate)
{
	printf("SerialCom::start!!!\n");
	fd = OpenDev(portname);
	if (fd>0)
    		set_speed(fd,Baudrate);
	else
	{
		printf("Can't Open Serial Port!\n");
		//exit(0);
	}
  if (set_Parity(fd,8,1,'N')== 0)
  {
    printf("Set Parity Error\n");
   // exit(1);
  }

 // fcntl(fd,F_SETFL,FNDELAY);
}

void SerialCom::listenUART()
{
	unsigned char buff[512];
	int nread;
	int i;
	unsigned char tmpc[24];
	unsigned int cRobID1,cRobID2;
	while(thread_close==0)
  	{
		memset(buff,0,sizeof(buff));
		nread=read(fd,buff,512);
		if(nread>=1)
		{
			buff[nread]='\0';
			//printf("%s",buff);
			memset(buff,0,sizeof(buff));
		}
	}

}


int SerialCom::CheckHead(char *pcData,int nDatalen,int &nPos)
{
	int i;
	if(pcData[0]!='[')
	{
		for(i=1;i<32;i++)
		{
			if(pcData[i]==']')
			{
				nPos=i+1;
				return 1;
			}
			else if(pcData[i]=='[')
			{
				nPos=i;
				return -1;
			}
		}
		return -2;
	}
	else
	{
		if(pcData[31]==']')
		{
			nPos=0;
			return 0;
		}
		else
		{
			for(i=31;i>=0;i--)
			{
				if(pcData[i]==']')
				{
					nPos=i+1;
					return 1;
				}
				else if(pcData[i]=='[')
				{
					nPos=i;
					return -1;
				}
			}
		}
		return -2;
	}
	return false;
}

int SerialCom::ReadData(char *pcData,int nDataLen)
{
	//printf("pcData:%d , nDataLen:%d \n",pcData,nDataLen);

	//read(m_pCSerialCom->fd,pcData,nDataLen);

	char cData[33],cData1[33],cData2[33];
	int i,nPos,nRtn,nTmp1,nTmp2;
	memset(cData,0,33);

	/*if(RcvStream(32,cData,m_pCSerialCom->fd)!=-1)
	{

	}*/


	/*int nRtnNum=0,nRecvThisNum;

	while(nRtnNum<32)
	{
		nRecvThisNum=read(m_pCSerialCom->fd,cData+nRtnNum,32);
		printf("Recv Num :%d  \n",nRtnNum);
	}*/

	if(RcvStream(32,cData,m_pCSerialCom->fd)!=-1)
		{

		}

	nPos=0;
	while((nRtn=CheckHead(cData,32,nPos))!=0)
	{
		//printf("nRtn:%d  ,%s,  nPos:%d\n",nRtn,cData,nPos);
		if(nRtn==1)
		{

			nTmp1=32-nPos;
			memset(cData1,0,33);
			//read(m_pCSerialCom->fd,cData1,nPos);
			if(RcvStream(nPos,cData1,m_pCSerialCom->fd)==-1)
			{
				return -1;
			}

			//printf("cData1:%s  \n",cData1);
			memset(cData2,0,33);
			memcpy(cData2,cData+nPos,nTmp1);
			//printf("AcData2:%s  \n",cData2);
			memcpy(cData2+nTmp1,cData1,nPos);
			//printf("BcData2:%s  \n",cData2);
			memcpy(cData,cData2,32);
		}
		else if(nRtn==-1)
		{
			if(nPos==0)nPos=32;
			nTmp1=32-nPos;
			memset(cData1,0,33);
			//read(m_pCSerialCom->fd,cData1,nPos);
			if(RcvStream(nPos,cData1,m_pCSerialCom->fd)==-1)
			{
				return -1;
			}

			//printf("cData1:%s  \n",cData1);
			memset(cData2,0,33);
			memcpy(cData2,cData+nPos,nTmp1);
			//printf("AcData2:%s  \n",cData2);
			memcpy(cData2+nTmp1,cData1,nPos);
			//printf("BcData2:%s  \n",cData2);
			memcpy(cData,cData2,32);
		}
		else if(nRtn==-2)
		{
			//read(m_pCSerialCom->fd,cData,32);
			if(RcvStream(32,cData1,m_pCSerialCom->fd)==-1)
			{
				return -1;
			}
		}
		nPos=0;
	}
	//printf("Recv  %s  \n",cData);
	memcpy(pcData,cData,32);
	return 0;
}


int SerialCom::RcvStream(int nGoalSize,char *pcTmpBuff,int sockfd)
{
	int nRecvCount,nLeftLen,nNumBytes;

	nLeftLen=nGoalSize;
	nRecvCount=0;

	clock_t StartTime=clock();
	clock_t EndTime;

	while (nRecvCount<nGoalSize)
	{
		if ((nNumBytes=read(sockfd, pcTmpBuff, nLeftLen)) <=0)
		{
			return -1;
		}
		//printf("nNumBytes:%d  \n",nNumBytes);
		nRecvCount+=nNumBytes;
		pcTmpBuff+=nNumBytes;
		nLeftLen=nGoalSize-nRecvCount;
	//	EndTime=clock()-StartTime;
	//	printf("time:EndTime %d  \n",EndTime);
	/*	if(EndTime>200000)
		{
			return -1;
		}*/
	}
	return 1;
}
