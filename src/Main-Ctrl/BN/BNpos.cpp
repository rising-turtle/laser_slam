#include "BNpos.h"

/*
FILE *fp;
pthread_t id1;
bool endthread=false;



void* readthreadFunction(void *arg)
{
	BNpos* robotpos1=(BNpos*)arg;
	robotpos1->listenUART();
}



void* printfthreadFunction(void *arg)
{
	BNpos* robotpos1=(BNpos*)arg;
	unsigned int recvtmp=0;
	while(1)
	{
		if(!endthread)
		{
			if(robotpos1->recvcnt>robotpos1->recvcnt_ex)
			{
				recvtmp=robotpos1->recvcnt-robotpos1->recvcnt_ex;
				robotpos1->recvcnt_ex=robotpos1->recvcnt;
				printf("id=%d,vx=%d,vy=%d,vr=%d,posx=%d,posy=%d,angle=%d,addr=%d,obs=%d\n",robotpos1->robot.robotid,robotpos1->robot.speedx,robotpos1->robot.speedy,robotpos1->robot.speedr,robotpos1->robot.positionx,robotpos1->robot.positiony,robotpos1->robot.angle,robotpos1->robot.address,robotpos1->robot.obstaclepos);
				printf("freq=%d\n",recvtmp);
			}
			else
			{
				recvtmp=0;
				printf("freq=%d\n",recvtmp);
			}
			sleep(1);
		}
		else
		{
			printf("printf thread quit\n");
			return 0;
		}
	}
}



int main()
{
	int i,j;
	char hehe;
	char dev[] ={"/dev/ttyUSB0"};
	BNpos robotpos(dev,57600,18);
	
	fp=fopen("log.txt","w");
	if(fp==NULL)
	{
		printf("log.txt file open error\n");
	}
	
	if(pthread_create(&(robotpos.listenuartid),NULL,readthreadFunction,(void*)(&robotpos))!=0)
	{
		printf("Create read thread error!\n");
	}

	if(pthread_create(&id1,NULL,printfthreadFunction,(void*)(&robotpos))!=0)
	{
		printf("create printf thread error!\n");
	}
	

	while(1)
	{
		
		hehe=getchar();
		if(hehe=='q')
		{
			endthread=true;
			pthread_join(id1,NULL);
			robotpos.closed();
			fclose(fp);
			printf("done\n");
			return 0;
		}
	}
	
	
	return 0;
}

*/








BNpos::BNpos(char* dev,unsigned int Baudrate,int myRobotID)
{
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

}

void BNpos::set_speed(int fd, int speed)
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
    	status = tcsetattr(fd, TCSANOW, &Opt);
    	if  (status != 0)
            perror("tcsetattr fd1");
     	return;
     	}
   tcflush(fd,TCIOFLUSH);
   }
}

void BNpos::closed()
{

	thread_close=1;
  pthread_join(listenuartid,NULL);
	close(fd);
}

BNpos::~BNpos()
{
	if(thread_close==0)
	{
		thread_close=1;
    pthread_join(listenuartid,NULL);
		close(fd);
	}
}


int BNpos::set_Parity(int fd,int databits,int stopbits,int parity)
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
  /* è®Ÿçœ®ååéæ­¢äœ?*/
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

int BNpos::OpenDev(char *Dev)
{
int	fd = open( Dev, O_RDWR );         //| O_NOCTTY | O_NDELAY
	if (-1 == fd)
		{ /*è®Ÿçœ®æ°æ®äœæ°*/
			perror("Can't Open Serial Port");
			return -1;
		}
	else
	return fd;

}

void BNpos::start(char* portname,unsigned int Baudrate)
{

	fd = OpenDev(portname);
	if (fd>0)
    		set_speed(fd,Baudrate);
	else
	{
		printf("Can't Open Serial Port!\n");
	}
  if (set_Parity(fd,8,1,'N')== 0)
  {
    printf("Set Parity Error\n");
  }
}

void BNpos::listenUART()
{
	unsigned char buff[512];
	int nread;
	int i;
	unsigned char tmpc[24];
	unsigned int cRobID1,cRobID2;
	int tmp=0;

	while(thread_close==0)
  	{
		memset(buff,0,sizeof(buff));
		nread=read(fd,buff,512);
		if(nread>=1)
		{
			for(i=0;i<nread;i++)
			{
				q1.push(buff[i]);
			}
			memset(buff,0,sizeof(buff));
		}
		while(q1.size()>=25)
		{
			if(q1.front()==0xFF)
			{
				q1.pop();
				if((q1.front() & 0xF0) == 0x90)
				{
					for(i=0;i<24;i++)
					{
						tmpc[i]=q1.front();
						q1.pop();
					}
					cRobID1 = ((tmpc[0] & 0x0c)<<2) | (tmpc[1]>>4);
					cRobID2 = ((tmpc[0] & 0x03)<<4 | (tmpc[1] & 0x0f));
					if(MyRobotID==cRobID1)
					{
						i=0;
						robot.robotid = MyRobotID;
						robot.speedx = ((tmpc[2+11*i] & 0x80)==0) ? ((tmpc[2+11*i] & 0x7f) | ((tmpc[10+11*i] & 0xc0)<<1)) : (0 - ((tmpc[2+11*i] & 0x7f) | ((tmpc[10+11*i] & 0xc0)<<1)));
						robot.speedy = ((tmpc[3+11*i] & 0x80)==0) ? ((tmpc[3+11*i] & 0x7f) | ((tmpc[10+11*i] & 0x30)<<3)) : (0 - ((tmpc[3+11*i] & 0x7f) | ((tmpc[10+11*i] & 0x30)<<3)));
						robot.speedr = ((tmpc[4+11*i] & 0x80)==0) ? ((tmpc[4+11*i] & 0x7f) | ((tmpc[10+11*i] & 0x0c)<<5)) : (0 - ((tmpc[4+11*i] & 0x7f) | ((tmpc[10+11*i] & 0x0c)<<5)));
						robot.angle  = ((tmpc[9+11*i] & 0x80)==0) ? ((tmpc[9+11*i] & 0x7f) | ((tmpc[10+11*i] & 0x03)<<7)) : (0 - ((tmpc[9+11*i] & 0x7f) | ((tmpc[10+11*i] & 0x03)<<7)));
						robot.positionx = ((tmpc[5+11*i] & 0x80)==0) ? (((tmpc[5+11*i] & 0x7f)<<8) | tmpc[6+11*i]) : (0 - ((tmpc[5+11*i] & 0x7f)<<8 | tmpc[6+11*i]));
						robot.positiony = ((tmpc[7+11*i] & 0x80)==0) ? (((tmpc[7+11*i] & 0x7f)<<8) | tmpc[8+11*i]) : (0 - ((tmpc[7+11*i] & 0x7f)<<8 | tmpc[8+11*i]));
						robot.address = ((tmpc[11+11*i] & 0x80)==0) ? (tmpc[11+11*i] & 0x7f) : 0xFF;
						robot.obstaclepos = ((tmpc[12+11*i] & 0x80)==0) ? 0 : (tmpc[12+11*i] & 0x7f);
						tmp=(tmpc[11+11*i] & 0x7f);
						//fprintf(fp,"addr=%d,tmp=%d,id=%d\n",robot.address,tmp,robot.robotid);
						
						recvcnt++;
					}
					else if(MyRobotID==cRobID2)
					{
						i=1;
						robot.robotid = MyRobotID;
						robot.speedx = ((tmpc[2+11*i] & 0x80)==0) ? ((tmpc[2+11*i] & 0x7f) | ((tmpc[10+11*i] & 0xc0)<<1)) : (0 - ((tmpc[2+11*i] & 0x7f) | ((tmpc[10+11*i] & 0xc0)<<1)));
						robot.speedy = ((tmpc[3+11*i] & 0x80)==0) ? ((tmpc[3+11*i] & 0x7f) | ((tmpc[10+11*i] & 0x30)<<3)) : (0 - ((tmpc[3+11*i] & 0x7f) | ((tmpc[10+11*i] & 0x30)<<3)));
						robot.speedr = ((tmpc[4+11*i] & 0x80)==0) ? ((tmpc[4+11*i] & 0x7f) | ((tmpc[10+11*i] & 0x0c)<<5)) : (0 - ((tmpc[4+11*i] & 0x7f) | ((tmpc[10+11*i] & 0x0c)<<5)));
						robot.angle  = ((tmpc[9+11*i] & 0x80)==0) ? ((tmpc[9+11*i] & 0x7f) | ((tmpc[10+11*i] & 0x03)<<7)) : (0 - ((tmpc[9+11*i] & 0x7f) | ((tmpc[10+11*i] & 0x03)<<7)));
						robot.positionx = ((tmpc[5+11*i] & 0x80)==0) ? (((tmpc[5+11*i] & 0x7f)<<8) | tmpc[6+11*i]) : (0 - ((tmpc[5+11*i] & 0x7f)<<8 | tmpc[6+11*i]));
						robot.positiony = ((tmpc[7+11*i] & 0x80)==0) ? (((tmpc[7+11*i] & 0x7f)<<8) | tmpc[8+11*i]) : (0 - ((tmpc[7+11*i] & 0x7f)<<8 | tmpc[8+11*i]));
						robot.address = ((tmpc[11+11*i] & 0x80)==0) ? (tmpc[11+11*i] & 0x7f) : 0;
						robot.obstaclepos = ((tmpc[12+11*i] & 0x80)==0) ? 0 : (tmpc[12+11*i] & 0x7f);
						tmp=(tmpc[11+11*i] & 0x7f);
						//fprintf(fp,"addr=%d,tmp=%d,id=%d\n",robot.address,tmp,robot.robotid);
						
						recvcnt++;
					}
				}
				else if((q1.front() & 0xf0)==0x70)
				{
					for(i=0;i<24;i++)
						{
							tmpc[i]=q1.front();
							q1.pop();
						}
						if(tmpc[2]==0x1D)
						{
							printf("board change freq to %d\n",tmpc[1]);
							//fprintf(fp,"change done to %d\n",tmpc[1]);
						}
				}
				else
				{
					q1.pop();
				}
			}
			else
			{
				q1.pop();
			}

		}
	}

}

