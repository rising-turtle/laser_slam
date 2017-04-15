#include     <stdio.h>      /*标准输入输出定义*/
#include     <stdlib.h>     /*标准函数库定义*/
#include     <unistd.h>     /*Unix标准函数定义*/
#include     <sys/types.h>  /**/
#include     <sys/stat.h>   /**/
#include     <fcntl.h>      /*文件控制定义*/
#include     <termios.h>    /*PPSIX终端控制定义*/
#include     <errno.h>      /*错误号定义*/
#include	<memory.h>
#include	<math.h>
#include	<time.h>
#include	<stdint.h>
#include	<pthread.h>
#include	<queue>
#include        "JetFire.h"

using namespace std;
#include "JetFire.h"


class SerialCom
{

	public:
		typedef struct{
			int robotid;
			int speedx;
			int speedy;
			int speedr;
			int positionx;
			int positiony;
			int angle;
			int address;
			int obstaclepos;
		}robotstate;
		int thread_close;//switch to close thread
		pthread_t id;
		robotstate robot;
		unsigned int recvcnt;
		unsigned int recvcnt_ex;
		pthread_mutex_t m_BNposMutex;
		queue<unsigned char> q1;
		int fd;

		int Run();

		static int SendControlCMD(float fVL,float fVR,int nLTime,int nRTime);
		static int SendControlCMD_Rot(float RotDegree, float Rot_Velocity);
		static int SendGetRobotStatus();
		static int SendClearOdo();

		static int GetRobotOdo();
		static int GetRobotStatus();

		static int ReadData(char *pcData,int nDataLen);
		static int WrtieData(unsigned char *pcData,int nDataLen);
		static SerialCom *m_pCSerialCom;

		static void* threadFunction(void *arg);
	private:
		
		int speed_arr[18];
		int name_arr[18];
		int MyRobotID;
		unsigned char MyAddressNow;
	public:
		SerialCom(char* dev,unsigned int Baudrate,int myRobotID);
		~SerialCom();
		void listenUART();
		void closed();
	private:
		void start(char* portname,unsigned int Baudrate);
		void set_speed(int fd, int speed);
		int set_Parity(int fd,int databits,int stopbits,int parity);
		int OpenDev(char *Dev);
		void changeaddress(unsigned char MyAddressNow);


		static int CheckHead(char *pcData,int nDatalen,int &nPos);

		static int RcvStream(int nGoalSize,char *pcTmpBuff,int sockfd);
};
