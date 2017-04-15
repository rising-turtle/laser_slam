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

using namespace std;



class BNpos
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
		pthread_t listenuartid;
		robotstate robot;
		unsigned int recvcnt;
		unsigned int recvcnt_ex;
		pthread_mutex_t m_BNposMutex;
		queue<unsigned char> q1;
		unsigned char MyAddressNow;
	private:
		int fd;
		int speed_arr[18];
		int name_arr[18];
		int MyRobotID;
		
	public:
		BNpos(char* dev,unsigned int Baudrate,int myRobotID);
		~BNpos();
		void listenUART();
		void closed();
	private:
		void start(char* portname,unsigned int Baudrate);
		void set_speed(int fd, int speed);
		int set_Parity(int fd,int databits,int stopbits,int parity);
		int OpenDev(char *Dev);
		
};
