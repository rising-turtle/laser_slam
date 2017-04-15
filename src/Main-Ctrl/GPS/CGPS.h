#ifndef CGPS_H_
#define CGPS_H_

#pragma once

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







typedef struct{
	double lat;
	double lon;
}GPSllh;


typedef struct{
	double x;//x is to east
	double y;//y is to north
}Distance;

typedef struct{
	double vel;
	double velangle;
}GPSVel;




class CGPS
{
	public:
		CGPS();
		~CGPS();
	public:
		double lat,lon,vel,velangle;
		int worknum,satnum;
		time_t position_time;
		
		int speed_arr[14];
		int name_arr[14];
		//int speed_arr[] = { B38400, B19200, B9600, B4800, B2400, B1200, B300,
	    	//	B38400, B19200, B9600, B4800, B2400, B1200, B300, };
		//int name_arr[] = {38400,  19200,  9600,  4800,  2400,  1200,  300,
	    	//	38400,  19200,  9600, 4800, 2400, 1200,  300, };
	private:
		int fd;
		double earth_R;
		double pi;
	public:
		bool getGPSValue(double *lat1,double *lon1);//return latitude and lontitude to *lat1 and *lon1,return false means parameters error
		bool getGPSXY(GPSllh *p0,GPSllh *p1,Distance *d1);//input p0,p1,output delta x and delta y from d1,return false means parameters error
		int getGPSSatelliteNum();
		bool checkGPS();
		bool getGPSVel(GPSVel *vel1); 
		void start(char* portname); 
		void listenUART();
		void closed();
	private:
		void set_speed(int fd, int speed);
		int set_Parity(int fd,int databits,int stopbits,int parity);
		int OpenDev(char *Dev);
};


#endif
