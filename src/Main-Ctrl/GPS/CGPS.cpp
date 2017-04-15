#include "CGPS.h"

CGPS::CGPS()
{
	//int speed_arr[] = { B38400, B19200, B9600, B4800, B2400, B1200, B300,
	//	B38400, B19200, B9600, B4800, B2400, B1200, B300, };
	//int name_arr[] = {38400,  19200,  9600,  4800,  2400,  1200,  300,
	//	38400,  19200,  9600, 4800, 2400, 1200,  300, };
	speed_arr[0]=B38400;
	speed_arr[1]=B19200;
	speed_arr[2]=B9600;
	speed_arr[3]=B4800;
	speed_arr[4]=B2400;
	speed_arr[5]=B1200;
	speed_arr[6]=B300;

	speed_arr[7]=B38400;
	speed_arr[8]=B19200;
	speed_arr[9]=B9600;
	speed_arr[10]=B4800;
	speed_arr[11]=B2400;
	speed_arr[12]=B1200;
	speed_arr[13]=B300;

	name_arr[0]=38400;
	name_arr[1]=19200;
	name_arr[2]=9600;
	name_arr[3]=4800;
	name_arr[4]=2400;
	name_arr[5]=1200;
	name_arr[6]=300;

	name_arr[7]=38400;
	name_arr[8]=19200;
	name_arr[9]=9600;
	name_arr[10]=4800;
	name_arr[11]=2400;
	name_arr[12]=1200;
	name_arr[13]=300;
	
}

CGPS::~CGPS()
{
}

void CGPS::set_speed(int fd, int speed)
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
/**
*@brief   设置串口数据位，停止位和效验位
*@param  fd     类型  int  打开的串口文件句柄*
*@param  databits 类型  int 数据位   取值 为 7 或者8*
*@param  stopbits 类型  int 停止位   取值为 1 或者2*
*@param  parity  类型  int  效验类型 取值为N,E,O,,S
*/
int CGPS::set_Parity(int fd,int databits,int stopbits,int parity)
{
	struct termios options;
 if  ( tcgetattr( fd,&options)  !=  0)
  {
  	perror("SetupSerial 1");
  	return(0);
  }
  options.c_cflag &= ~CSIZE;
  switch (databits) /*设置数据位数*/
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
		options.c_cflag |= (PARODD | PARENB);  /* 设置为奇效验*/ 
		options.c_iflag |= INPCK;             /* Disnable parity checking */
		break;
	case 'e':
	case 'E':
		options.c_cflag |= PARENB;     /* Enable parity */
		options.c_cflag &= ~PARODD;   /* 转换为偶效验*/  
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
  /* 设置停变量止位*/   
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

  tcflush(fd,TCIFLUSH); /* Update the options and do it NOW */
  if (tcsetattr(fd,TCSANOW,&options) != 0)
  	{
  		perror("SetupSerial 3");
		return (0);
	}
  return (1);
 }
/**
*@breif 打开串口
*/
int CGPS::OpenDev(char *Dev)
{
int	fd = open( Dev, O_RDWR );         //| O_NOCTTY | O_NDELAY
	if (-1 == fd)
		{ /*设置数据位数*/
			perror("Can't Open Serial Port");
			return -1;
		}
	else
	return fd;

}
/**
*@breif 	main()
*/
void CGPS::start(char* portname)
{
	//char *dev ="/dev/ttyUSB0";
	fd = OpenDev(portname);
	if (fd>0)
    		set_speed(fd,9600);
	else
	{
		printf("Can't Open Serial Port!\n");
		exit(0);
	}
  if (set_Parity(fd,8,1,'N')== 0)
  {
    printf("Set Parity Error\n");
    exit(1);
  }
	
  
}


void CGPS::listenUART()
{
	int i;
	int counti=0;
	int latstart=0,latend=0,lonstart=0,lonend=0,workstart=0,satstart=0,satend=0,velstart=0,velend=0,velanglestart=0,velangleend=0;	
	double lat_tmp=0,lon_tmp=0,vel_tmp,velangle_tmp=0;
	int lat_degree=0,lon_degree=0;
	double lat_minute=0,lon_minute=0;
	char tmp[20];
	int sizetmp;
	
	
	
	int nread;
	char buff[512];
	earth_R=6378.137;
	pi=3.14159265;
	
	


	lat=0;
	lon=0;
	vel=0;
	velangle=0;
	worknum=0;
	satnum=0;


	while((nread = read(fd,buff,512))>0)
	{
	    if((buff[0]=='$') && (buff[1]=='G') && (buff[2]=='P') && (buff[3]=='G') && (buff[4]=='G') && (buff[5]=='A'))
	    {
		latstart=0;
		latend=0;
		lonstart=0;
		lonend=0;
		counti=0;
		for(i=0;i<nread;i++)
		{
		    if(buff[i] != ',')
		    {
		    }
		    else
		    {
			counti++;
			if(counti == 2)
			{
			    latstart = i;
			}
			else if(counti == 3)
			{
			    latend = i;
			}
			else if(counti == 4)
			{
			    lonstart = i;
			}
			else if(counti == 5)
			{
			    lonend = i;
			}
			else if(counti == 6)
			{
			    workstart = i;
			}
			else if(counti == 7)
			{
			    satstart = i;
			}
			else if(counti == 8)
			{
			    satend = i;
			    break;
			}
		    }

		}
		if(counti != 0)
		{
		    counti=0;
		    memset(tmp,0,20);
		    sizetmp=latend-latstart-1;
		    for(i=0;i<sizetmp;i++)
		    {
			tmp[i]=buff[i+latstart+1];
		    }
		    sscanf(tmp,"%lf",&lat_tmp);

		    memset(tmp,0,20);
		    sizetmp=lonend-lonstart-1;
		    for(i=0;i<sizetmp;i++)
		    {
			tmp[i]=buff[i+lonstart+1];
		    }
		    sscanf(tmp,"%lf",&lon_tmp);

		    lat_degree=(int)(lat_tmp/100);
		    lon_degree=(int)(lon_tmp/100);
		    lat_minute=lat_tmp-100*lat_degree;
		    lon_minute=lon_tmp-100*lon_degree;
		    lat=(double)(lat_degree)+lat_minute/60;
		    lon=(double)(lon_degree)+lon_minute/60;
		    time(&position_time);

		    memset(tmp,0,20);
		    tmp[0]=buff[workstart+1];
		    sscanf(tmp,"%d",&worknum);

		    memset(tmp,0,20);
		    sizetmp=satend-satstart-1;
		    for(i=0;i<sizetmp;i++)
		    {
			tmp[i]=buff[i+satstart+1];
		    }
		    sscanf(tmp,"%d",&satnum);

		    //printf("%lf",positiontime);
		    //printf("%lf,%lf,%d,%d\n",lat,lon,worknum,satnum);
		    //printf("%s\n",buff);
		}
	    }
	    if((buff[0]=='$') && (buff[1]=='G') && (buff[2]=='P') && (buff[3]=='R') && (buff[4]=='M') && (buff[5]=='C'))
	    {
		velstart=0;
		velend=0;
		velanglestart=0;
		velangleend=0;
		counti=0;
		for(i=0;i<nread;i++)
		{
		    if(buff[i] != ',')
		    {
		    }
		    else
		    {
			counti++;
			if(counti == 7)
			{
			    velstart = i;
			}
			else if(counti == 8)
			{
			    velend = i;
			    velanglestart = i;
			}
			else if(counti == 9)
			{
			    velangleend = i;
			    break;
			}
		    }

		}
		if(counti != 0)
		{
		    counti=0;
		    memset(tmp,0,20);
		    sizetmp=velend-velstart-1;
		    for(i=0;i<sizetmp;i++)
		    {
			tmp[i]=buff[i+velstart+1];
		    }
		    sscanf(tmp,"%lf",&vel_tmp);

		    memset(tmp,0,20);
		    sizetmp=velangleend-velanglestart-1;
		    for(i=0;i<sizetmp;i++)
		    {
			tmp[i]=buff[i+velanglestart+1];
		    }
		    sscanf(tmp,"%lf",&velangle_tmp);


		    vel=vel_tmp*0.514;
		    velangle=velangle_tmp;	

		    //printf("%lf,%lf\n",vel,velangle);
		    //printf("%s\n",buff);
		}
	    }
	    memset(buff,0,512);
	    nread=0;
	    usleep(100000);
	}
}

void CGPS::closed()
{

    close(fd);
}

bool CGPS::getGPSValue(double *lat1,double *lon1)
{
    if((lat==0) || (lon==0))
	{
		return 0;
	}
	else
	{
		lat1=&lat;
		lon1=&lon;
		return 1;
	}
}


bool CGPS::getGPSVel(GPSVel *vel1)
{
	if((vel==0) && (velangle==0))
	{
		return 0;
	}
	else
	{
		vel1->vel=vel;
		vel1->velangle=velangle;
		return 1;
	}
}

bool CGPS::getGPSXY(GPSllh *p0,GPSllh *p1,Distance *d1)
{
	double x,y,cos_lat=0,pi_lat=0;
	double lat0=0,lon0=0,lat1=0,lon1=0;
	lat0=p0->lat;
	lon0=p0->lon;
	lat1=p1->lat;
	lon1=p1->lon;
	if((lat0==0) || (lon0==0) || (lat1==0) || (lon1==0))
	{
		return 0;
	}
	else
	{
		pi_lat=lat0*pi/180;
		cos_lat=cos(pi_lat);
		x=(lon1-lon0)*pi*(earth_R*1000*cos_lat)/180;
		y=(lat1-lat0)*pi*earth_R*1000/180;
		d1->x=x;
		d1->y=y;
		return 1;
	}
}


int CGPS::getGPSSatelliteNum()
{
	return satnum;
}


bool CGPS::checkGPS()
{
	if(worknum == 1)
		return 1;
	else
		return 0;
}








