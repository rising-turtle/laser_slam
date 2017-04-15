
#include "SerialPort.h"
#include <string.h>
SerialPort *SubCtrl::m_pSerialPort;

SubCtrl::SubCtrl()
{
	m_pSerialPort = NULL;
}

SubCtrl::~SubCtrl()
{
}

int SubCtrl::Init(int port, int baud, PACKETPARSE parse)
{
	m_pSerialPort = (SerialPort *)malloc(sizeof(SerialPort));

	if (m_pSerialPort == NULL) {
		return 0;
	}

	m_pSerialPort->m_port = port;
	m_pSerialPort->m_baud = baud;

	m_pSerialPort->m_hcom = -1;
	
	m_pSerialPort->m_sndflag = FALSE;
	m_pSerialPort->m_revflag = FALSE;

	sem_init(&m_pSerialPort->snd_sgl,0,0);
	sem_init(&m_pSerialPort->rev_sgl,0,0);
	sem_init(&m_pSerialPort->snd_thread_exit,0,0);
	m_pSerialPort->m_rev_process = FALSE;
	sem_init(&m_pSerialPort->hreq,0,0);
	sem_init(&m_pSerialPort->hrsp,0,0);
	m_pSerialPort->sndbuf.flag = '$';
	m_pSerialPort->sndbuf.midflag= 'T';
	m_pSerialPort->sndbuf.endflag= '#';
	
	m_pSerialPort->fnparse = parse;
    
	CircBuffAlloc(&(m_pSerialPort->circ_buff),1024);

	return 1;
}

void SubCtrl::UnInit()
{
	if (m_pSerialPort != NULL) {
		sem_destroy(&m_pSerialPort->snd_sgl);
		sem_destroy(&m_pSerialPort->rev_sgl);
		sem_destroy(&m_pSerialPort->hreq);
		sem_destroy(&m_pSerialPort->hrsp);

		if (m_pSerialPort->m_hcom != -1) {
			close(m_pSerialPort->m_hcom);		
		}		

		CircBuffFree(&(m_pSerialPort->circ_buff));
			
		free(m_pSerialPort);
		m_pSerialPort = NULL;
	}
}


int SubCtrl::open_port()
{
	char port_name[20] = {0};
	struct termios opt;

	sprintf(port_name, "/dev/ttyUSB0", m_pSerialPort->m_port);

	printf("port_name:%s \n",port_name);

	printf("enter open port function....\n");

	m_pSerialPort->m_hcom = open(port_name,O_RDWR | O_NDELAY);
	
	if (-1 == m_pSerialPort->m_hcom) {
		printf("enter open port function....return false \n");
		return FALSE;	
	}
	bzero(&opt,sizeof(struct termios));
	
	cfsetispeed(&opt,B38400);
	cfsetospeed(&opt,B38400);

	

	  opt.c_cflag &= ~CSIZE;  
    opt.c_cflag |= CS8;   
    opt.c_cflag &= ~CSTOPB; 
    opt.c_cflag &= ~PARENB; 
    opt.c_cflag &= ~INPCK;
    opt.c_cflag |= (CLOCAL | CREAD);
 
    opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
 
    opt.c_oflag &= ~OPOST;
    opt.c_oflag &= ~(ONLCR | OCRNL);    //ÌíŒÓµÄ
 
    opt.c_iflag &= ~(ICRNL | INLCR);
    opt.c_iflag &= ~(IXON | IXOFF | IXANY);    //ÌíŒÓµÄ
    
    opt.c_cc[VTIME] = 0;
    opt.c_cc[VMIN] = 0;

	tcflush(m_pSerialPort->m_hcom,TCIOFLUSH);

	printf("enter open port function. ok.....\n");

	return (tcsetattr (m_pSerialPort->m_hcom, TCSANOW, &opt));
	
}

int SubCtrl::start_running()
{

	if(-1 == open_port()) {
		printf("enter open port function. start_running false.....\n");
		return(FALSE);
	}
    	printf("enter open port function. start_running ok.....\n");
	{
	    	m_pSerialPort->m_sndflag = TRUE;
        	pthread_create(&(m_pSerialPort->snd_id),NULL,recv_thread_proc,NULL);
	}
	
	{
		pthread_attr_t attr;
        	m_pSerialPort->m_revflag = TRUE;
		pthread_attr_init(&attr);
		pthread_attr_setdetachstate(&attr,PTHREAD_CREATE_DETACHED);
	   	pthread_create(&(m_pSerialPort->rev_id),NULL,send_thread_proc,NULL);
		
	}
	
	return (TRUE);
}

void SubCtrl::stop_running()
{
	if(m_pSerialPort->m_sndflag && m_pSerialPort->m_revflag)
	{
		//send thread exit.
		fd_set rfds;
		m_pSerialPort->m_sndflag = FALSE;
		m_pSerialPort->m_sndlen = 0;
		sem_post(&m_pSerialPort->snd_sgl);
		sem_wait(&m_pSerialPort->snd_thread_exit);
		sem_destroy(&m_pSerialPort->snd_thread_exit);
	    
		m_pSerialPort->m_revflag = FALSE;

		while(m_pSerialPort->m_rev_process == TRUE) {
			sleep(10);		
		}
		printf("enter stop running exit rev thread.....\n");
		if (pthread_cancel(m_pSerialPort->rev_id) == 0) {
			pthread_join(m_pSerialPort->rev_id,NULL);
		}
		printf("enter stop running exit rev thread ok.....\n");
//		sem_wait(&m_pSerialPort->rev_thread_exit);
		
	}
	UnInit();
}

void SubCtrl::SendData(char *pcData,int nDatalen)
{
	m_pSerialPort->m_sndlen=nDatalen;
	memcpy(&m_pSerialPort->sndbuf,pcData,nDatalen);
	sem_post(&(m_pSerialPort->snd_sgl));
}

void *SubCtrl::send_thread_proc(void *lpParameter)
{
	int written_length;

	while(TRUE == m_pSerialPort->m_sndflag) 
	{
		//EnterCriticalSection(&param->cs);
	
		sem_wait(&(m_pSerialPort->snd_sgl));
	
		printf("start 2 send:%d!!!\n",m_pSerialPort->m_sndlen);
		if(m_pSerialPort->m_sndlen > 0)
		{

			printf("start 2 send:%d4444444!\n",m_pSerialPort->m_sndlen);

			if (written_length != m_pSerialPort->m_sndlen) {
				printf("start 2 send:%d8888!\n",m_pSerialPort->m_sndlen);

				write(m_pSerialPort->m_hcom, &m_pSerialPort->sndbuf, m_pSerialPort->m_sndlen);
				//tcflush(m_pSerialPort->m_hcom,TCOFLUSH);
			}
			printf("start 2 send:%d5555444!\n",m_pSerialPort->m_sndlen);
			sem_post(&(m_pSerialPort->hreq));
		}
	}
	sem_post(&(m_pSerialPort->snd_thread_exit));
}

void *SubCtrl::recv_thread_proc(void *lpParameter)
{
	int read_length = 0;
	int len = 0;
	char  read_buffer[BUFFER_LENGTH];
	unsigned char  frame[64];
	unsigned int frame_length=0;
	fd_set rfds;
	pthread_setcancelstate(PTHREAD_CANCEL_ENABLE,NULL);
	pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS,NULL);
	printf("enter thread rev_thread_proc ....\n");
	

	while(m_pSerialPort->m_revflag){
		
		int fd_sel;
		FD_ZERO(&rfds);
		FD_SET(m_pSerialPort->m_hcom,&rfds);
		

		pthread_testcancel();
		if (select(1+m_pSerialPort->m_hcom,&rfds,NULL,NULL,NULL) > 0) {
		pthread_testcancel();
		m_pSerialPort->m_rev_process = TRUE;
		printf("enter thread rev_thread_proc 2....\n");

			if (FD_ISSET(m_pSerialPort->m_hcom,&rfds)) {
				
				printf("Recv Data!!\n");
				read_length = read(m_pSerialPort->m_hcom,read_buffer,BUFFER_LENGTH);
				printf("read_length:%d!!\n",read_length);
				if (read_length > 0) {
					
					CircBuffPut(&(m_pSerialPort->circ_buff), read_buffer, read_length);

				  	if(1 == CircBuffGetFrame(&(m_pSerialPort->circ_buff),frame, 64, &frame_length)) {		
					printf("enter thread rCircBuffGetFrame ...frame_length %d.\n",frame_length);
					
					  if(m_pSerialPort->fnparse)
						  if(0 == m_pSerialPort->fnparse(frame, frame_length)){
							sem_post(&(m_pSerialPort->hrsp));
							printf("set m_pSerialPort->hrsp signal ....\n");
						  }else {
							printf("set m_pSerialPort->hrsp no signal ....\n");						
						}	

				  	}
				} 
			}		
		}
 		m_pSerialPort->m_rev_process = FALSE;
		
	}
//	sem_post(&(m_pSerialPort->rev_thread_exit));
}

/*
int SubCtrl::Callback_SS(unsigned char* buff, int buff_len)
{
int i;
	for(i=0;i<buff_len;i++)
	{

		printf("recv %d \n",buff[i]);
	}
	return 0;
}
int main()
{
	Init(10, 10, Callback_SS);
	//open_port();
	start_running();
	char cData=10;
while(1)
{
	SendData(&cData,1);
	sleep(1);
}
	return 0;
}
*/
