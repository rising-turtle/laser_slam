#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <semaphore.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <time.h>

#include "circbuff.h"

#define BUFFER_LENGTH    	0x20
#define PalVersion   		0x300

#define FALSE			0
#define TRUE			1
        
                            
typedef int (*PACKETPARSE)(unsigned char* buff, int buff_len);


typedef struct sndbuf
{
	int delaytime;
	int velocity;
	char flag;
	char direction;
	char posneg;
	char midflag;
	char endflag;	
}sendmsg;

typedef struct _SerialPort
{
	int m_hcom;	
	int m_port;
	int m_baud;

	int m_sndflag;
	int m_revflag;
	int m_sndlen;
	int m_revlen;

	//char m_sndbuf[BUFFER_LENGTH];
	sendmsg sndbuf;

	sem_t snd_sgl;
	sem_t rev_sgl;
	sem_t hreq;
	sem_t hrsp;
	sem_t snd_thread_exit;
	sem_t rev_thread_exit;
	int m_rev_process;

	pthread_t snd_id,rev_id;
	
	PACKETPARSE fnparse;
	CIRCBUFF circ_buff;
	
}SerialPort;

class SubCtrl
{
public:
	SubCtrl();
	~SubCtrl();
	int Init(int port, int baud, PACKETPARSE parse);
	int start_running();
	void stop_running();
	void UnInit();
	static void SendData(char *pcData,int nDatalen);
private:
	int open_port();
	static void *recv_thread_proc(void *lpParameter);
	static void *send_thread_proc(void *lpParameter);
	static SerialPort *m_pSerialPort;
};








