
#ifndef _CIRCBUFF_H_
#define _CIRCBUFF_H_

#include <pthread.h>

#define u_int32_t unsigned int
#define u_int8_t  unsigned char

#define FAILURE   1
#define SUCCESS   0
//define circle buffer


typedef struct _CIRCBUFF{
	u_int8_t      *buff;
	u_int32_t     size;		// size of the buffer
	u_int32_t     count;		// current count of bytes in the buffer
	u_int8_t      *StartPtr;	// points to first element (fix)
	u_int8_t      *EndPtr;	// points after last element (fix)
    u_int8_t      *ReadPtr;    // points to next buffer to read
	u_int8_t      *WritePtr;   // points to next buffer to write
	u_int8_t      FullFlag;	// true if buffer is full
	u_int8_t      EmptyFlag;	// true if buffer is empty
	u_int32_t     sum;        // statistic write data
//	CRITICAL_SECTION CS;    // sync object
	pthread_mutex_t mutex;
}CIRCBUFF;


class CircBuff
{
public:
	CircBuff
	u_int8_t CircBuffAlloc(CIRCBUFF *pCircBuff, const u_int32_t size);
	u_int8_t CircBuffFree(CIRCBUFF *pCircBuff);

	u_int32_t CircBuffPut(CIRCBUFF *pCircBuff, void *buffer, const u_int32_t size);
	u_int32_t CircBuffGet(CIRCBUFF *pCircBuff, void *buffer, const u_int32_t size);
	u_int32_t CircBuffGetFrame(CIRCBUFF *pCircBuff, void *buffer, const u_int32_t size,u_int32_t *frame_length);

	u_int32_t CircRoomLeft(CIRCBUFF *pCircBuff);
	u_int8_t CircBuffReset(CIRCBUFF *pCircBuff);
}



#endif
