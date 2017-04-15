//==================================================================
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "circbuff.h"



//allocate circle buff and initialize CircBuff
u_int8_t CircBuffAlloc(CIRCBUFF *CircBuff, const u_int32_t size)
{
	int ret = FAILURE;

	if(size <= 0) {
		//DEBUGMSG (ZONE_FUNCTION | ZONE_ERROR , 
		//	    (TEXT(" CircBuffAlloc (Invalid param size)\r\n")));

		return (ret);
	}

	//allocate buffer
	CircBuff->buff = (u_int8_t*)malloc(size+1);
	if(CircBuff->buff == NULL) {
		//DEBUGMSG (ZONE_FUNCTION | ZONE_ERROR , 
		//	    (TEXT(" CircBuffAlloc (Allocate buffer failure)\r\n")));

		return ret;
	}

	memset(CircBuff->buff, 0, size+1);

	//initialize sync object

	pthread_mutex_init(&(CircBuff->mutex),NULL);

	pthread_mutex_lock(&(CircBuff->mutex));

	//initize circle buffer
	CircBuff->size      = size;
	CircBuff->count     = 0; 
	CircBuff->StartPtr  = CircBuff->buff;
	CircBuff->EndPtr    = CircBuff->StartPtr + size + 1;

	CircBuff->ReadPtr   = CircBuff->buff;
	CircBuff->WritePtr  = CircBuff->buff;
	CircBuff->EmptyFlag = 1;			
	CircBuff->FullFlag  = 0;
	CircBuff->sum       = 0;

	pthread_mutex_unlock(&(CircBuff->mutex));
 
	return (ret == SUCCESS);
    
}

//Free circle buffer
u_int8_t CircBuffFree (CIRCBUFF *CircBuff)
{
	pthread_mutex_lock(&(CircBuff->mutex));

	if(CircBuff->buff != NULL) {

	    free(CircBuff->buff);
	    CircBuff->buff = NULL;
	}

	pthread_mutex_unlock(&(CircBuff->mutex));

	pthread_mutex_destroy(&(CircBuff->mutex));

	return (SUCCESS);
}

u_int8_t CircBuffReset(CIRCBUFF *CircBuff)
{
	pthread_mutex_lock(&(CircBuff->mutex));

	memset(CircBuff->buff, 0, CircBuff->size);

	//initize circle buffer
	CircBuff->count     = 0; 

	CircBuff->ReadPtr   = CircBuff->WritePtr;
	CircBuff->EmptyFlag = 1;			
	CircBuff->FullFlag  = 0;

	pthread_mutex_unlock(&(CircBuff->mutex));

	return (SUCCESS);
}

u_int32_t CircRoomLeft(CIRCBUFF *CircBuff)
{
	return (CircBuff->size - CircBuff->count);
}

//put data to CircBuff
u_int32_t CircBuffPut(CIRCBUFF *CircBuff, void *buffer, const u_int32_t size)
{	
	u_int32_t wrcnt, maxcnt, DiffToEnd;
	u_int8_t *pSrc = (u_int8_t*)buffer;

	if(size == 0 || buffer == NULL || CircBuff->FullFlag) {

		//DEBUGMSG (ZONE_FUNCTION | ZONE_ERROR , 
		//	    (TEXT(" CircBuffPut (Invalid params)\r\n")));

		return (FAILURE);
	}

	pthread_mutex_lock(&(CircBuff->mutex));

	//make sure the size of CircBuff that can be use
	maxcnt = CircBuff->size - CircBuff->count;
	if(size > maxcnt) {
		wrcnt = maxcnt;
	} else {
		wrcnt = size;
	}

	//add data to circle buffer
	DiffToEnd = (u_int32_t)(CircBuff->EndPtr - CircBuff->WritePtr);

    if(wrcnt <= DiffToEnd) {

		memcpy(CircBuff->WritePtr, pSrc, wrcnt);
		CircBuff->WritePtr += wrcnt;

	} else {

		memcpy(CircBuff->WritePtr, pSrc, DiffToEnd);
		pSrc = pSrc + DiffToEnd;

		memcpy(CircBuff->StartPtr, pSrc, wrcnt-DiffToEnd);
		CircBuff->WritePtr = CircBuff->StartPtr + wrcnt - DiffToEnd;

	}

	if (CircBuff->WritePtr >= CircBuff->EndPtr){
		CircBuff->WritePtr = CircBuff->StartPtr;	
	}
	

	//count the data of circle buffer
	CircBuff->count += wrcnt;

	//set flag
	CircBuff->EmptyFlag = 0;

	if(CircBuff->count == CircBuff->size) {

		//DEBUGMSG (ZONE_FUNCTION  , 
		//	    (TEXT(" CircBuffPut (Circle buffer is full!)\r\n")));

		CircBuff->FullFlag = 1;
	}

	CircBuff->sum += wrcnt;

	pthread_mutex_unlock(&(CircBuff->mutex));


	return (wrcnt);

}

//read data from circle buffer
u_int32_t CircBuffGet(CIRCBUFF *CircBuff, void *buffer, const u_int32_t size)
{
	u_int32_t rdsize, DiffToEnd;
	u_int8_t *pDst = (u_int8_t*)buffer;

	if(size == 0 || buffer == NULL) {

		//DEBUGMSG (ZONE_FUNCTION | ZONE_ERROR , 
		//	    (TEXT(" CircBuffGut (Invalid params)\r\n")));

		return (FAILURE);
	}

	pthread_mutex_lock(&(CircBuff->mutex));

	//make sure the size of read from circle buffer
	if(size > CircBuff->count) {

		rdsize = CircBuff->count;

	} else {

		rdsize = size;

	}

	DiffToEnd = (u_int32_t)(CircBuff->EndPtr - CircBuff->ReadPtr);

	if(rdsize <= DiffToEnd) {

		memcpy(pDst, CircBuff->ReadPtr, rdsize);
		CircBuff->ReadPtr += rdsize;

	} else {

		memcpy(pDst, CircBuff->ReadPtr, DiffToEnd);
		pDst += DiffToEnd;
		memcpy(pDst, CircBuff->StartPtr, rdsize - DiffToEnd);

		CircBuff->ReadPtr = CircBuff->StartPtr + rdsize - DiffToEnd;
	}

	if (CircBuff->ReadPtr >= CircBuff->EndPtr) {

		CircBuff->ReadPtr = CircBuff->StartPtr;	

	}
	
	
	//set flag
	CircBuff->count -= rdsize;
	CircBuff->FullFlag = 0;

	if(CircBuff->count == 0) {

		//DEBUGMSG (ZONE_FUNCTION  , 
		//	    (TEXT(" CircBuffPut (Circle buffer is empty!)\r\n")));


		CircBuff->EmptyFlag = 1;
	}

	pthread_mutex_unlock(&(CircBuff->mutex));

    return (rdsize);

}

//read data from circle buffer
u_int32_t CircBuffGetFrame(CIRCBUFF *CircBuff, void *buffer, const u_int32_t buff_size, u_int32_t *frame_length)
{
	static u_int8_t  frame[64];
	static u_int32_t  frame_index = 0;
	static u_int8_t head = 0, tail = 0;
	static u_int32_t packet_length = 0, packet_left = 0;
	u_int32_t i = 0, ret =0;

	u_int8_t *pDst = (u_int8_t*)buffer;

	if(buff_size == 0 || buffer == NULL) {

		//DEBUGMSG (ZONE_FUNCTION | ZONE_ERROR , 
		//	    (TEXT(" CircBuffGut (Invalid params)\r\n")));

		return (FAILURE);
	}

	pthread_mutex_lock(&(CircBuff->mutex));


	while(i<CircBuff->count){
		if(head == 0 && (*CircBuff->ReadPtr) == 0xAB){
			memset(frame, 0, sizeof(frame));
			frame_index = 0;
			head = 0;
			tail =0; 
			packet_length = 0;
			packet_left = 0;

			frame[frame_index++] = *(CircBuff->ReadPtr);
			head = 1;
		} else if(head ==1 && packet_length == 0){
			packet_length = 3 + (*(CircBuff->ReadPtr));
			frame[frame_index++] = *(CircBuff->ReadPtr);
			packet_left = packet_length-2;
		}else if(head==1 && packet_length != 0){
			packet_left = packet_left-1;
			if(packet_left >= 0){
				frame[frame_index++] = *(CircBuff->ReadPtr);
				if(packet_left==0)
					tail = 1;
			}
			
		}
		i++;

		CircBuff->ReadPtr++;
		if (CircBuff->ReadPtr >= CircBuff->EndPtr) {
			CircBuff->ReadPtr = CircBuff->StartPtr;	
		}
       
		if(head == 1 && tail ==1)
			break;
	}
	
	CircBuff->count -= i;
	CircBuff->FullFlag = 0;

	if(CircBuff->count == 0) {

		//DEBUGMSG (ZONE_FUNCTION  , 
		//	    (TEXT(" CircBuffPut (Circle buffer is empty!)\r\n")));
		CircBuff->EmptyFlag = 1;
	}
	if(head == 1 && tail == 1){
		memcpy(pDst, frame, frame_index);		
		*frame_length = frame_index;
		head = 0;
		ret = 1;
	}
	pthread_mutex_unlock(&(CircBuff->mutex));

   return (ret);

}
