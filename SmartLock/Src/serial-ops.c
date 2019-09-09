#include "stm32f1xx_hal.h"
#include "string.h"
#include "serial-ops.h"


volatile uchar AT_RECV_OK_FLAGS;
uchar at_recvbuf[64];

uint8_t at_rxlen;

uchar *at_get_recvbuf(void)
{
	return (uchar *)at_recvbuf;
}

int at_get_recvbuf_len(void)
{
	return sizeof(at_recvbuf)/sizeof(at_recvbuf[0]);
}

void AT_RECV_SET_OK(void)
{
	AT_RECV_OK_FLAGS = 1;
}

void AT_RECV_RESET_OK(void)
{
	AT_RECV_OK_FLAGS = 0;
}


int at_recv_is_ok(void)
{
	return AT_RECV_OK_FLAGS ;
}


int at_send_than_recv(UART_HandleTypeDef *huart,
		const char *sendbuf,int sendlen,char *recvbuf,int recvbuflen)
{
#define	CNT_MAX   20
	int cnt;
	
	HAL_StatusTypeDef ret;

	if(!huart || !sendbuf  ){
		return -12;
	}

	AT_RECV_RESET_OK();
	
	ret = HAL_UART_Transmit(huart,(uchar *)sendbuf,sendlen,100);
//	ret = HAL_UART_Transmit(huart,(uchar *)"AT+VERSION\r\n",strlen("AT+VERSION\r\n"),100);
	if(ret != HAL_OK){
		return -15;
	}
	
	for(cnt = 0;cnt < CNT_MAX;cnt++){
		if(at_recv_is_ok() ){
			break;
		}
		HAL_Delay(1);
	}

	if(cnt == CNT_MAX){
		return AT_TIMEOUT;
	}
	
	/*data have been store int at_recvbuf*/
	if((uchar *)recvbuf != (uchar *)at_recvbuf){
		memcpy(recvbuf,at_recvbuf,sizeof(at_recvbuf));
	}

	return cnt;
}

/*
	own timerout function in ms
	contain  only send no recv  &&   only recv no send
*/
int at_send_than_recv_timeout(UART_HandleTypeDef *huart,
						const char *sendbuf,int sendlen,char *recvbuf,
																int recvbuflen,int timeout)
{
	int cnt = 0;
	HAL_StatusTypeDef ret;

	if(!huart){
		return AT_ARGSERR;
	}
	
	//allow only send no recv
	if(sendbuf && sendlen>0 ){
		ret = HAL_UART_Transmit_IT(huart,(uchar *)sendbuf,sendlen);
		if(ret != HAL_OK){
			return AT_SENDERR;
		}
	}
	//if no need recv,than success
	if(!recvbuf || recvbuflen<=0 || timeout<0 ){
		return AT_OK;		//cnt is also equeue 0
	}
	
	AT_RECV_RESET_OK();		
	//if allow recv ,below code 
	for(cnt = 0;cnt < timeout;cnt++){
		if(at_recv_is_ok() ){
#if		0
			char rbuf[32];
			sprintf(rbuf,"fun:%s len%d",at_recvbuf,at_rxlen);
			OLED_ShowStr(0,32,(uint8_t *)rbuf,1);
			fflash();
#endif
			break;
		}
		HAL_Delay(1);
	}

	if(cnt == timeout){
		return AT_TIMEOUT;
	}
	
	/*data have been store int at_recvbuf*/
	if((uchar *)recvbuf != (uchar *)at_recvbuf){
		memcpy(recvbuf,at_recvbuf,sizeof(at_recvbuf));
	}

	return cnt;		/*return time use in ms*/
}


/*
	own timerout function in ms
	contain  only send no recv  &&   only recv no send
*/
int at_send_than_recv_match_timeout(UART_HandleTypeDef *huart,
						      const char *sendbuf,int sendlen,char *recvbuf,
												int recvbuflen,char *matchstr,int timeout)
{
	#define	TIME_PER_IN_MS		1
	
	int cnt = 0;
	int cmpsize;
	HAL_StatusTypeDef ret;

	if(!huart){
		return AT_ARGSERR;
	}
	
	timeout = timeout/TIME_PER_IN_MS;
	
	//allow only send no recv
	if(sendbuf && sendlen>0 ){
		ret = HAL_UART_Transmit_IT(huart,(uchar *)sendbuf,sendlen);
		if(ret != HAL_OK){
			return AT_SENDERR;
		}
	}
	//if no need recv,than success
	if(!recvbuf || recvbuflen<=0 || timeout<0 || !matchstr){
		return AT_OK;		//cnt is also equeue 0
	}
	

	//if allow recv ,below code 
	AT_RECV_RESET_OK();		
	//enable uart  recv interrupt dma
	HAL_UART_Receive_DMA(huart,(uint8_t *)at_recvbuf,at_get_recvbuf_len());
	
	for(cnt = 0;cnt < timeout;cnt++){

		if(at_recv_is_ok() ){
			
#if		0
			char rbuf[64];
			snprintf(rbuf,21,"op:%s len%d",at_recvbuf,at_rxlen);
			rbuf[15] = '\0';
			OLED_ShowStr(0,32,(uint8_t *)at_recvbuf,1);
			fflash();
#endif
						
			//get serial data,and store in char at_recvbuf[64]
			if(strstr( (char *)at_recvbuf, matchstr)){
				break;		//got it ,exit loop
			}
			// here mean at_recvbuf is not this time need,deal it by others
			int at_resp_other_handle(char *at_resp);
			
			//than wait next resp
			AT_RECV_RESET_OK();	
			HAL_UART_Receive_DMA(huart,(uint8_t *)at_recvbuf,at_get_recvbuf_len());

		}
		HAL_Delay(TIME_PER_IN_MS);
	}

	if(cnt == timeout){
		return AT_TIMEOUT;
	}
	
	/*data have been store int at_recvbuf*/
	cmpsize = recvbuflen > sizeof(at_recvbuf) ? sizeof(at_recvbuf):recvbuflen;
	memcpy(recvbuf,at_recvbuf,cmpsize);

	return cnt*TIME_PER_IN_MS;		/*return time use in ms*/
}



