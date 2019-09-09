#ifndef	SERIAL_OPS_H
#define	SERIAL_OPS_H

#include "stm32f1xx_hal.h"

#define	AT_OK				0
#define	AT_TIMEOUT	-51
#define	AT_BUSY			-52
#define	AT_ERR			-53

#define	AT_SENDERR	-54
#define	AT_ARGSERR  -55


typedef	unsigned char uchar;

uchar *at_get_recvbuf(void);
int at_get_recvbuf_len(void);

void AT_RECV_SET_OK(void);
void AT_RECV_RESET_OK(void);

int at_recv_is_ok(void);


int at_send_than_recv(UART_HandleTypeDef *huart,
		const char *sendbuf,int sendlen,char *recvbuf,int recvbuflen);

int at_send_than_recv_timeout(UART_HandleTypeDef *huart,
						const char *sendbuf,int sendlen,char *recvbuf,
																int recvbuflen,int timeout);

int at_send_than_recv_match_timeout(UART_HandleTypeDef *huart,
						      const char *sendbuf,int sendlen,char *recvbuf,
												int recvbuflen,char *matchstr,int timeout);
#endif
