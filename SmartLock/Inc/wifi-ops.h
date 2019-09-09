#ifndef WIFI_OPS_HHHHH
#define WIFI_OPS_HHHHH


#include "stm32f1xx_hal.h"




int wifi_modue_force_reset(UART_HandleTypeDef *huart);


/********************************************************
					wifi station function
********************************************************/
int wifi_station_init(UART_HandleTypeDef *huart);

int wifi_station_join_ap(UART_HandleTypeDef *huart,
												   char *apname,char *passwd);

int wifi_station_connect(UART_HandleTypeDef *huart,
											char *nettype,char *ip,int port);

int wifi_station_send_data(UART_HandleTypeDef *huart,
															char *sendbuf,int sendlen);

int wifi_station_send_recv_data_timeout(UART_HandleTypeDef *huart,
						char *sendbuf,int sendlen,char *recvbuf,int recvlen,int timeout);


int wifi_station_disconnect(UART_HandleTypeDef *huart);

int wifi_station_exit_ap(UART_HandleTypeDef *huart);


/********************************************************
					wifi AP function
********************************************************/
int wifi_ap_init(UART_HandleTypeDef *huart);

int wifi_ap_set_args(UART_HandleTypeDef *huart,
				char *apname,char *passwd);

/*返回 连接号, 0-xxx    err:负值*/
int wifi_ap_listen(UART_HandleTypeDef *huart,int port);

int wifi_ap_waitfor_connect(UART_HandleTypeDef *huart,int timeout);

int wifi_listen_and_wait_connect_timeout
			(UART_HandleTypeDef *huart,int port,int timeout);

int wifi_ap_wait_recvdata_timeout(UART_HandleTypeDef *huart,
									int linkno,char *rxbuf,int *rxlen,int timeout);

int wifi_ap_send_data(UART_HandleTypeDef *huart,int linkid,char *sendbuf,int sendlen);


#endif
