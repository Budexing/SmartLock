
#include "stdlib.h"
#include "string.h"

#include "stm32f1xx_hal.h"
#include "serial-ops.h"
#include "wifi-ops.h"

#define	RTBUF_SIZE	64



int wifi_modue_force_reset(UART_HandleTypeDef *huart)
{
	int ret;
	char rxbuf[RTBUF_SIZE];
	char *pt_resetcmd = "AT+RST\r\n";
		
	ret = at_send_than_recv_match_timeout(huart,
				    pt_resetcmd,strlen(pt_resetcmd),rxbuf,sizeof(rxbuf),"OK",3000);	
	if(ret <0){
		return 0;
	}
	
	HAL_Delay(3000);	//wait reset success
	return 0;
}

/*******************************************
		below is wifi station function
********************************************/

int wifi_station_init(UART_HandleTypeDef *huart)
{
	int ret;
	char *ptcmd = "AT+CWMODE?\r\n";
	char *ptsinglemux_cmd = "AT+CIPMUX=0\r\n";
	char *pt_modecmd = "AT+CWMODE=1\r\n";
	
	char rxbuf[RTBUF_SIZE];
	
	//step1 : check currrent is station mode?
	ret = at_send_than_recv_match_timeout(huart,
				    ptcmd,strlen(ptcmd),rxbuf,sizeof(rxbuf),"+CWMODE:1",500);
	if(1){
		char *pt_resetcmd = "AT+RST\r\n";
		//if no in station mode,than step into station and reset it
		ret = at_send_than_recv_match_timeout(huart,
				     pt_modecmd,strlen(pt_modecmd),rxbuf,sizeof(rxbuf),"OK",800);
		ret = at_send_than_recv_match_timeout(huart,
				    pt_resetcmd,strlen(pt_resetcmd),rxbuf,sizeof(rxbuf),"OK",3000);	
		
		HAL_Delay(3000);	//wait reset success
	}

	ret = at_send_than_recv_match_timeout(huart,
				    ptsinglemux_cmd,strlen(ptsinglemux_cmd),rxbuf,sizeof(rxbuf),"OK",1000);
	if(0 ){
		return -15;
	}
	
	ret = wifi_station_disconnect(huart);
	if(0 ){
		return -16;
	}	
	
	ret = wifi_station_exit_ap(huart);
	if(0 ){
		return -18;
	}		
	// in station mode
	return 0;

}

int wifi_station_join_ap(UART_HandleTypeDef *huart,
															char *apname,char *passwd)
{
	int ret;
	char cmdbuf[RTBUF_SIZE];
	char rxbuf[RTBUF_SIZE];
	
	if(!huart || !apname || !passwd){
		return -12;
	}
#if		0
	/*	加入 ap之前,首先要退出 ap		:	AT+CWQAP  返回  OK	*/
	snprintf(cmdbuf,sizeof(cmdbuf),"AT+CWQAP\r\n");		
	ret = at_send_than_recv_match_timeout(huart,
				  cmdbuf,strlen(cmdbuf),rxbuf,sizeof(rxbuf),"OK",200);
	if(ret < 0){
		return -13;
	}
#endif
	
	//join ap:  AT+CWJAP="TP-LINK_34C2","HQYJCQ2017"
	snprintf(cmdbuf,sizeof(cmdbuf),"AT+CWJAP=\"%s\",\"%s\"\r\n",apname,passwd);		
	ret = at_send_than_recv_match_timeout(huart,
				  cmdbuf,strlen(cmdbuf),rxbuf,sizeof(rxbuf),"WIFI GOT IP",5000);
	if(ret < 0){
		return -14;
	}

	/*我们测试发现   加入ap之后,需要延迟300ms 才能获取 ip,连接等后续功能  */
	HAL_Delay(350);
	
	return 0;
}

int wifi_station_connect(UART_HandleTypeDef *huart,
											char *nettype,char *ip,int port)
{
	int ret;
	char cmdbuf[RTBUF_SIZE];
	char rxbuf[RTBUF_SIZE];
	
	if(!huart || !nettype || !ip || !port){
		return -12;
	}
	
	/*获取本地 ip地址*/
	strcpy(cmdbuf,"AT+CIFSR\r\n");
	ret = at_send_than_recv_match_timeout(huart,
		cmdbuf,strlen(cmdbuf),rxbuf,sizeof(rxbuf),"+CIFSR:",8000);
	if(ret < 0){
		return -25;
	}
	
	//	AT+CIPSTART="TCP","192.168.0.114",1010		返回值 CONNECT  ...  OK  or busy p...
	snprintf(cmdbuf,sizeof(cmdbuf),
		"AT+CIPSTART=\"%s\",\"%s\",%d\r\n",	nettype,ip,port);		
	ret = at_send_than_recv_match_timeout(huart,
				  cmdbuf,strlen(cmdbuf),rxbuf,sizeof(rxbuf),"OK",8000);
	if(ret < 0){
		return ret;
	}
	
	return 0;
#if		0
	if(strstr(rxbuf,"OK")){
		return 0;
	}else {
		return -15;
	}
	
	return -17;
#endif
}


int wifi_station_send_data(UART_HandleTypeDef *huart,char *sendbuf,int sendlen)
{
	int ret;
	char cmdbuf[RTBUF_SIZE];
	char rxbuf[RTBUF_SIZE];
	
	if(!huart || !sendbuf || sendlen <=0 ){
		return -12;
	}
	
	//	AT+CIPSEND=len   模块返回 >, 持续写入len长度的数据即可
	snprintf(cmdbuf,sizeof(cmdbuf),
		"AT+CIPSEND=%d\r\n",sendlen);		
	ret = at_send_than_recv_match_timeout(huart,
				  cmdbuf,strlen(cmdbuf),rxbuf,sizeof(rxbuf),">",20);
	if(ret < 0){
		return ret;
	}
	
	ret = at_send_than_recv_match_timeout(huart,
				  sendbuf,sendlen,rxbuf,sizeof(rxbuf),"SEND OK",500);
	if(ret < 0){
		return ret;
	}
	
	return 0;
}

/*
	服务器工作模式是被动的,
	只有当收到 client请求的时候才会 向客户端发送数据, 
	所以当client发送数据 的时候,紧接着就会收到来自服务器的数据
	client发送和接受应该是一起的(要求服务器配合)

客户端在一个时刻可能会收到 多种数据,如何识别服务器发来的 数据
要求服务器每一条数据 都要有 一个前缀 "SERV:"+随机值

随机数是client产生,发送给服务器,服务器在回应数据的时候,将该随机数 发过来
表示是对本次 请求的应答

*/
int wifi_station_send_recv_data_timeout(UART_HandleTypeDef *huart,
						char *sendbuf,int sendlen,char *recvbuf,int recvlen,int timeout)
{
	int ret;
	char cmdbuf[RTBUF_SIZE];
	char rxbuf[RTBUF_SIZE*2];
	char *pt_start;
	
	if(!huart || !sendbuf || sendlen <=0 ||   RTBUF_SIZE*2 < recvlen){
		return -10;
	}
	
	//	AT+CIPSEND=len   模块返回 >, 持续写入len长度的数据即可
	snprintf(cmdbuf,sizeof(cmdbuf),
		"AT+CIPSEND=%d\r\n",sendlen);		
	ret = at_send_than_recv_match_timeout(huart,
				  cmdbuf,strlen(cmdbuf),rxbuf,sizeof(rxbuf),">",50);
	if(ret < 0){
		return -11;
	}
	
#if    1	
	ret = at_send_than_recv_match_timeout(huart,
				sendbuf,sendlen,rxbuf,sizeof(rxbuf),"+IPD,",timeout);
	if(ret < 0){
		return -12;
	}

	//拷贝 需要的数据到 参数缓存
	pt_start = strstr(rxbuf,":");
	if(!pt_start){
		return -13;
	}
	pt_start++;
	
	memcpy(recvbuf,pt_start,recvlen);
	
	/*等待接收数据	*/
	#if		0
	ret = at_send_than_recv_match_timeout(huart,
						NULL,0,recvbuf,recvlen,"+IPD",timeout);
	if(ret < 0){
		return ret;
	}
	#endif
	
#endif
	
	return 0;
}


int wifi_station_disconnect(UART_HandleTypeDef *huart)
{
	int ret;
	char cmdbuf[RTBUF_SIZE];
	char rxbuf[RTBUF_SIZE];
	
	if(!huart  ){
		return -12;
	}
	
	//	AT+CIPCLOSE  返回 CLOSED OK
	snprintf(cmdbuf,sizeof(cmdbuf),"AT+CIPCLOSE\r\n");		
	ret = at_send_than_recv_match_timeout(huart,
				  cmdbuf,strlen(cmdbuf),rxbuf,sizeof(rxbuf),"OK",200);
	if(ret < 0){
		return ret;
	}
	
	return 0;
}

int wifi_station_exit_ap(UART_HandleTypeDef *huart)
{
	int ret;
	char cmdbuf[RTBUF_SIZE];
	char rxbuf[RTBUF_SIZE];
	
	if(!huart  ){
		return -12;
	}
	
	//	AT+CWQAP  返回  OK
	snprintf(cmdbuf,sizeof(cmdbuf),"AT+CWQAP\r\n");		
	ret = at_send_than_recv_match_timeout(huart,
				  cmdbuf,strlen(cmdbuf),rxbuf,sizeof(rxbuf),"OK",2000);
	if(ret < 0){
		return 0;
	}
	
	return 0;
}


/*################################
		below is ap mode
#################################*/

int wifi_ap_init(UART_HandleTypeDef *huart)
{
	int ret;
	char *ptcmd = "AT+CWMODE?\r\n";
	
	char rxbuf[RTBUF_SIZE];
	
	//step1 : check currrent is station mode?
	ret = at_send_than_recv_match_timeout(huart,
				    ptcmd,strlen(ptcmd),rxbuf,sizeof(rxbuf),"+CWMODE:2",50);
	if(ret < 0){
		char *pt_modecmd = "AT+CWMODE=2\r\n";
		char *pt_resetcmd = "AT+RST\r\n";
		
		//if no in station mode,than step into station and reset it
		ret = at_send_than_recv_match_timeout(huart,
				     pt_modecmd,strlen(pt_modecmd),rxbuf,sizeof(rxbuf),"OK",50);
		ret = at_send_than_recv_match_timeout(huart,
				    pt_resetcmd,strlen(pt_resetcmd),rxbuf,sizeof(rxbuf),"OK",50);	
		
		HAL_Delay(3000);	//wait reset success
	}

	// in station mode
	return 0;

}

int wifi_ap_set_args(UART_HandleTypeDef *huart,
				char *apname,char *passwd)
{

	int ret;
	char cmdbuf[RTBUF_SIZE];
	char rxbuf[RTBUF_SIZE];
	char *ptsinglemux_cmd = "AT+CIPMUX=1\r\n";
	

	if(!huart || !apname || !passwd){
		return -12;
	}
	
	//start/set ap: AT+CWSAP="apname","passwd",6,2
	snprintf(cmdbuf,sizeof(cmdbuf),"AT+CWSAP=\"%s\",\"%s\",6,2\r\n",apname,passwd);		
	ret = at_send_than_recv_match_timeout(huart,
				  cmdbuf,strlen(cmdbuf),rxbuf,sizeof(rxbuf),"OK",3000);
	if(ret < 0){
		return ret;
	}
	
	//setup mul mode
	ret = at_send_than_recv_match_timeout(huart,
				    ptsinglemux_cmd,strlen(ptsinglemux_cmd),rxbuf,sizeof(rxbuf),"OK",50);
	if(0 ){
		return ret;
	}
	
	return 0;
}


/*  */
int wifi_ap_listen(UART_HandleTypeDef *huart,int port)
{
	int ret;
	char cmdbuf[RTBUF_SIZE];
	char rxbuf[RTBUF_SIZE];
	
	if(!huart  || port <=0){
		return -12;
	}
	
	//	AT+CIPSERVER=1,1111	 返回OK
	snprintf(cmdbuf,sizeof(cmdbuf),"AT+CIPSERVER=1,%d\r\n",port);		
	ret = at_send_than_recv_match_timeout(huart,
				  cmdbuf,strlen(cmdbuf),rxbuf,sizeof(rxbuf),"OK",2000);
	if(ret < 0){
		return ret;
	}
	
	return 0;
}


/*	return link no	*/
int wifi_ap_waitfor_connect(UART_HandleTypeDef *huart,int timeout)
{
	int ret;
	int linkid;
	char rxbuf[RTBUF_SIZE];
	
	if(!huart ){
		return -12;
	}
	
	//	AT+CIPSERVER=1,1111	 返回OK
	ret = at_send_than_recv_match_timeout(huart,
				  NULL,0,rxbuf,sizeof(rxbuf),"CONNECT",timeout);
	if(ret < 0){
		return ret;
	}
	
	linkid = rxbuf[0] - '0';

	return linkid;
}


/*	return link no	*/
int wifi_listen_and_wait_connect_timeout
			(UART_HandleTypeDef *huart,int port,int timeout)
{
	int ret;
	
	ret = wifi_ap_listen(huart,port);
	if(ret <0){
		return ret;
	}

	ret = wifi_ap_waitfor_connect(huart,timeout);
	if(ret <0){
		return ret;
	}	
	
	return ret;
}


int wifi_ap_wait_recvdata_timeout(UART_HandleTypeDef *huart,
									int linkno,char *rxbuf,int *rxlen,int timeout)
{
	int ret;
	char Lrxbuf[RTBUF_SIZE];
	
	char *ptrchar;
	int Lrxlen;
	
	if(!huart ){
		return -12;
	}
	
	//	+IPD,cli_id,datalen:content   返回OK
	//	example:		+IPD,0,14:I am server!
	ret = at_send_than_recv_match_timeout(huart,
				  NULL,0,Lrxbuf,sizeof(Lrxbuf),"+IPD,",timeout);
	if(ret < 0){
		return ret;
	}
	
	//get datalen
	ptrchar = strstr(Lrxbuf,",");
	if(!ptrchar){
		return -14;
	}
	ptrchar++;ptrchar++;ptrchar++;
	
	Lrxlen = atol(ptrchar);
	*rxlen = Lrxlen;
	
	ptrchar = strstr(Lrxbuf,":");
	if(!ptrchar){
		return -16;
	}
	ptrchar++;
	memcpy(rxbuf,ptrchar,Lrxlen);
	
	return 0;	
}

int wifi_ap_send_data(UART_HandleTypeDef *huart,int linkid,char *sendbuf,int sendlen)
{
	int ret;
	char cmdbuf[RTBUF_SIZE];
	char rxbuf[RTBUF_SIZE];
	
	if(!huart || !sendbuf || sendlen <=0 ||linkid <0){
		return -12;
	}
	
	//	AT+CIPSEND=id,len   模块返回 >, 持续写入len长度的数据即可
	snprintf(cmdbuf,sizeof(cmdbuf),
		"AT+CIPSEND=%d,%d\r\n",linkid,sendlen);		
	ret = at_send_than_recv_match_timeout(huart,
				  cmdbuf,strlen(cmdbuf),rxbuf,sizeof(rxbuf),">",20);
	if(ret < 0){
		return ret;
	}
	
	ret = at_send_than_recv_match_timeout(huart,
				  sendbuf,sendlen,rxbuf,sizeof(rxbuf),"SEND OK",500);
	if(ret < 0){
		return ret;
	}
	
	return 0;
}


