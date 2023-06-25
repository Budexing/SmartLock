
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "string.h"
#include "flash-ops.h"
#include "serial-ops.h"
#include "wifi-ops.h"
#include "keyboard.h"
#include "OLED.h"
#include "stdlib.h"

struct user_info{
	char username[32];
	char passwd[16];
	char wifiname[16];
	char wifipasswd[16];
};
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
char uart3_rxbuf[64];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_UART5_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
int ap_mode(void);
void ap_mode_data(int);
int station_mode(void);
int station_mode_data(int);
void key_interrupt_fun(uint16_t);
int main_menu_flash(void);
int password_flash(void);
int tips_flash(void);
int time(void);

#define MAIN_MENU 0
#define PASS_MENU 1
#define TIPS_MENU 2

#define WIFI_MODE_AP 3
#define WIFI_MODE_STATION 4
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
struct user_info info;
int menu_flag=MAIN_MENU;
int temp_flag=0;
int wifi_flag;
int linkid;
int year,month,day;
int hour,min,sec;
char Y[16],M[16],D[16];
char H[16],MIN[16],S[16];
/*int yeartemp=0,monthtemp=0,daytemp=0;
int hourtemp=0,mintemp=0,sectemp=0;*/
char cmdbuf[64];
char tipsbuf[16];
char starbuf[16];
char pswdbuf[16];
char pswdprintbuf[16];
char configbuf[16];
char timeconfig[16];
char timetempbuf[6];
char rxbuf[100];
char rxprintbuf[100];
char orginpswd[16]="123456";
char orginpswdbuf[16];
char uart_rxbuf[1];
char judgebuf[16];
char funcbuf[32];
char *ptchar;
int pos=0;


void HAL_GPIO_EXTI_Callback (uint16_t GPIO_PIN)//按键中断
{
	/*按键消抖*/
	static int pre_ticks=0;
	int ticks;
	ticks=HAL_GetTick();
	if(ticks-pre_ticks<300){
		return;
	}
	pre_ticks=ticks;
	/*读取按键中断*/
	char keynum;
	keynum=Get_KeyNum(GPIO_PIN);
	if(keynum>='0'&&keynum<='9'){
		OLED_CLS();
		menu_flag=PASS_MENU;
		starbuf[pos]='*';
		pswdbuf[pos]=keynum;
		pos++;
		//beep(40);
	}
	
	if(keynum=='A'){
		OLED_CLS();
		int ret;
		menu_flag=TIPS_MENU;
		temp_flag=1;
		strcpy(tipsbuf,"ap mode");
		//ap_mode();
	}
	
	if(keynum=='C'){
		OLED_CLS();
		menu_flag=PASS_MENU;
		for(int i=0;i<=pos;i++){
		
			starbuf[i]=NULL;
			pswdbuf[i]=NULL;
		}
		pos=0;
	}
	
	if(keynum=='D'){
		OLED_CLS();
		menu_flag=PASS_MENU;
		pos--;
		starbuf[pos]=NULL;
		pswdbuf[pos]=NULL;
	}
	
	if(keynum=='E'){
		OLED_CLS();
		int ret;
		ret=strncmp(/*orginpswd*/info.passwd,pswdbuf,6);
		if(ret==0){
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
			//HAL_Delay(1000);
			//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
			//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
			menu_flag=TIPS_MENU;
			strcpy(tipsbuf,"unlock success");
			memset(starbuf,0,sizeof(starbuf));
			memset(pswdbuf,0,sizeof(pswdbuf));
			pos=0;
		}else{
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);
			//HAL_Delay(1000);
			//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
			menu_flag=TIPS_MENU;
			strcpy(tipsbuf,"unlock fail");
			memset(starbuf,0,sizeof(starbuf));
			memset(pswdbuf,0,sizeof(pswdbuf));
			pos=0;
		}
	}
	
	if(keynum=='Q'){
		OLED_CLS();
		menu_flag=MAIN_MENU;
		
		memset(starbuf,0,sizeof(starbuf));
		memset(pswdbuf,0,sizeof(pswdbuf));
		pos=0;
	}
}


int ap_mode(void)//ap模式
{

	int ret;
	ret=wifi_ap_init(&huart3);
	if(ret<0){
	
		char *ptmp = "wifi_ap_init err\n";
		HAL_UART_Transmit(&huart5,(uint8_t*)ptmp,strlen(ptmp),100);
		//HAL_UART_Transmit(&huart5,(uint8_t*)ptmp,strlen(ptmp),100);
		return -12;
	}
	HAL_UART_Transmit(&huart5,(uint8_t*)"wifi_ap_init ok\n",strlen("wifi_ap_init ok\n"),100);
	
	ret=wifi_ap_set_args(&huart3,"Manuel","12345678");
	if(ret<0){
	
		char *ptmp="wifi_ap_set_arg err\n";
		HAL_UART_Transmit(&huart5,(uint8_t*)ptmp,strlen(ptmp),100);
		return -12;
	}
	HAL_UART_Transmit(&huart5,(uint8_t *)"wifi_ap_set_args ok\n",strlen("wifi_ap_set_args ok\n"),100);
	
	/*等待客户端连接，客户端需要连接本ap�?10086端口，该函数会等待timeout ms，如果有连接到来，会返回丿个大于等于�?*/
	linkid=wifi_listen_and_wait_connect_timeout(&huart3,10086,60*1000);
	
	if(linkid<0){
		char*ptmp="wifi_listen_and_wait_connect_timeout err\n";
		HAL_UART_Transmit(&huart5,(uint8_t*)ptmp,strlen(ptmp),100);
		return -12;
	}
	HAL_UART_Transmit(&huart5,(uint8_t *)"connect ok\n",strlen("connect ok\n"),100);
	/*while(wifi_flag==WIFI_MODE_AP){
		ap_mode_data(ret);
	}*/
	//wifi_flag=WIFI_MODE_AP;
	//return ret;
}

void ap_mode_data(int ret)//ap模式下接收数�?
{

	char rxbuf[64];
	int rxlen;
	memset(rxbuf,0,sizeof(rxbuf));
	
	wifi_ap_wait_recvdata_timeout(&huart3,linkid,rxbuf,&rxlen,60*100);
	
	
	/*if(ret>=0){
		return;
	}else{*/
	ret=wifi_ap_send_data(&huart3,linkid,rxbuf,rxlen);
	//}
	
	/*---------------------进入AP模式----------------------------*/
		//如果收到的字符串前面为AP:func，则进入AP模式，将信息写入flash
		if(strncmp(rxbuf,"AP:func",7)==0){
			/*-------------------读取func倿-------------------*/
			ptchar=strstr(rxbuf,"func=");
			ptchar=ptchar+strlen("func=");
			memset(funcbuf,0,sizeof(funcbuf));
			int i=0;
			while(*ptchar!=','&&*ptchar!=';'){
				funcbuf[i]=*ptchar;
				ptchar++;
				i++;
			}
			
			if(strncmp(funcbuf,"LOGINFO",7)==0){
				/*------------------读取用户吿--------------------*/
				ptchar=strstr(rxbuf,"usr=");
				ptchar=ptchar+strlen("usr=");
				memset(info.username,0,sizeof(info.username));
				i=0;
				while(*ptchar!=','&&*ptchar!=';'){			
					info.username[i]=*ptchar;
					ptchar++;
					i++;
				}
				/*-----------------------------------------------*/
			
				/*-----------------读取用户密码------------------*/
				ptchar=strstr(rxbuf,"passwd=");
				ptchar=ptchar+strlen("passwd=");
				memset(info.passwd,0,sizeof(info.passwd));
				i=0;
				while(*ptchar!=','&&*ptchar!=';'){
				
					info.passwd[i]=*ptchar;
					ptchar++;
					i++;
				}
				/*-----------------------------------------------*/
				
				flash_ops_write(PAGE0_ADDR,(int *)&info,sizeof(info));
				HAL_UART_Transmit(&huart5,(uint8_t *)"set user success",strlen("set user success"),100);
			}else if(strncmp(funcbuf,"WIFIINFO",8)==0){
	
				/*-----------------读取wifi吿--------------------*/
				ptchar=strstr(rxbuf,"apname=");
				ptchar=ptchar+strlen("apname=");
				memset(info.wifiname,0,sizeof(info.wifiname));
				i=0;
				while(*ptchar!=','&&*ptchar!=';'){			
					info.wifiname[i]=*ptchar;
					ptchar++;
					i++;
				}
				/*-----------------------------------------------*/
				
				/*-----------------读取wifi密码------------------*/
				ptchar=strstr(rxbuf,"appass=");
				ptchar=ptchar+strlen("appass=");
				memset(info.wifipasswd,0,sizeof(info.wifipasswd));
				i=0;
				while(*ptchar!=','&&*ptchar!=';'){			
					info.wifipasswd[i]=*ptchar;
					ptchar++;
					i++;
				}
				/*-----------------------------------------------*/
				flash_ops_write(PAGE0_ADDR,(int *)&info,sizeof(info));
				HAL_UART_Transmit(&huart5,(uint8_t *)"set wifi success",strlen("set wifi success"),100);
			}
		
		/*-----------------------------------------------------------*/
		
		/*-----------------------------------------------------------*/
		}
		/*-----------------------------------------------------------*/
}

int station_mode(void)//station模式
{
	int ret;
	char sendbuf[64];
	char rxbuf[64];
	char logbuf[64];
	char gettimebuf[64];
	
	/*char userbuf[64];
	char userpassbuf[16];*/
	
	//memset(&info,0,sizeof(info));
	//flash_ops_read(PAGE0_ADDR,&info,sizeof(info));//读取flash
	/*-------------------------wifi初始�?---------------------------*/
	ret=wifi_station_init(&huart3);
	if(ret<0){
		char *ptmp="wifi_station_init err\n";
		HAL_UART_Transmit(&huart5,(uint8_t *)ptmp,strlen(ptmp),100);
		return -12;
	}
	HAL_UART_Transmit(&huart5,(uint8_t *)"init ok\n",strlen("init ok\n"),100);
	/*--------------------------------------------------------------*/

	/*--------------------------加入热点----------------------------*/
	ret=wifi_station_join_ap(&huart3,info.wifiname,info.wifipasswd);
	if(ret<0){
		char *ptmp="wifi_station_join_ap err\n";
		HAL_UART_Transmit(&huart5,(uint8_t *)ptmp,strlen(ptmp),100);
		return -12;
	}
	HAL_UART_Transmit(&huart5,(uint8_t *)"ap ok\n",strlen("ap ok\n"),100);
	/*--------------------------------------------------------------*/
	
	/*-------------------------连接服务�?---------------------------*/
	ret=wifi_station_connect(&huart3,"TCP","106.12.216.65",10066);
	if(ret<0){
		char *ptmp="wifi_station_connect err\n";
		HAL_UART_Transmit(&huart5,(uint8_t *)ptmp,strlen(ptmp),100);
		return -12;
	}
	HAL_UART_Transmit(&huart5,(uint8_t *)"connect ok\n",strlen("connect ok\n"),100);
	/*--------------------------------------------------------------*/
	
	/*--------------------------登录账户----------------------------*/
	memset(rxbuf,0,sizeof(rxbuf));
	memset(sendbuf,0,sizeof(sendbuf));
	memset(tipsbuf,0,sizeof(tipsbuf));
	
	sprintf(logbuf,"SL2SER:func=log,usr=%s,passwd=%s;",info.username,info.passwd);
	strcpy(sendbuf,logbuf);
	ret=wifi_station_send_recv_data_timeout(&huart3,sendbuf,strlen(sendbuf),rxbuf,sizeof(rxbuf),300);
	HAL_UART_Transmit(&huart5,(uint8_t *)rxbuf,strlen(rxbuf),100);
	if(ret <0){
			char *ptmp = "wifi_station_send_recv_log_timeout err\n";
			HAL_UART_Transmit(&huart5,(uint8_t *)ptmp,strlen(ptmp),100);
			return -12;
	}
	strcpy(tipsbuf,rxbuf);
	menu_flag=TIPS_MENU;
	/*--------------------------------------------------------------*/
	
	/*------------------------获取服务器时�?------------------------*/
	memset(sendbuf,0,sizeof(sendbuf));
	memset(rxbuf,0,sizeof(rxbuf));
	//memset(gettimebuf,0,sizeof(gettimebuf));
	strcpy(sendbuf,"SL2SER:func=gettime;");
	
	ret=wifi_station_send_recv_data_timeout(&huart3,sendbuf,strlen(sendbuf),rxbuf,sizeof(rxbuf),300);
	
	if(ret <0){
			char *ptmp = "wifi_station_send_recv_tim_timeout err\n";
			HAL_UART_Transmit(&huart5,(uint8_t *)ptmp,strlen(ptmp),100);
			return -12;
	}
	
	if(strncmp(rxbuf,"RESP:time",9)==0){
		int i=0;
		memset(Y,0,sizeof(Y));
		ptchar=strstr(rxbuf,"Y=");
		ptchar=ptchar+strlen("Y=");
		while(*ptchar!=','&&*ptchar!=';'){
			Y[i]=*ptchar;
			ptchar++;
			i++;
		}
		year=atoi(Y);
		
		i=0;
		memset(M,0,sizeof(M));
		ptchar=strstr(rxbuf,"M=");
		ptchar=ptchar+strlen("M=");
		while(*ptchar!=','&&*ptchar!=';'){
			M[i]=*ptchar;
			ptchar++;
			i++;
		}
		month=atoi(M);
		
		i=0;
		memset(D,0,sizeof(D));
		ptchar=strstr(rxbuf,"D=");
		ptchar=ptchar+strlen("D=");
		while(*ptchar!=','&&*ptchar!=';'){
			D[i]=*ptchar;
			ptchar++;
			i++;
		}
		day=atoi(D);
		
		i=0;
		memset(H,0,sizeof(H));
		ptchar=strstr(rxbuf,"H=");
		ptchar=ptchar+strlen("H=");
		while(*ptchar!=','&&*ptchar!=';'){
			H[i]=*ptchar;
			ptchar++;
			i++;
		}
		hour=atoi(H);
		
		i=0;
		memset(MIN,0,sizeof(MIN));
		ptchar=strstr(rxbuf,"min=");
		ptchar=ptchar+strlen("min=");
		while(*ptchar!=','&&*ptchar!=';'){
			MIN[i]=*ptchar;
			ptchar++;
			i++;
		}
		min=atoi(MIN);
		
		i=0;
		memset(S,0,sizeof(S));
		ptchar=strstr(rxbuf,"S=");
		ptchar=ptchar+strlen("S=");
		while(*ptchar!=','&&*ptchar!=';'){
			S[i]=*ptchar;
			ptchar++;
			i++;
		}
		sec=atoi(S);
		
	}
	//wifi_flag=WIFI_MODE_STATION;
	/*--------------------------------------------------------------*/
	return ret;
}

int station_mode_data(int ret)
{
	char sendbuf[64];
	char rxbuf[64];
	char gettimebuf[16];
	/*发送命令*/
	memset(rxbuf,0,sizeof(rxbuf));
	memset(sendbuf,0,sizeof(sendbuf));
	strcpy(sendbuf,"SL2SER:func=getcmd;");
	ret=wifi_station_send_recv_data_timeout(&huart3,sendbuf,strlen(sendbuf),rxbuf,sizeof(rxbuf),300);
	/*if(ret<0){
		char *ptmp = "wifi_station_send_recv_cmd_timeout err\n";
			HAL_UART_Transmit(&huart5,(uint8_t *)ptmp,strlen(ptmp),100);
			return -12;
	}*/
	
	/*判断命令状态*/
	ptchar=strstr(rxbuf,"RESP:");
	ptchar=ptchar+strlen("RESP:");
	memset(cmdbuf,0,sizeof(cmdbuf));
	int i=0;
	while(*ptchar!=','&&*ptchar!=';'){
		cmdbuf[i]=*ptchar;
		ptchar++;
		i++;
	}
	if(strncmp("NULL",cmdbuf,4)==0){
		
	}else if(strncmp("lock",cmdbuf,4)==0){
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
		strcpy(tipsbuf,"unlocked");
		menu_flag=TIPS_MENU;
		HAL_Delay(1000);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
	}else if(strncmp("unlock",cmdbuf,6)==0){
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
		strcpy(tipsbuf,"lock success");
		menu_flag=TIPS_MENU;
	}
	
	HAL_UART_Transmit(&huart5,(uint8_t *)rxbuf,strlen(rxbuf),100);
	HAL_Delay(100);
}
int main_menu_flash(void)
{
	/*HAL_Delay(1000);
	sec++;
	if(sec==60){
		sec=0;min++;
		if(min==60){
			//......
		}
	}*/
	char datebuf[16];
	char timebuf[16];
	sprintf(datebuf,"    %02d-%02d-%02d   ",year,month,day);
	sprintf(timebuf,"    %02d:%02d:%02d   ",hour,min,sec);
	OLED_ShowStr(0,0,datebuf,CODE8X16);
	OLED_ShowStr(0,16,timebuf,CODE8X16);
	OLED_ShowStr(0,32," SMARTLOCKMARKII",CODE8X16);
	OLED_ShowStr(0,64,"WIFI            ",CODE8X16);
  fflash();
}

int password_flash(void)
{
	//OLED_CLS();
	OLED_ShowStr(0,16,"  Enter passwd  ",CODE8X16);
	OLED_ShowStr(0,32,starbuf,CODE8X16);
	fflash();
}

int tips_flash(void)
{
	OLED_CLS();
	//OLED_ShowStr(0,0,rxprintbuf,CODE8X16);
	OLED_ShowStr(0,16,tipsbuf,CODE8X16);
	//OLED_ShowStr(0,32,pswdprintbuf,CODE8X16);
	fflash();
	if(strncmp("unlock success",tipsbuf,14)==0){
		//HAL_Delay(1000*2);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
	}else if(strncmp("unlock fail",tipsbuf,11)==0){
		//HAL_Delay(1000);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);	
	}else if(strncmp(" reset success ",tipsbuf,15)==0){
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
	}else if(strncmp(" wrong oldpass ",tipsbuf,15)==0){
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);
	}
}

int time(void)
{
	HAL_Delay(1000);
	sec++;
	if(sec==60){
		sec=0;min++;
		if(min==60){
			//......
		}
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//char flashbuf[128];
	int ret;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_UART5_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	Key_Bord_Init();
	OLED_Init();
	
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
	
	__HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart3,(uint8_t *)uart3_rxbuf,sizeof(uart3_rxbuf));
	
	year=2019,month=7,day=17;
	hour=16,min=38,sec=0;
	
	memset(&info,0,sizeof(info));
	flash_ops_read(PAGE0_ADDR,&info,sizeof(info));//读取flash
	if(/*strncmp(info.username,"manuel",6)!=0*/strlen(info.username)==0||strlen(info.passwd)==0||strlen(info.wifiname)==0||strlen(info.wifipasswd)==0){
		ret=ap_mode();
		while(1){
			ap_mode_data(ret);
		}
		//wifi_flag=WIFI_MODE_AP;
	}else{
		ret=station_mode();
		wifi_flag=WIFI_MODE_STATION;
	}
	
  /* USER CODE END 2 */
   int b,c,d;
	printf("hello world");
	printf("shut the fucking up");
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
  while (1)
  {
		if(temp_flag==1){
			ret=ap_mode();
			temp_flag=0;
			wifi_flag=WIFI_MODE_AP;
			ap_mode_data(ret);
		}
		station_mode_data(ret);
		switch(menu_flag){
			case MAIN_MENU:
				main_menu_flash();
				time();
			break;
			case PASS_MENU:
				password_flash();
				time();
			break;
			case TIPS_MENU:
				tips_flash();
				time();
				HAL_Delay(1000);
				menu_flag=MAIN_MENU;
			break;
		}
		
		/*switch(wifi_flag){
			case WIFI_MODE_AP:
				ap_mode_data(ret);
				break;
			case WIFI_MODE_STATION:
				station_mode_data(ret);
				break;
		}*/
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* UART5 init function */
static void MX_UART5_Init(void)
{

  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
