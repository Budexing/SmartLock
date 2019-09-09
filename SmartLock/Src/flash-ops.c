#include "stm32f1xx_hal.h"

#include "flash-ops.h"

/*
	每次写入 都是 4B,要求 data是4B对齐的,即len是4的整数倍
*/
int flash_ops_write(int addr,int *data,int len)
{
	int i;
	int ret;
	HAL_StatusTypeDef status;
	
	if( (len%4) != 0){
		int mode = len%4;
		len -= mode;
		len+=4;
	}
	
	/*1 解锁*/
	HAL_FLASH_Unlock();
	
	/*2 擦除*/
	FLASH_EraseInitTypeDef f; 					//初始化FLASH_EraseInitTypeDef 
	f.TypeErase = FLASH_TYPEERASE_PAGES; 
	f.PageAddress = addr; 
	f.NbPages = 1;
  uint32_t PageError = 0;							//设置PageError
	status = HAL_FLASHEx_Erase(&f, &PageError);	//调用擦除函数
	if(status != HAL_OK  ){
		ret = -12;
		goto end;
	}
	
	/*3 编程 烧写	*/
	for(i=0;i<len/4;i+=1){
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,addr+i*4, data[i]);
	}
	
	ret = 0;
	
end:	
	/*4 锁住*/
	HAL_FLASH_Lock();	
	
	return ret;
}

int flash_ops_read(int addr,void *data,int len)
{
	int i;
	
	volatile char *ptaddr = (__IO char *)addr;
	char *ptdata = (char *)data;
	
	for(i=0;i<len;i++){
		ptdata[i] = ptaddr[i];
	}
	
	return 0;
}
