#ifndef		__flash_ops_HH_H
#define		__flash_ops_HH_H

/*
	rct6 公有256KB的flash空间
	从 800 0000 到803 ffff
	每个页大小为 2KB
	我们从后面 预留出来6个页,供我们自己使用,  每个可以保存2KB的内容
*/


#define	PAGET_ADDR	0x08007000	

#define	PAGE0_ADDR	0x0803D000	
#define	PAGE1_ADDR	0x0803D800	
#define	PAGE2_ADDR	0x0803E000	
#define	PAGE3_ADDR	0x0803E800	
#define	PAGE4_ADDR	0x0803F000	
#define	PAGE5_ADDR	0x0803F800	


int flash_ops_write(int addr,int *data,int len);

int flash_ops_read(int addr,void *data,int len);


#endif
