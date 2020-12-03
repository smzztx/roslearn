/*************************
**
** modbus-rtu-test.c 
** 移植libmodbus库到ARM开发板，并测试成功
**
**************************/
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include "modbus.h"  //modbus动态库文件


int main(int argc, char *argv[])
{
	uint16_t tab_reg[64] = {0}; //定义存放数据的数组
    modbus_t *ctx = NULL;
 
    int rc;
	int i;
							        //以串口的方式创建libmobus实例,并设置参数
	ctx = modbus_new_rtu("/dev/ttyS1", 115200, 'N', 8, 1);					
	if (ctx == NULL)                //使用UART1,对应的设备描述符为ttyS1
	{
    	fprintf(stderr, "Unable to allocate libmodbus contex\n");
    	return -1;
	}
	
	modbus_set_debug(ctx, 1);      //设置1可看到调试信息
	modbus_set_slave(ctx, 1);      //设置slave ID
	
	if (modbus_connect(ctx) == -1) //等待连接设备
	{
    	fprintf(stderr, "Connection failed:%s\n", modbus_strerror(errno));
    	return -1;
	}
	
	while (1)
	{
    	printf("\n----------------\n");
    	rc = modbus_read_registers(ctx, 0, 10, tab_reg);
    	if (rc == -1)                   //读取保持寄存器的值，可读取多个连续输入保持寄存器
    	{
			fprintf(stderr,"%s\n", modbus_strerror(errno));
			return -1;
    	}
    	for (i=0; i<10; i++)
    	{
			printf("reg[%d] = %d(0x%x)\n", i, tab_reg[i], tab_reg[i]);
    	}
		
    	usleep(3000000);
	}
    modbus_close(ctx);  //关闭modbus连接
	modbus_free(ctx);   //释放modbus资源，使用完libmodbus需要释放掉
 
	return 0;
}