/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897(已满)  三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		main
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ790875685)
 * @version    		查看doc内version文件 版本说明
 * @Software 		MDK FOR C51 V9.60
 * @Target core		STC8H8K64S4
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-06-01
 ********************************************************************************************************************/

#include "headfile.h"
#include "Position_PID.h"
#include "PID.h"

//board.h文件中FOSC的值设置为0,则程序自动识别系统频率

/*board.h文件中FOSC的值设置不为0，则系统频率为FOSC的值，
在使用stc-isp工具下载程序的时候需要将IRC频率设置为FOSC的值*/

/*在board_init中,已经将P54引脚设置为复位
如果需要使用P54引脚,可以在board.c文件中的board_init()函数中删除SET_P54_RESRT即可*/

uint16 duty;
extern int dat;
extern float speed;

int out[2];


void vcan_sendware(void *wareaddr, uint32 waresize)
{
#define VCAN_PORT UART_1
#define CMD_WARE     3

    int16 i, tem;
    uint8 cmdf[2] = {CMD_WARE, ~CMD_WARE};
    uint8 cmdr[2] = {~CMD_WARE, CMD_WARE};

    for (i = 0; i < waresize; i += 2)
    {
        tem = ((uint8 *) wareaddr)[i];
        ((uint8 *) wareaddr)[i] = ((uint8 *) wareaddr)[i + 1];
        ((uint8 *) wareaddr)[i + 1] = tem;
    }
    uart_putbuff(VCAN_PORT, cmdf, sizeof(cmdf));
    uart_putbuff(VCAN_PORT, (uint8 *) wareaddr, waresize);
    uart_putbuff(VCAN_PORT, cmdr, sizeof(cmdr));
}


void main()
{
	DisableGlobalIRQ();	//关闭总中断
	board_init();	
	pit_timer_ms(TIM_3, 20);
	pwm_init(PWM4P_P66, 13000, 0);     //初始化PWM5  使用引脚P2.5  输出PWM频率10000HZ   占空比为百分之 pwm_duty / PWM_DUTY_MAX * 100
	P64=1;
	ctimer_count_init(CTIM0_P34);
	duty=1000;
	Position_PID_Init(26,4.2,1.2,40000);
	//PID_Init(&Motor[0],12000,6000,POSITION_PID,2,0.015,0.4);
	EnableGlobalIRQ();	//开启总中断
    while(1)
	{
		out[0]=dat*50;
		out[1]=220*50;
		//pwm_duty(PWM4P_P66, duty);
		//printf("%d %d\r\n",dat,speed);
		vcan_sendware(out,sizeof(out));
    }
}

/*
 *  @brief      ?????????? (STC8H8K64)
 *  @param      *wareaddr       ??????????
 *  @param      waresize        ???????
 *  @return     void
 *  Sample usage:               vcan_sendware(out, sizeof(out));
 *  @note       out?????    ?: int16 out[2];
 *              ?? uart_putbuff ????????,????????16????
 *              ???? int16 ??
 */

