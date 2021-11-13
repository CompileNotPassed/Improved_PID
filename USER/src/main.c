/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897(����)  ��Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		main
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ790875685)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		MDK FOR C51 V9.60
 * @Target core		STC8H8K64S4
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-06-01
 ********************************************************************************************************************/

#include "headfile.h"
#include "Position_PID.h"
#include "PID.h"

//board.h�ļ���FOSC��ֵ����Ϊ0,������Զ�ʶ��ϵͳƵ��

/*board.h�ļ���FOSC��ֵ���ò�Ϊ0����ϵͳƵ��ΪFOSC��ֵ��
��ʹ��stc-isp�������س����ʱ����Ҫ��IRCƵ������ΪFOSC��ֵ*/

/*��board_init��,�Ѿ���P54��������Ϊ��λ
�����Ҫʹ��P54����,������board.c�ļ��е�board_init()������ɾ��SET_P54_RESRT����*/

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
	DisableGlobalIRQ();	//�ر����ж�
	board_init();	
	pit_timer_ms(TIM_3, 20);
	pwm_init(PWM4P_P66, 13000, 0);     //��ʼ��PWM5  ʹ������P2.5  ���PWMƵ��10000HZ   ռ�ձ�Ϊ�ٷ�֮ pwm_duty / PWM_DUTY_MAX * 100
	P64=1;
	ctimer_count_init(CTIM0_P34);
	duty=1000;
	Position_PID_Init(26,4.2,1.2,40000);
	//PID_Init(&Motor[0],12000,6000,POSITION_PID,2,0.015,0.4);
	EnableGlobalIRQ();	//�������ж�
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

