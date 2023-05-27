/**
 ****************************************************************************************************
 * @file        btim.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2020-04-20
 * @brief       基本定时器 驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 STM32F103开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20200422
 * 第一次发布
 *
 ****************************************************************************************************
 */

#ifndef __BTIM_H
#define __BTIM_H

/******************************************************************************************/
/* 基本定时器 定义 */

/* TIMX 中断定义 
 * 默认是针对TIM6/TIM7
 * 注意: 通过修改这4个宏定义,可以支持TIM1~TIM8任意一个定时器.
 */
#include <stdint.h>
#define BTIM_TIMX_INT                       TIM3
#define BTIM_TIMX_INT_IRQn                  TIM3_IRQn
#define BTIM_TIMX_INT_IRQHandler            TIM3_IRQHandler
#define BTIM_TIMX_INT_CLK_ENABLE()          do{ __HAL_RCC_TIM3_CLK_ENABLE(); }while(0)   /* TIM6 时钟使能 */

/******************************************************************************************/

void btim_timx_int_init(uint16_t arr, uint16_t psc);    /* 基本定时器 定时中断初始化函数 */

void btim_timx_arrset(uint16_t period);  /*基本定时器 定时器预装载值函数*/
void btim_timx_counterset(uint16_t count);
void btim_timx_enable(uint8_t enable);  /*基本定时器 定时器使能函数*/


#endif

















