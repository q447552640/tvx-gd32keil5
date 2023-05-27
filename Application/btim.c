/**
 ****************************************************************************************************
 * @file        btim.c
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

#include "./BSP/TIMER/btim.h"
#include "./BSP/USART3/usart3.h"


TIM_HandleTypeDef timx_handler; /* 定时器参数句柄 */

/**
 * @brief       基本定时器TIMX中断服务函数
 * @param       无
 * @retval      无
 */
void BTIM_TIMX_INT_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&timx_handler); /* 定时器回调函数 */
}
/**
 * @brief       回调函数，定时器中断服务函数调用
 * @param       无
 * @retval      无
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == (&timx_handler))
    {
        
        USART3_RX_STA |= 1 << 15; //标记接收完成
        btim_timx_enable(DISABLE); //关闭TIM7
        
    }
}

/**
 * @brief       设置预装载周期值
 * @param       无
 * @retval      无
 */
void btim_timx_arrset(uint16_t period)
{
   __HAL_TIM_SET_COUNTER(&timx_handler,0);
    __HAL_TIM_SET_AUTORELOAD(&timx_handler,period);    
}

/**
 * @brief       设置计数值
 * @param       无
 * @retval      无
 */
void btim_timx_counterset(uint16_t count)
{
    __HAL_TIM_SET_COUNTER(&timx_handler,count); 
}



/**
 * @brief       使能基本定时器
 * @param       无
 * @retval      无
 */
void btim_timx_enable(uint8_t enable)
{
    static TIM_HandleTypeDef *p = &timx_handler;
    
    if(enable) p->Instance->CR1|=(TIM_CR1_CEN);
    else       p->Instance->CR1 &= ~(TIM_CR1_CEN);
}


/**
 * @brief       基本定时器TIMX定时中断初始化函数
 * @note
 *              基本定时器的时钟来自APB1,当PPRE1 ≥ 2分频的时候
 *              基本定时器的时钟为APB1时钟的2倍, 而APB1为36M, 所以定时器时钟 = 72Mhz
 *              定时器溢出时间计算方法: Tout = ((arr + 1) * (psc + 1)) / Ft us.
 *              Ft=定时器工作频率,单位:Mhz
 *
 * @param       arr: 自动重装值。
 * @param       psc: 时钟预分频数
 * @retval      无
 */
void btim_timx_int_init(uint16_t arr, uint16_t psc)
{
    timx_handler.Instance = BTIM_TIMX_INT;                      /* 通用定时器X */
    timx_handler.Init.Prescaler = psc;                          /* 设置预分频器  */
    timx_handler.Init.CounterMode = TIM_COUNTERMODE_UP;         /* 向上计数器 */
    timx_handler.Init.Period = arr;                             /* 自动装载值 */
    timx_handler.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;   /* 时钟分频因子 */
    HAL_TIM_Base_Init(&timx_handler);

    HAL_TIM_Base_Start_IT(&timx_handler);                       /* 使能通用定时器x和及其更新中断：TIM_IT_UPDATE */
    
}

/**
 * @brief       定时器底册驱动，开启时钟，设置中断优先级
                此函数会被HAL_TIM_Base_Init()函数调用
 * @param       无
 * @retval      无
 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == BTIM_TIMX_INT)
    {
        BTIM_TIMX_INT_CLK_ENABLE();                     /* 使能TIM时钟 */
        HAL_NVIC_SetPriority(BTIM_TIMX_INT_IRQn, 1, 3); /* 抢占1，子优先级3，组2 */
        HAL_NVIC_EnableIRQ(BTIM_TIMX_INT_IRQn);         /* 开启ITMx中断 */
    }
}
