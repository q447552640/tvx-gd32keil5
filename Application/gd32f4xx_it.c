/*!
    \file    gd32f4xx_it.c
    \brief   interrupt service routines

    \version 2016-08-15, V1.0.0, firmware for GD32F4xx
    \version 2018-12-12, V2.0.0, firmware for GD32F4xx
    \version 2020-09-30, V2.1.0, firmware for GD32F4xx
    \version 2022-03-09, V3.0.0, firmware for GD32F4xx
*/

/*
    Copyright (c) 2022, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/
#include "gd32f4xx.h"
#include "gd32f4xx_it.h"
#include "gd32f4xx_usart.h"
#include "systick.h"
#include <stdio.h>

extern uint8_t tx_size;
extern uint8_t rx_size;
extern __IO uint8_t txcount;
extern __IO uint16_t rxcount;
extern uint8_t rxbuffer[1024];
extern uint8_t txbuffer[];
extern uint16_t USART0_RX_STA;

extern uint8_t tx5_size;
extern uint8_t rx5_size;
extern __IO uint8_t tx5count;
extern __IO uint16_t rx5count;
extern uint8_t rx5buffer[1024];
extern uint8_t tx5buffer[];

/*!
    \brief      this function handles NMI exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void NMI_Handler(void)
{
}

/*!
    \brief      this function handles HardFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
// void HardFault_Handler(void)
//{
//     /* if Hard Fault exception occurs, go to infinite loop */
//     while(1) {
//
//		}
// }

/*!
    \brief      this function handles MemManage exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void MemManage_Handler(void)
{
    /* if Memory Manage exception occurs, go to infinite loop */
    while (1)
    {
    }
}

/*!
    \brief      this function handles BusFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void BusFault_Handler(void)
{
    /* if Bus Fault exception occurs, go to infinite loop */
    while (1)
    {
    }
}

/*!
    \brief      this function handles UsageFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void UsageFault_Handler(void)
{
    /* if Usage Fault exception occurs, go to infinite loop */
    while (1)
    {
    }
}

/*!
    \brief      this function handles SVC exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
// void SVC_Handler(void)
//{
// }

/*!
    \brief      this function handles DebugMon exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void DebugMon_Handler(void)
{
}

/*!
    \brief      this function handles PendSV exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
// void PendSV_Handler(void)
//{
// }

///*!
//    \brief      this function handles SysTick exception
//    \param[in]  none
//    \param[out] none
//    \retval     none
//*/
// void SysTick_Handler(void)
//{
//    delay_decrement();
//}

/*!
    \brief      this function handles USART RBNE interrupt request and TBE interrupt request
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USART5_IRQHandler(void)
{
    uint8_t ch = 0;
    if ((RESET != usart_interrupt_flag_get(USART5, USART_INT_FLAG_RBNE)) &&
        (RESET != usart_flag_get(USART5, USART_FLAG_RBNE)))
    {
        /* receive data */
        ch = (usart_data_receive(USART5) & 0x7F);
        // printf("%x",ch);
        // printf(ch+'0');
        // usart_data_transmit(USART5, (uint8_t)ch);
        usart_data_transmit(USART0, (uint8_t)ch);
        
        // if(rx5count == rx5_size) {
        //     usart_interrupt_disable(USART5, USART_INT_RBNE);
        // }
    }
    // if((RESET != usart_interrupt_flag_get(USART5, USART_INT_FLAG_RBNE)) &&
    //         (RESET != usart_flag_get(USART5, USART_FLAG_RBNE))) {
    //     /* receive data */
    //     rx5buffer[rx5count++] = usart_data_receive(USART5);
    //     if(rx5count == rx5_size) {
    //         usart_interrupt_disable(USART5, USART_INT_RBNE);
    //     }
    // }
    // if((RESET != usart_flag_get(USART5, USART_FLAG_TBE)) &&
    //         (RESET != usart_interrupt_flag_get(USART5, USART_INT_FLAG_TBE))) {

    //     /* transmit data */
    //     usart_data_transmit(USART5, tx5buffer[tx5count++]);
    //     if(tx5count == tx5_size) {
    //         usart_interrupt_disable(USART5, USART_INT_TBE);
    //     }
    // }
}