#ifndef _DATALINKLAYER_H
#define _DATALINKLAYER_H

#include "CharQueue.h"
#include "SimCom.h"

#include "gd32f4xx_usart.h"

#define DL_BUF_LEN 200
#define DL_RETRY_TIMES 10

bool dl_init(UART_HandleTypeDef *device);
bool dl_receive(char *data, SIMCOM_LENGTH_TYPE *length);
bool dl_send(const char *data, SIMCOM_LENGTH_TYPE length);

#endif
