/*
 * simcom7670_config.h
 *
 *  Created on: 29 July 2022
 *      Author: Nguyen Duc Long
 */

#ifndef SIMCOM7670_SIMCOM7670_H_
#define SIMCOM7670_SIMCOM7670_H_

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <stdint.h>
#include <math.h>
#include <inttypes.h>
#include "DebugLog.h"
#include <cmsis_os2.h>

#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)
#define ECHO_TASK_STACK_SIZE (2048)
#define BUF_SIZE (2048)

typedef void (*uart_mqtt_handle_t)(uint8_t *data, uint16_t length);

typedef struct simcom_t
{
	uint32_t uart_num;
	int tx_io_num;
	int rx_io_num;
	int baud_rate;
	bool AT_buff_avai;
	uint8_t AT_buff[BUF_SIZE];
} simcom;

typedef struct client_t
{
	char uri[51];
	int port;
	char user_name[50];
	char client_id[50];
	char password[50];
	int mqtt_id;
} client;

typedef enum
{
	AT_OK,
	AT_ERROR,
	AT_TIMEOUT,
} AT_flag;

void init_simcom(uint32_t uart_num, int tx_num, int rx_num, int baud_rate);
void uart_simcom(void *arg);
void mqtt_set_callback(void *callback_pointer);
void restart_simcom(void);
bool waitModuleReady(int timeout);
bool isInit(int retry);
bool mqtt_start(client clientMQTT, int versionMQTT, int keepalive, int clean_session, int retry);
bool mqtt_stop(client clientMQTT, int retry);
bool mqtt_subscribe(client clientMQTT, char *topic, int qos, int retry);
bool mqtt_message_publish(client clientMQTT, char *data, char *topic, int qos, int retry);
bool get_signal_strength(int *rssi, int *rsrp, int *rsrq, int retry);
bool getCellId(int *mcc, int *mnc, char *lac, char *cid, int retry);
bool getRTC(char *time_string, int retry);
void getGPS(void);
#endif /* SIMCOM7670_SIMCOM7670_H_ */
