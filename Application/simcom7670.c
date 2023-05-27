#include "simcom7670.h"
#include "cmsis_os2.h"
simcom simcom_7670;

static uart_mqtt_handle_t p_uart_handle = NULL;

void mqtt_set_callback(void *callback_pointer)
{
	if (callback_pointer)
	{
		p_uart_handle = callback_pointer;
	}
}

// set config uart simcom and set task for uart
void init_simcom(uint32_t uart_num, int tx_num, int rx_num, int baud_rate)
{
	simcom_7670.baud_rate = baud_rate;
	simcom_7670.uart_num = uart_num;
	simcom_7670.tx_io_num = tx_num;
	simcom_7670.rx_io_num = rx_num;

	// uart_config_t uart_congfig =
	// {
	// 	.baud_rate = baud_rate,
	// 	.data_bits = UART_DATA_8_BITS,
	// 	.parity = UART_PARITY_DISABLE,
	// 	.stop_bits = UART_STOP_BITS_1,
	// 	.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
	// };
	// ESP_ERROR_CHECK(uart_driver_install(uart_num, BUF_SIZE, 0, 0, NULL, 0));
	// ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_congfig));
	// ESP_ERROR_CHECK(uart_set_pin(uart_num, tx_num, rx_num, ECHO_TEST_RTS, ECHO_TEST_CTS));
	xTaskCreate(uart_simcom, "uart_echo_task1", 4096, NULL, 10, NULL);
}

// use uart rx to find telegram from simcom
// 1. receive message from sever when server send something to simcom (+CMQTTRXSTART:)
// 2. receive feedback from simcom when send AT command to simcom, different feedback cases will be considered directly in each statement
void uart_simcom(void *arg)
{
	uint8_t data[BUF_SIZE];
	while (1)
	{
		int len = uart_read_bytes(simcom_7670.uart_num, data, BUF_SIZE - 1, 100 / portTICK_PERIOD_MS);
		if (len)
		{
			data[len] = '\0';
			DebugLog("REC: %s", (char *)data);
			memcpy(simcom_7670.AT_buff, data, len);
			simcom_7670.AT_buff_avai = true;
			if (strstr((char *)data, "+CMQPUB:"))
			{
				if (p_uart_handle != NULL)
				{
					p_uart_handle(data, len);
				}
			}
		}
		osDelay(10);
	}
}

// // use UART RX send AT command
static void send_ATComand(char *ATcommand)
{
	DebugLog("Send: %s", ATcommand);
	simcom_7670.AT_buff_avai = false;
	memset(simcom_7670.AT_buff, 0, BUF_SIZE);
	uart_write_bytes(simcom_7670.uart_num, (char *)ATcommand, strlen((char *)ATcommand));
	uart_write_bytes(simcom_7670.uart_num, "\r\n", strlen("\r\n"));
	osDelay(100);
}

// // check feedback and timeout when send AT command, for a period less than timeout, continuously check the return signal. When time is greater than timeout, return AT_TIMEOUT.
static AT_flag _readFeedback(uint32_t timeout, char *expect)
{
	uint64_t timeCurrent = osKernelGetSysTimerCount() / 1000;
	while (osKernelGetSysTimerCount() / 1000 < (timeout + timeCurrent))
	{
		osDelay(10);
		if (simcom_7670.AT_buff_avai)
		{
			if (strstr((char *)simcom_7670.AT_buff, "ERROR"))
				return AT_ERROR;
			else if (strstr((char *)simcom_7670.AT_buff, expect))
				return AT_OK;
		}
	}
	return AT_TIMEOUT;
}

void restart_simcom(void)
{

	osDelay(500);
	gpio_set_level(POWER_KEY, 1);
	osDelay(3000);
	gpio_set_level(POWER_KEY, 0);
	osDelay(2000);
	gpio_set_level(POWER_KEY, 1);
	osDelay(1000);
	gpio_set_level(POWER_KEY, 0);
}

bool waitModuleReady(int timeout)
{
	AT_flag res;
	simcom_7670.AT_buff_avai = false;
	memset(simcom_7670.AT_buff, 0, BUF_SIZE);
	res = _readFeedback(timeout, "+CTZV:");
	if (res == AT_OK)
		return true;
	else if (res == AT_ERROR)
		return false;
	return false;
}

// // send AT for simecom, if return AT_OK, funtion return true to check moderm init ok, same with other cases
bool isInit(int retry)
{
	AT_flag res;
	while (retry--)
	{
		send_ATComand("AT");
		res = _readFeedback(1000, "OK");
		if (res == AT_OK)
			return true;
		else if (res == AT_ERROR)
			return false;
	}
	return false;
}

// new mqtt
static int mqtt_new(client clientMQTT, int timeout, int buf_size, int retry)
{
	AT_flag res;
	char buf[200];
	sprintf(buf, "AT+CMQNEW=\"%s\",\"%d\",%d,%d", clientMQTT.uri, clientMQTT.port, timeout, buf_size);
	while (retry--)
	{
		send_ATComand(buf);
		res = _readFeedback(timeout, "+CMQNEW:");
		if (res == AT_OK)
		{
			clientMQTT.mqtt_id = simcom_7670.AT_buff[8] - '0';
			return clientMQTT.mqtt_id;
		}
		else if (res == AT_ERROR)
			return 0;
	}
	return 0;
}

// connect to broker
bool mqtt_start(client clientMQTT, int versionMQTT, int keepalive, int clean_session, int retry)
{
	if (mqtt_new(clientMQTT, 50000, 512, 3) == 0)
		return false;
	AT_flag res;
	char buf[300];
	sprintf(buf, "AT+CMQCON=%d,%d,\"%s\",%d,%d,0,\"%s\",\"%s\"", clientMQTT.mqtt_id, versionMQTT, clientMQTT.client_id, keepalive, clean_session, clientMQTT.user_name, clientMQTT.password); // will flag = 0
	send_ATComand("AT+CREVHEX=0\r\n");
	while (retry--)
	{
		send_ATComand(buf);
		res = _readFeedback(5000, "OK");
		if (res == AT_OK)
			return true;
		else if (res == AT_ERROR)
			return false;
	}
	return false;
}

// 1. Disconnect from server
// 2. Release the client
// 3. Stop MQTT Service
bool mqtt_stop(client clientMQTT, int retry)
{
	AT_flag res;
	char buf[200];
	sprintf(buf, "AT+CMQDISCON=%d", clientMQTT.mqtt_id);
	while (retry--)
	{
		send_ATComand(buf);
		res = _readFeedback(1000, "OK");
		if (res == AT_OK)
			return true;
		else if (res == AT_ERROR)
			return false;
	}
	return false;
}

// subscribe one topic to server
bool mqtt_subscribe(client clientMQTT, char *topic, int qos, int retry)
{
	AT_flag res;
	char buf[200];
	sprintf(buf, "AT+CMQSUB=%d,\"%s\",%d", clientMQTT.mqtt_id, topic, qos);
	while (retry--)
	{
		send_ATComand(buf);
		res = _readFeedback(1000, "OK");
		if (res == AT_OK)
			return true;
		else if (res == AT_ERROR)
			return false;
	}
	return false;
}

bool mqtt_message_publish(client clientMQTT, char *data, char *topic, int qos, int retry)
{
	AT_flag res;
	char buf[512];
	int cnt = 0;
	for (int i = 0; i < strlen(data); i++)
	{
		if (data[i] == '\\')
		{
			cnt++;
		}
	}
	sprintf(buf, "AT+CMQPUB=%d,\"%s\",%d,0,1,%d,%s", clientMQTT.mqtt_id, topic, qos, strlen(data) - cnt - 2, data);
	while (retry--)
	{
		send_ATComand(buf);
		res = _readFeedback(1000, "OK");
		if (res == AT_OK)
			return true;
		else if (res == AT_ERROR)
			return false;
	}
	return false;
}

bool get_signal_strength(int *rssi, int *rsrp, int *rsrq, int retry)
{
	AT_flag res;
	char buf[50];
	sprintf(buf, "AT+CENG?");
	while (retry--)
	{
		send_ATComand(buf);
		res = _readFeedback(1000, "+CENG:");
		if (res == AT_OK)
		{
			char str[100];
			memcpy(str, simcom_7670.AT_buff, strlen((char *)simcom_7670.AT_buff) + 1);
			// get first token
			char *token = strtok(str, ":");
			// get all token
			int i = 1;
			while (token != NULL)
			{
				token = strtok(NULL, ",");
				if (i == 4)
				{
					*rsrp = atoi(token);
				}
				else if (i == 5)
				{
					*rsrq = atoi(token);
				}
				else if (i == 6)
				{
					*rssi = atoi(token);
				}
				i++;
			}
			return true;
		}
		else if (res == AT_ERROR)
			return false;
	}
	return false;
}

bool getCellId(int *mcc, int *mnc, char *lac, char *cid, int retry)
{
	/* get cid and lac*/
	AT_flag res;
	char buf[200];
	osDelay(100);
	sprintf(buf, "AT+CGREG?");
	while (retry--)
	{
		send_ATComand(buf);
		res = _readFeedback(1000, "+CGREG:");
		if (res == AT_OK)
		{
			char str[100];
			memcpy(str, simcom_7670.AT_buff, strlen((char *)simcom_7670.AT_buff) + 1);
			// get first token
			char *token = strtok(str, ":");
			// get all token
			int i = 1;
			while (token != NULL)
			{
				token = strtok(NULL, ",\"");
				if (i == 3)
				{
					memcpy(lac, token, strlen(token) + 1);
				}
				else if (i == 4)
				{
					memcpy(cid, token, strlen(token) + 1);
				}
				i++;
			}
			break;
		}
		else if (res == AT_ERROR)
			return false;
	}
	return false;
}

bool getRTC(char *time_string, int retry)
{
	AT_flag res;
	char buf[100];
	osDelay(100);
	sprintf(buf, "AT+CLCK?");
	while (retry--)
	{
		send_ATComand(buf);
		res = _readFeedback(1000, "+CCLK:");
		if (res == AT_OK)
		{
			char str[100];
			memcpy(str, simcom_7670.AT_buff, strlen((char *)simcom_7670.AT_buff) + 1);
			// get first token
			char *token = strtok(str, " ");
			// get all token
			while (token != NULL)
			{
				token = strtok(NULL, " ");
				memcpy(time_string, token, strlen(token) + 1);
			}
			break;
		}
		else if (res == AT_ERROR)
			return false;
	}
	return false;
}
