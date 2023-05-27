#include "main.h"
#include "RTE_Components.h"
#include CMSIS_device_header
#include "cmsis_os2.h"
#include "gd32f4xx_can.h"
#include "gd32f4xx_usart.h"
#include "gd32f4xx_gpio.h"
#include "gd32f4xx_rcu.h"
#include "gd32f4xx_misc.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <time.h>

// 字段定义
#define CAN_BAUDRATE 250
// 这段是can的全局对象
FlagStatus can0_receive_flag;
FlagStatus can0_error_flag;
can_parameter_struct can_init_parameter;
can_filter_parameter_struct can_filter_parameter;
can_trasnmit_message_struct transmit_message;
can_receive_message_struct receive_message;
// 这段是4g模块的全局对象
#define UART0_BUFFER_SIZE 1024
// MQTT的配置信息
#define MQTT_HOST "mqtt://node1.m.aquaexcel.cn:1883"
#define MQTT_CLIENT_ID "unknown_device"
#define MQTT_USERNAME "admin"
#define MQTT_PASSWORD "admin"
#define MQTT_TOPIC "tvx/%s/workinfo"
#define MQTT_TOPIC_SUB "tvxcmd/%s/ctrl"

char sim_card_number[22] = {0};

uint8_t txbuffer[UART0_BUFFER_SIZE];
uint8_t rxbuffer[UART0_BUFFER_SIZE];
uint8_t tx_size = UART0_BUFFER_SIZE;
uint8_t rx_size = UART0_BUFFER_SIZE;
__IO uint8_t txcount = 0;
__IO uint16_t rxcount = 0;

/*
    通过判断接收连续2个字符之间的时间差不大于10ms来决定是不是一次连续的数据.
    如果2个字符接收间隔超过timer,则认为不是1次连续数据.也就是超过timer没有接收到
    任何数据,则表示此次接收完毕.
    接收到的数据状态
    [15]:0,没有接收到数据;1,接收到了一批数据.
    [14:0]:接收到的数据长度
*/
uint16_t USART0_RX_STA = 0;
osTimerId_t usart0_timer_id;

uint8_t tx5buffer[UART0_BUFFER_SIZE];
uint8_t rx5buffer[UART0_BUFFER_SIZE];
// uint8_t tx5_size = UART0_BUFFER_SIZE;
// uint8_t rx5_size = UART0_BUFFER_SIZE;
__IO uint8_t tx5count = 0;
__IO uint16_t rx5count = 0;

// UART3 是485_1
// UART4 是485_2
// CAN buffer
uint8_t canDataBuffer[54] = {0};
// 默认连不上控制器，直到控制器给了回复

// 方法定义
// void app_main (void);
// 初始化方法组
void Init_GPIO(void);
void Init_CAN(void);
void can_config(can_parameter_struct can_parameter, can_filter_parameter_struct can_filter);
/* this function handles CAN0 RX0 exception */
void CAN0_RX0_IRQHandler(void);
void USART0_IRQHandler(void);
void Init_MQTT(void);
// void Init_GPS(void);
// void Init_BlueTooth(void);
void Init_Debug(void);
// void InitSystem(void);
//  任务方法组
void CanProjectMainAPP(void *pvParameters);
void TestLED(void *pvParameters);
void TestDBG(void *pvParameters);
void CANTask(void);
void TestLTETask(void);
// void MQTTTask(void);
void push_mqtt_message(char *fmt, ...);
// void GPSTask(void);
// 中断响应
void nvic_config(void);
// 4g模块代码的声明
void usart0_sendData(char *fmt, ...);
void debug_info(char *fmt, ...);
uint8_t *lte_check_cmd(uint8_t *str);
uint8_t lte_send_cmd(uint8_t *cmd, uint8_t *ack, uint16_t waittime);
void zdw_uart0_send_string(char *str);
// 方法实现
// void app_main (void)
//{
//	Init_GPIO();
//
//   // ...
//   for (;;) {}
// }

void Init_GPIO(void)
{

  // 灯光的初始化
  rcu_periph_clock_enable(RCU_GPIOA);
  gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);
  gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);

  // 4g模块的初始化
  // LET ON/OFF针脚 PB4
  rcu_periph_clock_enable(RCU_GPIOB);
  gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, GPIO_PIN_4);
  gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO_PIN_4);
  // LET WAK针脚 PB5
  gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_5);
  gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO_PIN_5);

  // LET RST针脚 PB8
  gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_8);
  gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO_PIN_8);

  // LET 1PPS 事件针脚 PA15
  gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_15);
  gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO_PIN_15);
}

void Init_Debug(void)
{
  /* enable GPIO clock */
  rcu_periph_clock_enable(RCU_GPIOC);

  /* enable USART clock */
  rcu_periph_clock_enable(RCU_USART5);

  /* configure the USART0 TX pin and USART0 RX pin */
  gpio_af_set(GPIOC, GPIO_AF_8, GPIO_PIN_6);
  gpio_af_set(GPIOC, GPIO_AF_8, GPIO_PIN_7);

  /* configure USART0 TX as alternate function push-pull */
  gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_6);
  gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6);

  /* configure USART0 RX as alternate function push-pull */
  gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_7);
  gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_7);

  /* USART configure */
  usart_deinit(USART5);
  usart_baudrate_set(USART5, 115200U);
  usart_word_length_set(USART5, USART_WL_8BIT); // 8位数据
  usart_stop_bit_set(USART5, USART_STB_1BIT);   // 1位停止位
  usart_parity_config(USART5, USART_PM_NONE);
  usart_receive_config(USART5, USART_RECEIVE_ENABLE);
  usart_transmit_config(USART5, USART_TRANSMIT_ENABLE);
  usart_enable(USART5);

  // printf("a usart transmit test example!");
  // while(1);
}

void Init_MQTT(void)
{
  /* enable GPIO clock */
  rcu_periph_clock_enable(RCU_GPIOB);

  /* enable USART clock */
  rcu_periph_clock_enable(RCU_USART0);

  /* configure the USART0 TX pin and USART0 RX pin */
  gpio_af_set(GPIOB, GPIO_AF_7, GPIO_PIN_6);
  gpio_af_set(GPIOB, GPIO_AF_7, GPIO_PIN_7);

  /* configure USART0 TX as alternate function push-pull */
  gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_6);
  gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6);

  /* configure USART0 RX as alternate function push-pull */
  gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_7);
  gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_7);

  /* USART configure */
  usart_deinit(USART0);
  usart_baudrate_set(USART0, 115200U);
  usart_word_length_set(USART0, USART_WL_8BIT); // 8位数据
  usart_stop_bit_set(USART0, USART_STB_1BIT);   // 1位停止位
  usart_parity_config(USART0, USART_PM_NONE);
  usart_receive_config(USART0, USART_RECEIVE_ENABLE);
  usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
  usart_enable(USART0);

  // printf("a usart transmit test example!");
  // while(1);
}

/*!
    \brief      initialize CAN and filter
    \param[in]  can_parameter
      \arg        can_parameter_struct
    \param[in]  can_filter
      \arg        can_filter_parameter_struct
    \param[out] none
    \retval     none
*/
void can_config(can_parameter_struct can_parameter, can_filter_parameter_struct can_filter)
{
  can_struct_para_init(CAN_INIT_STRUCT, &can_parameter);
  can_struct_para_init(CAN_INIT_STRUCT, &can_filter);
  /* initialize CAN register */
  can_deinit(CAN0);
  // can_deinit(CAN1);

  /* initialize CAN parameters */
  can_parameter.time_triggered = DISABLE;
  can_parameter.auto_bus_off_recovery = DISABLE;
  can_parameter.auto_wake_up = DISABLE;
  can_parameter.auto_retrans = ENABLE;
  can_parameter.rec_fifo_overwrite = DISABLE;
  can_parameter.trans_fifo_order = DISABLE;
  can_parameter.working_mode = CAN_NORMAL_MODE;
  //  can_parameter.working_mode = CAN_LOOPBACK_MODE;
  can_parameter.resync_jump_width = CAN_BT_SJW_1TQ;
  can_parameter.time_segment_1 = CAN_BT_BS1_4TQ;
  can_parameter.time_segment_2 = CAN_BT_BS2_3TQ;

  can_parameter.prescaler = 21;
  /* initialize CAN */
  can_init(CAN0, &can_parameter);
  // can_init(CAN1, &can_parameter);

  /* initialize filter */
  can_filter.filter_number = 0;
  can_filter.filter_mode = CAN_FILTERMODE_MASK;
  // can_filter.filter_mode = CAN_FILTERMODE_LIST;
  can_filter.filter_bits = CAN_FILTERBITS_32BIT;
  can_filter.filter_list_high = 0x0000;
  can_filter.filter_list_low = 0x0000;
  can_filter.filter_mask_high = 0x0000;
  can_filter.filter_mask_low = 0x0000;
  can_filter.filter_fifo_number = CAN_FIFO0;
  can_filter.filter_enable = ENABLE;

  can_filter_init(&can_filter);
}

/*!
    \brief      this function handles CAN0 RX0 exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void CAN0_RX0_IRQHandler(void)
{
  /* check the receive message */
  can_message_receive(CAN0, CAN_FIFO0, &receive_message);
  // receive_message.rx_sfid
  if (0x0383 == receive_message.rx_sfid)
  {
    uint8_t *rx_data = receive_message.rx_data;
    if (0x22 == rx_data[0])
    {
      for (int i = 0; i <= 5; i++)
      {
        canDataBuffer[(rx_data[1] - 1) * 6 + i] = rx_data[i + 2];
      }
    }
    can0_receive_flag = SET;
  }
  else
  {
    can0_error_flag = SET;
  }
  // printf("\r\n can0 receive data:%x,%x", receive_message.rx_data[0], receive_message.rx_data[1]);
  gpio_bit_toggle(GPIOA, GPIO_PIN_2);
}

void Init_CAN(void)
{
  /* enable can receive FIFO0 not empty interrupt */

  /* configure CAN0 NVIC */
  // nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);
  // nvic_irq_enable(CAN0_RX0_IRQn, 0, 3);

  can0_receive_flag = RESET;
  can0_error_flag = RESET;
  can_struct_para_init(CAN_TX_MESSAGE_STRUCT, &transmit_message);
  can_struct_para_init(CAN_RX_MESSAGE_STRUCT, &receive_message);
  /* enable CAN clock */
  rcu_periph_clock_enable(RCU_CAN0);
  rcu_periph_clock_enable(RCU_GPIOA);
  /* configure CAN0 GPIO */
  gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_11);
  gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_11);
  gpio_af_set(GPIOA, GPIO_AF_9, GPIO_PIN_11);

  gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);
  gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_12);
  gpio_af_set(GPIOA, GPIO_AF_9, GPIO_PIN_12);
  /* initialize CAN and filter */
  can_config(can_init_parameter, can_filter_parameter);

  /* initialize transmit message */
  //    transmit_message.tx_sfid = 0x300>>1;
  //    transmit_message.tx_efid = 0x00;
  //    transmit_message.tx_ft = CAN_FT_DATA;
  //    transmit_message.tx_ff = CAN_FF_STANDARD;
  //    transmit_message.tx_dlen = 2;
}

void nvic_config(void)
{
  // nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);
  nvic_irq_enable(CAN0_RX0_IRQn, 2, 0);
  nvic_irq_enable(USART0_IRQn, 0, 1);
  nvic_irq_enable(USART5_IRQn, 0, 1);

  // usart_interrupt_enable(USART0, USART_INT_TBE);
  usart_interrupt_enable(USART0, USART_INT_RBNE);
  // usart_interrupt_enable(USART5, USART_INT_TBE);
  usart_interrupt_enable(USART5, USART_INT_RBNE);
  can_interrupt_enable(CAN0, CAN_INT_RFNE0);
}

// region TASKS
#define IotInfoTemplate "{\
\"iccid\":\"%s\",\
\"csq\":%d,\
\"lat\":%s,\
\"ns\":\"%c\",\
\"lon\":%s,\
\"ew\":\"%c\",\
\"alt\":%s,\
\"speed\":%s,\
\"course\":%s,\
\"time\":%d\
}"

#define BaseInfoTemplate "{\
\"iccid\":\"%s\",\
\"deviceType\":%d,\
\"batterType\":%d,\
\"batterV\":%d,\
\"batterC\":%d,\
\"keyV\":%d,\
\"isConnect\":%d,\
\"isBind\":%d,\
\"isLease\":%d,\
\"isExpires\":%d,\
\"isLock\":%d,\
\"time\":%d\
}"

#define ErrorInfoTemplate "{\
\"iccid\":\"%s\",\
\"deviceType\":%d,\
\"errorCode\":%d,\
\"errorCode1\":%d,\
\"errorCode2\":%d,\
\"errorCode3\":%d,\
\"errorCode4\":\"%s\",\
\"time\":%d\
}"

#define WorkInfoTemplate "{\
\"iccid\":\"%s\",\
\"deviceType\":%d,\
\"batterType\":%d,\
\"batterV\":%d,\
\"carSpeed\":%d,\
\"walkTime\":%d,\
\"brushTime\":%d,\
\"suctionTime\":%d,\
\"HCLOTime\":%d,\
\"mBrushTime\":%d,\
\"sBrushTime\":%d,\
\"dustTime\":%d,\
\"dustCTime\":%d,\
\"hwaterTime\":%d,\
\"walkV\":%d,\
\"walkE\":%d,\
\"brushV\":%d,\
\"brushE\":%d,\
\"suctionV\":%d,\
\"suctionE\":%d,\
\"mBrushV\":%d,\
\"mBrushE\":%d,\
\"sBrushV\":%d,\
\"sBrushE\":%d,\
\"dustV\":%d,\
\"dustE\":%d,\
\"hWaterV\":%d,\
\"hWaterE\":%d,\
\"lat\":%s\,\
\"ns\":\"%c\",\
\"lon\":%s\,\
\"ew\":\"%c\",\
\"alt\":%s\,\
\"speed\":%s\,\
\"course\":%s\,\
\"time\":%d\
}"

/**
 * @brief  串口3发送
 * @note  确保一次发送数据不超过UART3_BUFFER_SIZE字节
 * @retval 无
 */
void push_mqtt_message(char *fmt, ...)
{
  char tmpBuffer[1024];
  uint16_t len = 0;
  va_list ap;
  va_start(ap, fmt);
  vsprintf((char *)tmpBuffer, fmt, ap);
  va_end(ap);
  len = strlen((const char *)tmpBuffer); // 此次发送数据的长度
  printf("Debug: MQTTJSON: %s\r\n", tmpBuffer);
  // tmpBuffer[len] = 0x1A;
  // tmpBuffer[len + 1] = 0;
  usart0_sendData("%s%d", "AT+CMQTTPAYLOAD=0,", len);
  osDelay(100);
  zdw_uart0_send_string((char *)tmpBuffer);
  osDelay(100);
  // lte_send_cmd(tmpBuffer, "", 100);
  //  zdw_uart0_send_data_len((uint8_t *)tmpBuffer, len);
}

void push_mqtt_message_task(void *pvParameters)
{
  while (1)
  {
    // can_interrupt_disable(CAN0, CAN_INT_RFNE0);
    //  V1.3协议
    // 如果与模块通讯正常，则发送基本信息到服务器
    // 取信号强度
    // AT+CSQ

    // +CSQ: 14,99

    // OK
    lte_send_cmd("AT+CSQ", NULL, 100);
    // 读取信号强度
    uint8_t *p, *p2;
    osDelay(10);
    while (!(USART0_RX_STA & 0X8000))
      ;
    p = lte_check_cmd("+CSQ:");
    p += 6;
    //+CSQ: 14,99
    p2 = p;
    while (*p2 != ',')
      p2++;
    *p2 = 0;
    uint8_t csq = atoi((const char *)p);

    // 取GPS信息
    // AT+CGPSINFO
    //+CGPSINFO:3113.343286,N,12121.234064,E,250311,072809.3,44.1,0.0,0
    lte_send_cmd("AT+CGPSINFO", NULL, 100);
    // 读取GPS信息
    osDelay(10);
    while (!(USART0_RX_STA & 0X8000))
      ;
    p = lte_check_cmd("+CGPSINFO:");
    p += 11;
    //+CGPSINFO:3113.343286,N,12121.234064,E,250311,072809.3,44.1,0.0,0

    // AT+CGNSSINFO
    // 2,09,05,00,3113.330650,N,12121.262554,E,131117,091918.0,32.9,0.0,255.0,1.1,0.8,0.7
    // +CGNSSINFO: ,,,,,,,,

    // p2=strchr((const char *)p, ',');
    printf("Debug: GPS: %s\r\n", (const char *)p);
    char gps[64];
    memcpy(gps, p, strlen(p));
    for (uint8_t i = 0; i < 64; i++)
    {
      if (gps[i] == '\r')
      {
        gps[i] = '\0';
        break;
      }
    }

    // TEST set gps "3113.343286,N,12121.234064,E,250311,072809.3,44.1,0.0,0"
    // strcpy(gps, "3157.68848,N,11832.20618,E,280223,101331.00,110.2,3.726,35.63");
    // gps = "3113.343286,N,12121.234064,E,250311,072809.3,44.1,0.0,0";

    // 3157.68926,N,11832.18452,E,280223,101140.00,28.6,2.770,221.53
    char lat[15] = "0.0";
    char ns = 'N';
    char lon[15] = "0.0";
    char ew = 'E';
    char gps_date[10] = "000000";
    char gps_time[10] = "000000";
    char alt[10] = "0.0";
    char speed[10] = "0.0";
    char course[10] = "0.0";
    //  如果全是“,”则表示没有GPS信息

    if (strlen(gps) > 10)
    {
      char *p3 = strchr((const char *)gps, ',');
      memcpy(lat, gps, p3 - gps);
      p3++;
      ns = *p3;
      p3 += 2;
      char *p4 = strchr((const char *)p3, ',');
      memcpy(lon, p3, p4 - p3);
      p4++;
      ew = *p4;
      p4 += 2;
      char *p5 = strchr((const char *)p4, ',');
      memcpy(gps_date, p4, p5 - p4);
      p5++;
      char *p6 = strchr((const char *)p5, ',');
      memcpy(gps_time, p5, p6 - p5);
      p6++;
      char *p7 = strchr((const char *)p6, ',');
      memcpy(alt, p6, p7 - p6);
      p7++;
      char *p8 = strchr((const char *)p7, ',');
      memcpy(speed, p7, p8 - p7);
      p8++;
      // 最后一个数据没有逗号
      char *p9 = strlen((const char *)p8) + p8;
      memcpy(course, p8, p9 - p8);
    }
    printf("Debug: lat: %s, ns: %c, lon: %s, ew: %c, date: %s, time: %s, alt: %s, speed: %s, course: %s\r\n", lat, ns, lon, ew, gps_date, gps_time, alt, speed, course);

    //	gps[strlen(p)]='\0';
    // USART0_RX_STA = 0;
    // 时间
    // AT+CCLK?
    //+CCLK: "23/02/24,04:18:33+32"
    uint8_t time_cmd_r = lte_send_cmd("AT+CCLK?", NULL, 100);
    while (!(USART0_RX_STA & 0X8000))
      ;
    p = lte_check_cmd("+CCLK:");
    p += 8;
    //+CCLK: "23/02/24,04:18:33+32"
    // p2=strchr((const char *)p, ',');
    // yy/MM/dd,hh:mm:ss±zz
    printf("Debug: Time: %s\r\n", (const char *)p);
    char timestr[64];
    memcpy(timestr, p, strlen(p));
    for (uint8_t i = 0; i < 64; i++)
    {
      if (timestr[i] == '\r')
      {
        timestr[i - 1] = '\0';
        break;
      }
    }
    time_t unix_time = 0;
    // 把yy/MM/dd,hh:mm:ss±zz转换成unix时间戳
    struct  tm timeinfo;
    timeinfo.tm_year = (timestr[0] - '0') * 10 + (timestr[1] - '0')+100;
    timeinfo.tm_mon = (timestr[3] - '0') * 10 + (timestr[4] - '0') - 1;
    timeinfo.tm_mday = (timestr[6] - '0') * 10 + (timestr[7] - '0');
    timeinfo.tm_hour = (timestr[9] - '0') * 10 + (timestr[10] - '0');
    timeinfo.tm_min = (timestr[12] - '0') * 10 + (timestr[13] - '0');
    timeinfo.tm_sec = (timestr[15] - '0') * 10 + (timestr[16] - '0');
		
    unix_time = mktime(&timeinfo)-28800;
    printf("Debug: unix_time: %d\r\n", unix_time);

    char iot_cmd[50];

    // unix_time = get_unix_time(timestr);
    //  timestr[strlen(p)]='\0';
    //   USART0_RX_STA = 0;
    {
      sprintf(iot_cmd, "IotInfo/IOT_%s", sim_card_number);
      uint8_t r = lte_send_cmd("AT+CMQTTTOPIC=0,31", ">", 100);
      if (r == 1)
      {
        printf("Debug: CMD AT+CMQTTTOPIC=0,7 OK\r\n");
      }
      lte_send_cmd(iot_cmd, "", 100);

      push_mqtt_message(IotInfoTemplate, sim_card_number, csq, lat, ns, lon, ew, alt, speed, course, unix_time);
      r = lte_send_cmd("AT+CMQTTPUB=0,1,60", "OK", 100);
      if (r == 1)
      {
        printf("Debug: CMD AT+CMQTTPUB=0,1,60 OK\r\n");
      }
    }
    osDelay(1000 * 2);

    if (canDataBuffer[5] == 0)
    {
      sprintf(iot_cmd, "BaseInfo/IOT_%s", sim_card_number);
      uint8_t r = lte_send_cmd("AT+CMQTTTOPIC=0,32", ">", 100);
      if (r == 1)
      {
        printf("Debug: CMD AT+CMQTTTOPIC=0,32 OK\r\n");
      }
      lte_send_cmd(iot_cmd, "", 100);

      push_mqtt_message(BaseInfoTemplate,
                        sim_card_number,
                        canDataBuffer[0],
                        canDataBuffer[1],
                        canDataBuffer[2],
                        canDataBuffer[3],
                        canDataBuffer[4],
                        canDataBuffer[5],
                        canDataBuffer[6],
                        canDataBuffer[7],
                        canDataBuffer[8],
                        canDataBuffer[9],
                        unix_time);

      r = lte_send_cmd("AT+CMQTTPUB=0,1,60", "OK", 100);

      if (r == 1)
      {
        printf("Debug: CMD AT+CMQTTPUB=0,1,60 OK\r\n");
      }

      // 检查开关是否开启，如果开关开启，则认为可能要进行工作。则发送工作信息到服务器
      if (canDataBuffer[4] != 0)
      {
        osDelay(1000 * 2);
        // TODO 产生报警信息
        sprintf(iot_cmd, "WorkInfo/IOT_%s", sim_card_number);
        uint8_t r = lte_send_cmd("AT+CMQTTTOPIC=0,32", ">", 100);
        if (r == 1)
        {
          printf("Debug: CMD AT+CMQTTTOPIC=0,32 OK\r\n");
        }
        lte_send_cmd(iot_cmd, "", 100);
        push_mqtt_message(WorkInfoTemplate,
                          sim_card_number,
                          canDataBuffer[0],
                          canDataBuffer[1],
                          canDataBuffer[2],
                          canDataBuffer[18],
                          canDataBuffer[10] + canDataBuffer[11] * 256,
                          canDataBuffer[12] + canDataBuffer[13] * 256,
                          canDataBuffer[14] + canDataBuffer[15] * 256,
                          canDataBuffer[16] + canDataBuffer[17] * 256,
                          canDataBuffer[26] + canDataBuffer[27] * 256,
                          canDataBuffer[28] + canDataBuffer[29] * 256,
                          canDataBuffer[30] + canDataBuffer[31] * 256,
                          canDataBuffer[32] + canDataBuffer[33] * 256,
                          canDataBuffer[34] + canDataBuffer[35] * 256,
                          canDataBuffer[37],
                          canDataBuffer[38],
                          canDataBuffer[39],
                          canDataBuffer[40],
                          canDataBuffer[41],
                          canDataBuffer[42],
                          canDataBuffer[43],
                          canDataBuffer[44],
                          canDataBuffer[45],
                          canDataBuffer[46],
                          canDataBuffer[47],
                          canDataBuffer[48],
                          canDataBuffer[49],
                          canDataBuffer[50],
                          lat,
                          ns,
                          lon,
                          ew,
                          alt,
                          speed,
                          course,
                          unix_time);
        r = lte_send_cmd("AT+CMQTTPUB=0,1,60", "OK", 100);

        if (r == 1)
        {
          printf("Debug: CMD AT+CMQTTPUB=0,1,60 OK\r\n");
        }
        osDelay(1000);
      }
      else
      {
        // 如果开关没有开启，则认为设备没有工作进行休眠。则多睡一会
        osDelay(10 * 1000);
      }
    }
    // else{
    //   // 设备开着，但是没有通讯，说明设备可能出现故障。则发送故障信息到服务器
    //   uint8_t r = lte_send_cmd("AT+CMQTTTOPIC=0,8", ">", 100);
    //   if (r == 1)
    //   {
    //     printf("Debug: CMD AT+CMQTTTOPIC=0,7 OK\r\n");
    //   }
    //   lte_send_cmd("iotInfo", "", 100);
    //   //TODO 产生故障信息
    //   osDelay(10*1000);
    // }
    // 如果有报错，则处理报错,19,22,23,24,25任意一位不为零则报错
    if (canDataBuffer[19] != 0 || canDataBuffer[22]!=0||canDataBuffer[23]!=0||canDataBuffer[24]!=0||canDataBuffer[25]!=0)
    {
      osDelay(1000 * 2);
      // TODO 产生报错信息
      sprintf(iot_cmd, "ErrorInfo/IOT_%s", sim_card_number);
      uint8_t r = lte_send_cmd("AT+CMQTTTOPIC=0,32", ">", 100);
      if (r == 1)
      {
        printf("Debug: CMD AT+CMQTTTOPIC=0,32 OK\r\n");
      }
      lte_send_cmd(iot_cmd, "", 100);
      //报错需要上报的是设备号，设备类型，故障码
      //#define ErrorInfoTemplate "{\
      // \"iccid\":\"%s\",\
      // \"deviceType\":%d,\
      // \"errorCode\":%d,\
      // \"errorCode1\":%d,\
      // \"errorCode2\":%d,\
      // \"errorCode3\":%d,\
      // \"errorCode4\":\"%s\",\
      // \"time\":%d\
      // }"
      push_mqtt_message(ErrorInfoTemplate,
                        sim_card_number,//iccid
                        canDataBuffer[0],//设备类型
                        canDataBuffer[19],//故障码
                        canDataBuffer[22],//故障码1
                        canDataBuffer[23],//故障码2
                        canDataBuffer[24],//故障码3
                        canDataBuffer[25],//故障码4
                        unix_time);//时间
      r = lte_send_cmd("AT+CMQTTPUB=0,1,60", "OK", 100);

      if (r == 1)
      {
        printf("Debug: CMD AT+CMQTTPUB=0,1,60 OK\r\n");
      }
      osDelay(1000);

    }
    // can_interrupt_enable(CAN0, CAN_INT_RFNE0);
    osDelay(1000 * 10);
  }
}

void a7600_4g_let_init(void)
{
  // 这里降低电压是无效的，一开始就供电了
  printf("Debug: A7600_4G_LTE_INIT\r\n");
  GPIO_BC(GPIOB) = GPIO_PIN_4;
  printf("Debug: A7600_4G_LTE_OFF\r\n");
  osDelay(2000);
  GPIO_BOP(GPIOB) = GPIO_PIN_4;
  printf("Debug: A7600_4G_LTE_ON\r\n");
  osDelay(1000);
  GPIO_BOP(GPIOB) = GPIO_PIN_8;
  osDelay(2000);
  GPIO_BC(GPIOB) = GPIO_PIN_8;
  // GPIO_BC
  // PB DONE
}

uint8_t a7600_4g_let_connect(void)
{
  // 4G模块默认开启是会给出DONE，可以用这个方法来判断是否开启
  // *ATREADY: 1
  // +CPIN: READY
  // +CGEV: EPS PDN ACT 1
  // SMS DONE
  // PB DONE
  USART0_RX_STA = 0;
  char *strx = 0;
  uint8_t *str = "PB DONE";
  // rxbuffer[USART0_RX_STA & 0X7FFF] = 0; // 添加结束符 一开始空的，不用加结束符
  strx = strstr((const char *)rxbuffer, (const char *)str);
  // 60s超时
  uint8_t wait_times_out = 0x3c;
  // osThreadYield();
  while (strx == 0)
  {
    // 使用数组的方式搜索
    uint8_t k = 0;
    for (uint16_t i = 0; i < USART0_RX_STA; i++)
    {
      char key = (char)*(str + k);
      if (k == 0 && key == rxbuffer[i])
      {
        k++;
        continue;
      }
      if (k > 0 && rxbuffer[i] == key)
      {
        k++;
        if (k == 7)
        {
          strx = (char *)1;
          break;
        }
      }
    }
    // strx = strstr((const char *)rxbuffer, (const char *)str);
    osDelay(1000);
    wait_times_out--;
    if (wait_times_out == 0)
    {
      printf("Debug: Wait For 4G Module Start \"PB DONE\" Time Out\r\n");
      break;
    }
  }

  // 发送AT指令，查看模块是否开启
  uint8_t r = lte_send_cmd("AT", "OK", 100);
  if (r == 1)
  {
    printf("Debug: CMD AT OK\r\n");
  }
  else
  {
    return 0;
  }

  // 查询SIM卡是否插入
  r = lte_send_cmd("AT+CPIN?", "+CPIN: READY", 100);
  if (r == 1)
  {
    printf("Debug: CMD AT+CPIN? OK\r\n");
  }
  else
  {
    return 0;
  }
  // 查询SIM卡卡号
  r = lte_send_cmd("AT+CICCID", NULL, 100);
  while (!(USART0_RX_STA & 0X8000))
    ;
  uint8_t *p, *p2;
  p = lte_check_cmd("+ICCID:");
  p += 8;
  printf("Debug: ICCID: %s\r\n", (const char *)p);
  for (uint8_t i = 0; i < 19; i++)
  {
    sim_card_number[i] = p[i];
  }

  // 10秒才能启动好
  // osDelay(1000 * 10);
  // printf("Debug: Wait For 10 seconds\r\n");
  // 读取下时间，如果是70/01开始的，说明是默认时间，这时候需要开启设备的自动时间同步
  // AT+CCLK?\r\n+CCLK: "20/01/01,00:00:00+00"\r\n
  r = lte_send_cmd("AT+CCLK?", NULL, 100);
  while (!(USART0_RX_STA & 0X8000))
    ;
  p = lte_check_cmd("+CCLK:");
  p += 8;
  //+CCLK: "23/02/24,04:18:33+32"
  // p2=strchr((const char *)p, ',');
  printf("Debug: Time: %s\r\n", (const char *)p);
  if (p[0] == '7' && p[1] == '0')
  {
    // 看下是否开启了自动时间同步
    // AT+CTZU?\r\n+CTZU: 1\r\n
    r = lte_send_cmd("AT+CTZU?", "CTZU: 1", 100);
    if (r != 1)
    {
      // 开启自动时间同步
      r = lte_send_cmd("AT+CTZU=1", "OK", 100);
      if (r == 1)
      {
        printf("Debug: CMD AT+CTZU=1 OK\r\n");
      }
      // 保存配置
      r = lte_send_cmd("AT&W", "OK", 100);
      if (r == 1)
      {
        printf("Debug: Auto Update Time Save Setting!OK\r\n");
      }
      return 0;
    }
  }

  // 发送指定，查看模块联网情况 AT+CGATT?\r\n+CGATT: 1\r\n
  r = lte_send_cmd("AT+CGATT?", "CGATT: 1", 100);
  while (r != 1)
  {
    r = lte_send_cmd("AT+CGATT=1", "OK", 100);
    if (r != 1)
      break;
    printf("Debug: AT+CGATT? ERROR\r\n"); //+CGATT: 1\r\n
    r = lte_send_cmd("AT+CGATT?", "CGATT: 1", 100);
    osDelay(100);
  }
  // 开启GPS模块
  r = lte_send_cmd("AT+CGNSSPWR=1", "OK", 100);
  // TODO 等下面这个命令
  //+CGNSSPWR: READY!
  if (r == 1)
  {
    printf("Debug: AT+CGNSSPWR=1 OK\r\n");
  }
  osDelay(1000 * 10);
  printf("Debug: Wait AGPS Reg 10s \r\n");
  // TODO AT+CGNSSPWR?
  // TODO +CGNSSPWR: 1
  // 开启AGPS
  r = lte_send_cmd("AT+CAGPS", "OK", 1000);
  // OK\r\n+AGPS: success.
  uint8_t fail_times = 10;
  while (r != 1)
  {
    printf("Debug: AT+CAGPS ERROR\r\n");
    r = lte_send_cmd("AT+CAGPS", "OK", 1000);
    osDelay(1000);
    fail_times--;
    if (fail_times == 0)
    {
      printf("Debug: AT+CAGPS ERROR\r\n");
      return 0;
    }
  }

  return 1;
}
// 4G 模块连接MQTT服务器
uint8_t a7600_4g_let_mqtt_connect(void)
{
  uint8_t fail_times = 0;
  uint8_t r = lte_send_cmd("AT+CMQTTSTART", "+CMQTTSTART: 0", 2000);
  while (r != 1)
  {
    printf("Debug:AT+CMQTTSTART Error:%d\r\n", fail_times);
    r = lte_send_cmd("AT+CMQTTSTART", "+CMQTTSTART: 0", 2000);
    osDelay(1000 * 2);
    fail_times++;
    if (fail_times >= 10)
    {
      printf("Debug: AT+CMQTTSTART ERROR\r\n");
      break;
    }
  }
  // 设置模块客户端的 ID 名字
  char iot_cmd[50];
  sprintf(iot_cmd, "AT+CMQTTACCQ=0,\"IOT_%s\",0", sim_card_number);
  r = lte_send_cmd(iot_cmd, "OK", 100);
  // 设置模块客户端的 ID 名字
  if (r == 1)
  {
    printf("Debug:%s,0 OK\r\n", iot_cmd);
  }
  // 设置模块连接的服务器地址
  r = lte_send_cmd("AT+CMQTTCONNECT=0,\"tcp://node1.tvxiot.njbdbd.net:1883\",60,1", "OK", 100);
	//r = lte_send_cmd("AT+CMQTTCONNECT=0,\"tcp://node1.m.aquaexcel.cn:1883\",60,1", "OK", 100);
  if (r == 1)
  {
    printf("Debug:AT+CMQTTCONNECT=0,\"tcp://node1.tvxiot.njbdbd.net:1883\",60,1 OK\r\n");
		//printf("Debug:AT+CMQTTCONNECT=0,\"tcp://node1.m.aquaexcel.cn:1883\",60,1 OK\r\n");
  }
  //+CMQTTCONNECT: 0,0
  osDelay(1000);
  lte_check_cmd("+CMQTTCONNECT: 0,0");

  // 设置模块订阅的主题
  r = lte_send_cmd("AT+CMQTTSUBTOPIC=0,3,1", ">", 100);
  r = lte_send_cmd("tvx", "OK", 100);
  r = lte_send_cmd("AT+CMQTTSUB=0", "OK", 100);
  osDelay(1000);
  lte_check_cmd("+CMQTTSUB: 0,0");
  osDelay(1000);
  osThreadNew(push_mqtt_message_task, NULL, NULL);
  return 1;
}

//* retarget the C library printf function to the USART *//
int fputc(int ch, FILE *f)
{
  usart_data_transmit(USART5, (uint8_t)ch);
  while (RESET == usart_flag_get(USART5, USART_FLAG_TBE))
    ;
  return ch;
}

void TestLED(void *pvParameters)
{
  while (1)
  {
    gpio_bit_toggle(GPIOA, GPIO_PIN_1);
    osDelay(1000);

    transmit_message.tx_sfid = 0x03C1;
    transmit_message.tx_efid = 0x00;
    transmit_message.tx_ft = CAN_FT_DATA;
    transmit_message.tx_ff = CAN_FF_STANDARD;
    transmit_message.tx_dlen = 8;
    transmit_message.tx_data[0] = 0x22;
    transmit_message.tx_data[1] = 0x00;
    transmit_message.tx_data[2] = 0x00;
    transmit_message.tx_data[3] = 0x00;
    transmit_message.tx_data[4] = 0x00;
    transmit_message.tx_data[5] = 0x00;
    transmit_message.tx_data[6] = 0x00;
    transmit_message.tx_data[7] = 0x00;

    can_message_transmit(CAN0, &transmit_message);
  }
}

void TestDBG(void *pvParameters)
{
  while (1)
  {
    // printf("0");
    //		usart_data_transmit(USART5,0x31);
    //
    //		while(usart_flag_get(USART5,USART_FLAG_TBE) == RESET); // 等待发送完成
    osDelay(500);
  }
}

void CanBusThread(void *pvParameters)
{
  while (1)
  {
    can_message_receive(CAN0, CAN_FIFO0, &receive_message);

    if ((0x300 >> 1 == receive_message.rx_sfid) && (CAN_FF_STANDARD == receive_message.rx_ff) && (2 == receive_message.rx_dlen))
    {
      can0_receive_flag = SET;
      printf("\r\n Debug: can0 receive data:%x,%x\r\n", receive_message.rx_data[0], receive_message.rx_data[1]);
      gpio_bit_toggle(GPIOA, GPIO_PIN_2);
    }
    else
    {
      can0_error_flag = SET;
    }
    osDelay(100);
  }
}

// endregion

void HardFault_Handler(void)
{
  while (1)
  {
    printf("Error:Some Thing is Wrong\r\n");
    osDelay(5000);
    __set_FAULTMASK(1);
    NVIC_SystemReset();
  }
}

// USART1 send data : send data to 4G
void zdw_uart0_send_data_len(char *data, int len)
{
  int i = 0;
  for (i = 0; i < len; i++)
  {
    usart_data_transmit(USART0, data[i]);
  }
}

// USART1 send string :send data to 4G
void zdw_uart0_send_string(char *str)
{
  unsigned int k = 0;
  do
  {
    usart_data_transmit(USART0, (uint8_t) * (str + k));
    k++;
    while (RESET == usart_flag_get(USART0, USART_FLAG_TBE))
      ;
  } while (*(str + k) != '\0');
}

/**
 * @brief  串口0发送
 * @note  确保一次发送数据不超过UART0_BUFFER_SIZE字节
 * @retval 无
 */
void usart0_sendData(char *fmt, ...)
{
  uint16_t len = 0;
  va_list ap;
  va_start(ap, fmt);
  vsprintf((char *)txbuffer, fmt, ap);
  va_end(ap);
  len = strlen((const char *)txbuffer); // 此次发送数据的长度
  txbuffer[len] = 0x0D;                 // 添加回车符
  txbuffer[len + 1] = 0x0A;             // 添加换行符
  txbuffer[len + 2] = '\0';             // 添加结束符
  len += 3;
  printf("Debug::send data::len::%d::str::%s", len, txbuffer);
  zdw_uart0_send_string((char *)txbuffer);
  // zdw_uart0_send_data_len((uint8_t *)txbuffer, len);
}

/**
 * @brief       检测gm196接收应答数据函数
 * @param       str  期待的应答数据
 * @retval      uint8_t * 返回期待应答结果的地址位置
 */
uint8_t *lte_check_cmd(uint8_t *str)
{
  char *strx = 0;

  if (USART0_RX_STA & 0X8000) // 接收到一次数据了
  {
    rxbuffer[USART0_RX_STA & 0X7FFF] = 0; // 添加结束符
    strx = strstr((const char *)rxbuffer, (const char *)str);
  }
  return (uint8_t *)strx;
}

/**
 * @brief       gm196发送指令函数
 * @param       uint8_t* cmd,发送的命令字符串(不需要添加回车了),当cmd<0XFF的时候,发送数字(比如发送0X1A),大于的时候发送字符串
                uint8_t* ack,期待的应答数据,如果为空,则表示不需要等待应答
                uint16_t waittime,等待应答时间(单位:10ms)

 * @retval      uint8_t 应答结果  0,发送成功  1,发送失败
 */
uint8_t lte_send_cmd(uint8_t *cmd, uint8_t *ack, uint16_t waittime)
{
  uint8_t res = 0;
  USART0_RX_STA = 0;
  rxbuffer[USART0_RX_STA & 0X7FFF] = 0;
  // 关掉usart0的中断防止抢占
  // usart_interrupt_disable(USART0, USART_INT_RBNE);
  // while (RESET == usart_interrupt_flag_get(USART0, USART_INT_FLAG_RBNE))
  //   ; // 等待接收完成
  // osThreadYield();
  // osDelay(10);
  if ((uint32_t)cmd <= 0XFF)
  {
    // while ((USART3->SR & 0X40) == 0); //等待上一次数据发送完成
    // USART3->DR = (uint32_t)cmd;
  }
  else
  {
    usart0_sendData("%s", cmd); // 发送命令
  }
  // while (RESET == usart_flag_get(USART0, USART_FLAG_TC))
  // ; // 等待发送完成
  // 等待应答
  uint8_t ch = 0;
  uint8_t flag = 1;
  uint8_t timeout = 0xff;
  // osThreadYield();
  // while (flag)
  // {
  //   while (RESET == usart_flag_get(USART0, USART_FLAG_RBNE)){
  //     timeout--;
  //     //osThreadYield();
  //     osDelay(100);
  //     if (timeout == 0)
  //     {
  //       rxbuffer[USART0_RX_STA] = 0; // 添加结束符
  //       USART0_RX_STA |= 1 << 15;    // 强制标记接收完成
  //       flag = 0;
  //       printf("Debug::lte_send_cmd::timeout");
  //       break;
  //     }
  // 	}

  //   // {
  //   //   timeout--;
  //   //   osThreadYield();
  //   //   osDelay(1000);
  //   //   if (timeout == 0)
  //   //   {
  //   //     flag = 0;
  //   //     printf("Debug::lte_send_cmd::timeout");
  //   //     break;
  //   //   }
  //   // }

  //   /* receive data */
  //   ch = (usart_data_receive(USART0) & 0x7F);
  //   // 透传数据给debug。
  //   usart_data_transmit(USART5, (uint8_t)ch);
  //   while (RESET == usart_flag_get(USART5, USART_FLAG_TBE))
  //     ;

  //   if (USART0_RX_STA < 1024) // 还可以接收数据
  //   {
  //     // TODO 可能得用超时的方式来实现
  //     rxbuffer[USART0_RX_STA++] = ch; // 记录接收到的值
  //     // if (ch == '\n')
  //     // {
  //     //   rxbuffer[USART0_RX_STA] = 0; // 添加结束符
  //     //   USART0_RX_STA |= 1 << 15;    // 强制标记接收完成
  //     //   flag = 0;
  //     //   break;

  //     // }
  //   }
  //   else
  //   {
  //     USART0_RX_STA |= 1 << 15; // 强制标记接收完成
  //     flag = 0;
  //     break;
  //   }
  // }

  usart_receiver_timeout_enable(USART0);
  usart_receiver_timeout_threshold_config(USART0, 1152 * waittime);
  while (RESET != usart_flag_get(USART0, USART_FLAG_RBNE))
    ;
  while (RESET == usart_flag_get(USART0, USART_FLAG_RT))
    ;
  // 补\0
  rxbuffer[USART0_RX_STA] = '\0';
  printf("Debug::receive data::len::%d::str::%s", USART0_RX_STA, rxbuffer);

  USART0_RX_STA |= 0X8000;
  usart_flag_clear(USART0, USART_FLAG_RT);
  if (ack && waittime) // 需要等待应答
  {
    // 由于原来的样例给的不合适，就用超时的方式来实现
    // while (RESET == usart_flag_get(USART0, USART_FLAG_RT))
    // ;
    // while (--waittime) // 等待倒计时
    //{
    // osDelay(10);
    //  osTimerOnce(USART0Timer, 10);
    // if (USART0_RX_STA & 0X8000) // 接收到期待的应答结果
    //{
    if (lte_check_cmd(ack))
    {
      USART0_RX_STA = 0;
      res = 1;
      //    break; // 得到有效数据
    }
    else
    {
      res = 2;
    }

    // USART0_RX_STA = 0;
    //}
    //}
    // if (waittime == 0)
    //  res = 2;
  }
  // usart_flag_clear(USART0, USART_FLAG_RT);
  // usart_interrupt_enable(USART0, USART_INT_RBNE);
  return res;
}

/*!
    \brief      this function handles USART RBNE interrupt request and TBE interrupt request
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USART0_IRQHandler(void)
{
  uint8_t ch = 0;
  if ((RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_RBNE)) &&
      (RESET != usart_flag_get(USART0, USART_FLAG_RBNE)))
  {
    /* receive data */
    ch = (usart_data_receive(USART0) & 0x7F);
    // 透传数据给debug。
    // usart_data_transmit(USART5, (uint8_t)'_');
    // while (RESET == usart_flag_get(USART5, USART_FLAG_TBE))
    //   ;
    usart_data_transmit(USART5, (uint8_t)ch);
    // printf(ch);
    while (RESET == usart_flag_get(USART5, USART_FLAG_TBE))
      ;
    if ((USART0_RX_STA & (1 << 15)) == 0) // 接收完的一批数据,还没有被处理,则不再接收其他数据
    {
      if (USART0_RX_STA < 1024) // 还可以接收数据
      {

        // btim_timx_counterset(0);            //计数器清空
        // 有数据就关了重开计时器，osTimer不能用在中断里面
        // osTimerStop(usart0_timer_id); // 关闭定时器7的中断(10ms
        // osStatus_t stat = osTimerStart(usart0_timer_id, 100U);
        // printf("timer status %d",stat);
        // if (USART0_RX_STA == 0)       // 使能定时器7的中断
        // {
        //   osTimerStart(usart0_timer_id, 100); // 10ms
        //   // btim_timx_enable(ENABLE);          //使能定时器7
        // }

        rxbuffer[USART0_RX_STA++] = ch; // 记录接收到的值
      }
      else
      {
        USART0_RX_STA |= 1 << 15; // 强制标记接收完成
      }
    }

    // if(rx5count == rx5_size) {
    //     usart_interrupt_disable(USART0, USART_INT_RBNE);
    // }
  }
  // if((RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_RBNE)) &&
  //         (RESET != usart_flag_get(USART0, USART_FLAG_RBNE))) {
  //     /* receive data */
  //     rxbuffer[rxcount++] = usart_data_receive(USART0);
  //     if(rxcount == rx_size) {
  //         usart_interrupt_disable(USART0, USART_INT_RBNE);
  //     }
  // }
  // if ((RESET != usart_flag_get(USART0, USART_FLAG_TBE)) &&
  //     (RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_TBE)))
  // {

  //   /* transmit data */
  //   usart_data_transmit(USART0, txbuffer[txcount++]);
  //   if (txcount == tx_size)
  //   {
  //     usart_interrupt_disable(USART0, USART_INT_TBE);
  //   }
  // }
}

void a7600_4g_timer(void *pvParameters)
{
  printf("timer out\r\n");
  USART0_RX_STA |= 1 << 15;     // 标记接收完成
  osTimerStop(usart0_timer_id); // 关闭定时器7的中断(10ms
}

void debuf_info(char *fmt, ...)
{
  // printf
}

const osTimerAttr_t timer_rx_attr =
    {
        .name = "timer_usart0_rx",
};

void CanProjectMainAPP(void *pvParameters)
{
  usart0_timer_id = osTimerNew(a7600_4g_timer, osTimerPeriodic, (void *)0, &timer_rx_attr);
  nvic_config();
  // 等待4g模块上电
  printf("Debug:Wait 4g power on\r\n");
  // char baseInfo[1024];
  // sprintf(baseInfo, "sssss%.2fzzzzzzzzzzzzz%.f%d",
  // 1.2312312312/0, 112.541321541 + 2.0012154 / 0,1/0);
  // printf(baseInfo);
  // 4G 模块上电
  a7600_4g_let_init();
  // TODO wait 4g power on
  // osDelay(1000 * 20);
  // 4G 模块连接网络
  uint8_t connect_result = a7600_4g_let_connect();
  while (connect_result != 1)
  {
    printf("Debug:Error!!!4g connect fail,re init!!\r\n");
    a7600_4g_let_init();
    connect_result = a7600_4g_let_connect();
  }

  // 4G 模块连接MQTT服务器
  connect_result = a7600_4g_let_mqtt_connect();
  while (connect_result != 1)
  {
    printf("Debug:Error!!!4g mqtt connect fail,re init!!\r\n");
    connect_result = a7600_4g_let_mqtt_connect();
  }

  osThreadNew(TestLED, NULL, NULL); // Create application main thread
  osThreadNew(TestDBG, NULL, NULL);
  osThreadNew(CanBusThread, NULL, NULL);

  // 默认can连不上
  canDataBuffer[5] = (uint8_t)0x01;
  // vPrintString("Start FreeRTOS\r\n");
  osDelay(osWaitForever);
  while (1)
    ;
}

// 主函数
int main(void)
{
  SystemCoreClockUpdate();

  Init_GPIO();
  Init_Debug();
  Init_MQTT();
  // CAN
  Init_CAN();

  osKernelInitialize(); // Initialize CMSIS-RTOS

  osThreadNew(CanProjectMainAPP, NULL, NULL);
  osKernelStart(); // Start thread execution
  for (;;)
  {
  }
}
