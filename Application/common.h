/*
 * common.h
 *
 *  Created on: Sep 5, 2022
 *      Author: pc
 */

#ifndef MAIN_COMMON_H_
#define MAIN_COMMON_H_

extern rtc_parameter_struct   DateTime; ;
extern uint8_t Datetime[20];

void Conver_DateTime(char *datetime, char kind);
time_t string_to_seconds(const char *timestamp_str);

#endif /* MAIN_COMMON_H_ */
