/*
 * common.c
 *
 *  Created on: Sep 5, 2022
 *      Author: pc
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <inttypes.h>
#include "common.h"


uint8_t Day[10];
uint8_t Time[10];

const char *monthName[12] = {
	"Jan", "Feb", "Mar", "Apr", "May", "Jun",
	"Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

void Conver_DateTime(char *datetime, char kind)
{
  if(kind == 'T')
  {
	  int Hour, Min, Sec;
	  sscanf(datetime, "%d:%d:%d", &Hour, &Min, &Sec);
	  DateTime.hour = Hour;
	  DateTime.minute = Min;
	  DateTime.second = Sec;
  }
  else
  {
	  char Month[12];
	  int Day, Year;
	  int index = 0;
	  uint8_t monthIndex;

		sscanf(datetime, "%s %d %d", Month, &Day, &Year);

		for (monthIndex = 0; monthIndex < 12; monthIndex++) {
			if (strcmp(Month, monthName[monthIndex]) == 0) {
				index = monthIndex;
				break;
			}
		}
		DateTime.day = Day + 1;
		DateTime.month = index + 1;
		DateTime.year = Year;
  }
}

time_t string_to_seconds(const char *timestamp_str)
{
    struct tm tm;
    time_t seconds;
    int r;

    if (timestamp_str == NULL) {
        printf("null argument\n");
        return (time_t)-1;
    }
    r = sscanf(timestamp_str, "%d-%d-%d %d:%d:%d", &tm.tm_year, &tm.tm_mon, &tm.tm_mday, &tm.tm_hour, &tm.tm_min, &tm.tm_sec);
    if (r != 6) {
        printf("expected %d numbers scanned in %s\n", r, timestamp_str);
        return (time_t)-1;
    }

    tm.tm_year -= 1900;
    tm.tm_mon -= 1;
    tm.tm_isdst = 0;
    seconds = mktime(&tm);
    if (seconds == (time_t)-1) {
        printf("reading time from %s failed\n", timestamp_str);
    }

    return seconds;
}



