#ifndef __Debug_LOG_H
#define __Debug_LOG_H

#include <string.h>

#define  DebugLog(...)    do { \
                                 printf("Debug: ") ;\
                                 printf(__VA_ARGS__);\
                               }while (0)

#endif /* __LCD_LOG_H */ 