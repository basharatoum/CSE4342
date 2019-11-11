#ifndef UTILITY_H_
#define UTILITY_H_
#include <stdint.h>
uint8_t asciiToUint8(const char str[]);
int32_t abs(int32_t a);
int validateTime(uint16_t hrs,uint16_t min,uint16_t sec);
void waitMicrosecond(uint32_t us);
int validateDate(uint16_t mth,uint16_t day,uint16_t yr);
uint32_t asciiToUint32(const char str[]);

#endif
