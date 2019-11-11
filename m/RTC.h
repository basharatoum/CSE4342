#ifndef RTC_H_
#define RTC_H_
#include <stdint.h>
void initRTC();
void resetRTC();
void startMatch(uint32_t T);
void HibIsr();
#endif
