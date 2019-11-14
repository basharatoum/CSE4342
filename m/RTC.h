#ifndef RTC_H_
#define RTC_H_
#include <stdint.h>
void initRTC();
void resetRTC();
void startMatch();
void HibIsr();
void endMatch();
extern uint32_t T;
extern uint8_t NSamples;

#endif
