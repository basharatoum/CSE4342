#ifndef UTILITY_H_
#define UTILITY_H_
#include <stdint.h>
uint8_t asciiToUint8(const char str[]);
int32_t abs(int32_t a);
int validateTime(uint16_t hrs,uint16_t min,uint16_t sec);
void waitMicrosecond(uint32_t us);
int validateDate(uint16_t mth,uint16_t day,uint16_t yr);
uint32_t asciiToUint32(const char str[]);
float asciiToFloat(const char str[]);
void MPUIsr();
void Sample();
float eucDis(int16_t x,int16_t y,int16_t z);
int validateInput();
void SampleWrapper();
float absfloat(float a);
void SetGating();
void setLogging();
uint32_t nextPage();
struct time{
    uint16_t hrs;
    uint16_t min;
    uint16_t sec;
};
struct date{
    uint16_t mth;
    uint16_t day;
    uint16_t yr;
};
extern uint8_t NSamples;
extern uint8_t Para,LTflag,Hflag;
extern float level,H;
extern uint8_t sleepflag;
extern struct time storedTime;
extern struct date storedDate;
extern uint32_t T;
extern char* tokens[10];
extern uint8_t logMask;
extern uint32_t state,startingState;
#endif
