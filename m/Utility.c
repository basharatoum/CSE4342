#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "Utility.h"

uint8_t asciiToUint8(const char str[])
{
    uint8_t data;
    if (str[0] == '0' && tolower(str[1]) == 'x')
        sscanf(str, "%hhx", &data);
    else
        sscanf(str, "%hhu", &data);
    return data;
}


uint32_t asciiToUint32(const char str[])
{
    uint32_t data;
    if (str[0] == '0' && tolower(str[1]) == 'x')
        sscanf(str, "%x", &data);
    else
        sscanf(str, "%u", &data);
    return data;
}
// utility function used for delta command, returns absolute value
int32_t abs(int32_t a)
{
    if(a<0)
        return -1*a;
    return a;
}

// function to wait for a period of time
void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}


int validateTime(uint16_t hrs,uint16_t min,uint16_t sec){
    if(hrs>=0 && hrs<24)
        if(min>=0 && min<60)
            if(sec >= 0 && sec<60)
                return 1;
    return 0;
}

int validateDate(uint16_t mth,uint16_t day,uint16_t yr){
    uint16_t daysOfEachMonth[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
    if(mth>0 && mth<=12)
        {
        if(day>0 && day<=daysOfEachMonth[mth-1]&& mth !=2)
            if(yr>=0)
                return 1;

        if(mth == 2 && yr> 0 && (yr%4==0)^(yr%100==0)^(yr%400==0)&& day>0 && day <daysOfEachMonth[mth-1]+1)
            return 1;
        }
    return 0;
}
