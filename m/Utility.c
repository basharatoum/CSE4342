#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "Utility.h"
#include <stdio.h>
#include "UART0.h"
#include "MPU.h"
#include <math.h>
#include "Temp.h"

uint8_t asciiToUint8(const char str[])
{
    uint8_t data;
    if (str[0] == '0' && tolower(str[1]) == 'x')
        sscanf(str, "%hhx", &data);
    else
        sscanf(str, "%hhu", &data);
    return data;
}
float asciiToFloat(const char str[])
{
    float data;
    if (str[0] == '0' && tolower(str[1]) == 'x')
        sscanf(str, "%x", &data);
    else
        sscanf(str, "%f", &data);
    return data;
}


uint32_t asciiToUint32(const char str[])
{
    uint32_t data;
    if (str[0] == '0' && tolower(str[1]) == 'x')
        sscanf(str, "%x", &data);
    else
        sscanf(str, "%d", &data);
    return data;
}
// utility function used for delta command, returns absolute value
int32_t abs(int32_t a)
{
    if(a<0)
        return -1*a;
    return a;
}

float absfloat(float a){
    if (a<0.0)
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



float eucDis(int16_t x,int16_t y,int16_t z){
    return sqrt(x*x+y*y+z*z);
}

void Sample(){
    int16_t values[3];
    char str[60];
    readAccelData(values);
    sprintf(str, "Accel data - > %d   %d   %d\r\n", values[0], values[1], values[2]);
    putsUart0(str);
    readGyroData(values);
    sprintf(str, "Gyro data - > %d   %d   %d\r\n", values[0], values[1], values[2]);
    putsUart0(str);
    readMagData(values);
    sprintf(str, "Mag data - > %d   %d   %d\r\n", values[0], values[1], values[2]);
    putsUart0(str);
    NSamples-=1;
}
void SampleWrapper(){
    int16_t values[3];

    if(NSamples>0){
            if (Para == 4){
                    Sample();
            }else{
                // Accel
                if(Para == 0){
                    readAccelData(values);
                    if(absfloat(eucDis(values[0],values[1],values[2])-level)>=H)
                    if(LTflag==0){
                         if(eucDis(values[0],values[1],values[2])>=level&&Hflag ==0){
                             Sample();
                             if(H>0.0){
                                 Hflag=1;
                             }
                         }else if(Hflag ==1){
                             Hflag = 0;
                         }
                    }else if(LTflag==1){
                         if(eucDis(values[0],values[1],values[2])<level&&Hflag ==0){
                             Sample();
                             if(H>0){
                                 Hflag=1;
                             }
                         }else if(Hflag ==1){
                             Hflag = 0;
                         }
                    }
                }else if(Para == 1){
                    // Gyroscope
                    readGyroData(values);
                    if(absfloat(eucDis(values[0],values[1],values[2])-level)>=H)
                    if(LTflag==0){
                         if(eucDis(values[0],values[1],values[2])>=level&&Hflag ==0){
                             Sample();
                             if(H!=0){
                                 Hflag=1;
                             }
                         }else if(Hflag ==1){
                             Hflag = 0;
                         }
                    }else if(LTflag==1){
                         if(eucDis(values[0],values[1],values[2])<level&&Hflag ==0){
                             Sample();
                             if(H>0){
                                 Hflag=1;
                             }
                         }else if(Hflag ==1){
                             Hflag = 0;
                         }
                    }
                }else if(Para == 2){
                    // Magenatic
                    readMagData(values);
                    if(absfloat(eucDis(values[0],values[1],values[2])-level)>=H&&Hflag==0)
                    if(LTflag==0){
                         if(eucDis(values[0],values[1],values[2])>=level&&Hflag ==0){
                             Sample();
                             if(H>0){
                                 Hflag=1;
                             }
                         }else if(Hflag ==1){
                             Hflag = 0;
                         }
                    }else if(LTflag==1){
                         if(eucDis(values[0],values[1],values[2])<level&&Hflag ==0){
                             Sample();
                             if(H>0){
                                 Hflag=1;
                             }
                         }else if(Hflag ==1){
                             Hflag = 0;
                         }
                    }
                }
                else if(Para == 3){
                    // Temperature
                    uint32_t Temp = getTemp();
                    if(absfloat(Temp-level)>=H)
                    if(LTflag==0){
                         if(Temp>=level&&Hflag==0){
                             Sample();
                             if(H!=0){
                                 Hflag=1;
                             }
                         }else if(Hflag ==1){
                             Hflag = 0;
                         }
                    }else if(LTflag==1){
                         if(Temp<level&&Hflag==0){
                             Sample();
                             if(H!=0){
                                 Hflag=1;
                             }
                         }else if(Hflag ==1){
                             Hflag = 0;
                         }
                    }
                }

            }
        }

}

void MPUIsr(){
    GPIO_PORTF_ICR_R |= 0x01;
    SampleWrapper();
    if(NSamples <=0){
        stopTrigger();
    }
   // read int_status  to clear interrupt
   readI2c0Register(MPU9250, 0x3A);
}

int validateInput(){
    if(Para <0||Para>4){
        return 0;
    }else if(Para>=0 && Para <=3){
        if(level <0){
            return 0;
        }
        if(LTflag>1||LTflag<0){
            return 0;
        }
    }

    if (T<=0){
        return 0;
    }
    return 1;
}

