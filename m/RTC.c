#include "tm4c123gh6pm.h"
#include "RTC.h"
#include <stdint.h>
#include <stdio.h>
#include "UART0.h"
#include "Utility.h"
#include "Temp.h"
#include "MPU.h"

//periodic temperature

void initRTC(){
    // Configure RTC
    HIB_CTL_R = HIB_CTL_CLK32EN | HIB_CTL_RTCEN;
    NVIC_EN1_R |= 1 << (INT_HIBERNATE - 16 - 32);
    while(!HIB_CTL_WRC&HIB_CTL_R);
    return;
}

void resetRTC(){
    while(!HIB_CTL_WRC&HIB_CTL_R);
    HIB_RTCLD_R = 0;
}

void startMatch(){

    // Set match value
    HIB_RTCM0_R = HIB_RTCC_R + T/1000;
    while (!(HIB_CTL_R & HIB_CTL_WRC));

    HIB_RTCSS_R = (uint16_t)(T%1000)*32768/1000 << HIB_RTCSS_RTCSSM_S;
    while (!(HIB_CTL_R & HIB_CTL_WRC));

    // interrupt mask for alarm
    HIB_IM_R = HIB_IM_RTCALT0;
    while (!(HIB_CTL_R & HIB_CTL_WRC));


    HIB_RTCLD_R = HIB_RTCC_R;
    while(!HIB_CTL_WRC&HIB_CTL_R);

    HIB_RTCT_R = 0x7FFF;
    while (!(HIB_CTL_R & HIB_CTL_WRC));


    startRTC();
}
void setupHibCtl(){
}

void endMatch(){
   // interrupt mask for alarm
   HIB_IM_R &= ~HIB_IM_RTCALT0;
   while (!(HIB_CTL_R & HIB_CTL_WRC));


   HIB_RTCLD_R = HIB_RTCC_R;
   while(!HIB_CTL_WRC&HIB_CTL_R);

   HIB_RTCT_R = 0x7FFF;
   while (!(HIB_CTL_R & HIB_CTL_WRC));

   startRTC();
}


void storeData(){

   /*
    * Hflag
    * Ltflag
    *  extern uint16_t currOffset;
    extern uint8_t NSamples;
    extern uint8_t Para,;
    extern float level,H;
    extern struct time storedTime;
    extern struct date storedDate;
    extern uint32_t T;
    extern uint8_t logMask;
    extern uint32_t state,startingState;*/

    uint32_t i =0;
    for(i=0;i<16;++i){
        switch(i){
        case 0:
           HIB_DATA_R |= currOffset;
           while (!(HIB_CTL_R & HIB_CTL_WRC));
           break;
        case 1:
            HWREG(0x400FC030 + (i*4))= (uint32_t)NSamples;
            while (!(HIB_CTL_R & HIB_CTL_WRC));

            break;
        case 2:
            HWREG(0x400FC030 + (i*4))= (uint32_t)Para;
            while (!(HIB_CTL_R & HIB_CTL_WRC));

            break;
        case 3:
            HWREG(0x400FC030 + (i*4))= (uint32_t)level;
            while (!(HIB_CTL_R & HIB_CTL_WRC));

            break;
        case 4:
            HWREG(0x400FC030 + (i*4))= (uint32_t)H;
            while (!(HIB_CTL_R & HIB_CTL_WRC));

            break;
        case 5:
            HWREG(0x400FC030 + (i*4)) = (uint32_t)storedTime.hrs;
            while (!(HIB_CTL_R & HIB_CTL_WRC));

            break;
        case 6:
            HWREG(0x400FC030 + (i*4)) = (uint32_t)storedTime.min;
            while (!(HIB_CTL_R & HIB_CTL_WRC));

            break;
        case 7:
            HWREG(0x400FC030 + (i*4)) = (uint32_t)storedTime.sec;
            while (!(HIB_CTL_R & HIB_CTL_WRC));

            break;
        case 8:
            HWREG(0x400FC030 + (i*4)) = (uint32_t)storedDate.day;
            while (!(HIB_CTL_R & HIB_CTL_WRC));

            break;
        case 9:
            HWREG(0x400FC030 +(i*4)) =(uint32_t)storedDate.mth;
            while (!(HIB_CTL_R & HIB_CTL_WRC));

            break;
        case 10:
            HWREG(0x400FC030 + (i*4)) = (uint32_t)storedDate.yr;
            while (!(HIB_CTL_R & HIB_CTL_WRC));

            break;
        case 11:
            HWREG(0x400FC030 + (i*4)) =(uint32_t) T;
            while (!(HIB_CTL_R & HIB_CTL_WRC));

            break;
        case 12:
            HWREG(0x400FC030 + (i*4)) = (uint32_t)logMask | (LTflag<<6)|(Hflag<<7)|(Trigflag<<8);
            while (!(HIB_CTL_R & HIB_CTL_WRC));

            break;
        case 13:
            HWREG(0x400FC030 + (i*4)) = (uint32_t)state;
            while (!(HIB_CTL_R & HIB_CTL_WRC));

            break;
        case 14:
            HWREG(0x400FC030 + (i*4)) = (uint32_t)startingState;
            while (!(HIB_CTL_R & HIB_CTL_WRC));

            break;
        }

    }

    return;
}

void retrieveData(){
    char str[60];
    uint8_t i =0;
    for(i=0;i<16;++i){
        switch(i){
        case 0:
            currOffset = HWREG(0x400FC030 + (i*4));
            break;
        case 1:
            NSamples = HWREG(0x400FC030 + (i*4));
            break;
        case 2:
            Para = HWREG(0x400FC030 + (i*4));
            break;
        case 3:
            level = HWREG(0x400FC030 + (i*4));
            break;
        case 4:
            H = HWREG(0x400FC030 + (i*4));
            break;
        case 5:
            storedTime.hrs =HWREG(0x400FC030 + (i*4));
            break;
        case 6:
            storedTime.min=HWREG(0x400FC030 + (i*4));
            break;
        case 7:
            storedTime.sec=HWREG(0x400FC030 + (i*4));
            break;
        case 8:
            storedDate.day=HWREG(0x400FC030 + (i*4));
            break;
        case 9:
            storedDate.mth=HWREG(0x400FC030 + (i*4));
            break;
        case 10:
            storedDate.yr=HWREG(0x400FC030 + (i*4));
            break;
        case 11:
             T=HWREG(0x400FC030 + (i*4));
            break;
        case 12:
            logMask = 0x3F & HWREG(0x400FC030 + (i*4));
            LTflag = (0x040&HWREG(0x400FC030 + (i*4)))>>6;
            Hflag = (0x080&HWREG(0x400FC030 + (i*4)))>>7;
            Trigflag = (0x100&HWREG(0x400FC030 + (i*4)))>>8;
            break;
        case 13:
            state=HWREG(0x400FC030 + (i*4));
            break;
        case 14:
            startingState=HWREG(0x400FC030 + (i*4));
            break;
        }

    }
    return;
}

void startRTC(){
    if(logMask&0x20){

        HIB_CTL_R = HIB_CTL_CLK32EN|HIB_CTL_RTCEN;
        while(!HIB_CTL_WRC&HIB_CTL_R);

        HIB_CTL_R = HIB_CTL_CLK32EN | HIB_CTL_RTCEN|HIB_CTL_RTCWEN|HIB_CTL_PINWEN|HIB_CTL_VDD3ON;
        while (!(HIB_CTL_R & HIB_CTL_WRC));
        HIB_CTL_R|=HIB_CTL_HIBREQ;
        while (!(HIB_CTL_R & HIB_CTL_WRC));
        while(1)
        {

        }
    }
    return;
}
void stopRTC(){
    HIB_CTL_R = HIB_CTL_CLK32EN;
    while(!(HIB_CTL_WRC&HIB_CTL_R));
}
void HibIsr(){
    char str[60];
    uint32_t status = HIB_MIS_R;


    if (status & HIB_MIS_RTCALT0==0x01)
    {
        HIB_IC_R |= status;
        while (!(HIB_CTL_R & HIB_CTL_WRC));

        retrieveData();
        if(logMask&0x20){
        initTemp();
        }
        SampleWrapper();

        storeData();
        if (NSamples>0){
            startMatch();
        }else if(NSamples <=0){
            endMatch();
        }
    }else if(status&HIB_MIS_EXTW){
        HIB_IC_R |= status;
        while (!(HIB_CTL_R & HIB_CTL_WRC));

        retrieveData();

        readI2c0Register(MPU9250, 0x3A);

        if(logMask&0x20){
        initTemp();
        }



    }
}
void HibSleep(){
    logMask|=0x20;
    storeData();
    HIB_IM_R |= HIB_IM_EXTW;

    HIB_CTL_R = HIB_CTL_CLK32EN | HIB_CTL_RTCEN|HIB_CTL_RTCWEN|HIB_CTL_PINWEN|HIB_CTL_VDD3ON;
    while (!(HIB_CTL_R & HIB_CTL_WRC));
    HIB_CTL_R|=HIB_CTL_HIBREQ;
    while (!(HIB_CTL_R & HIB_CTL_WRC));
    while(1)
    {

    }
}
