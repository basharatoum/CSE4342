#include "tm4c123gh6pm.h"
#include "RTC.h"
#include <stdint.h>
#include <stdio.h>
#include "UART0.h"
#include "Utility.h"
#include "MPU.h"


void initRTC(){
    // Configure RTC
    HIB_CTL_R = HIB_CTL_CLK32EN | HIB_CTL_RTCEN;
    NVIC_EN1_R |= 1 << (INT_HIBERNATE - 16 - 32);
}

void resetRTC(){
    while(!HIB_CTL_WRC&HIB_CTL_R);
    HIB_RTCLD_R = 0;
}

void startMatch(){
    HIB_CTL_R = HIB_CTL_CLK32EN;
    while(!(HIB_CTL_WRC&HIB_CTL_R));

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
    while (!(HIB_CTL_R & HIB_CTL_WRC))

    HIB_CTL_R = HIB_CTL_CLK32EN | HIB_CTL_RTCEN;
    while (!(HIB_CTL_R & HIB_CTL_WRC));
}

void endMatch(){
    HIB_CTL_R = HIB_CTL_CLK32EN;
    while(!(HIB_CTL_WRC&HIB_CTL_R));

   // interrupt mask for alarm
   HIB_IM_R &= ~HIB_IM_RTCALT0;
   while (!(HIB_CTL_R & HIB_CTL_WRC));


   HIB_RTCLD_R = HIB_RTCC_R;
   while(!HIB_CTL_WRC&HIB_CTL_R);

   HIB_RTCT_R = 0x7FFF;
   while (!(HIB_CTL_R & HIB_CTL_WRC))

   HIB_CTL_R = HIB_CTL_CLK32EN | HIB_CTL_RTCEN;
   while (!(HIB_CTL_R & HIB_CTL_WRC));
}


void HibIsr(){
    char str[60];
    uint32_t status = HIB_MIS_R;
    while (!(HIB_CTL_R & HIB_CTL_WRC));

    if (status & HIB_MIS_RTCALT0)
    {
        if(NSamples > 0){
            sprintf(str, "Data %d\r\n", NSamples);
            putsUart0(str);
            int16_t values[3];
            readAccelData(values);
            sprintf(str, "Accel data - > %d   %d   %d\r\n", values[0], values[1], values[2]);
            putsUart0(str);
            readGyroData(values);
            sprintf(str, "Gyro data - > %d   %d   %d\r\n", values[0], values[1], values[2]);
            putsUart0(str);
            readMagData(values);
            sprintf(str, "Mag data - > %d   %d   %d\r\n", values[0], values[1], values[2]);
            putsUart0(str);
            NSamples--;
            sprintf(str, "*********\r\n");
            putsUart0(str);
            startMatch();
        }else {
            endMatch();
        }

    }


    HIB_IC_R |= status;
    while (!(HIB_CTL_R & HIB_CTL_WRC));

}
