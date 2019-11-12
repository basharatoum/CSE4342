#include "tm4c123gh6pm.h"
#include "RTC.h"
#include <stdint.h>
#include <stdio.h>
#include "UART0.h"
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

void startMatch(uint32_t T){
    char str[60];

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

void HibIsr(){
    char str[60];

    sprintf(str, "Starting\r\n");
    putsUart0(str);

    uint32_t status = HIB_MIS_R;
    while (!(HIB_CTL_R & HIB_CTL_WRC));

    if (status & HIB_MIS_RTCALT0)
    {
    sprintf(str, "\r\nSample number \r\n");
    putsUart0(str);
    }

    HIB_IC_R |= status;
    while (!(HIB_CTL_R & HIB_CTL_WRC));

}
