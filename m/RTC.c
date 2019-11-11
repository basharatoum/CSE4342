#include "tm4c123gh6pm.h"
#include "RTC.h"
#include <stdint.h>
#include <stdio.h>
#include "UART0.h"
#include "MPU.h"
void initRTC(){
    // Configure RTC
    HIB_CTL_R = HIB_CTL_CLK32EN | HIB_CTL_RTCEN;
    NVIC_EN1_R |= 1 << ((INT_HIBERNATE-16)-32);
}

void resetRTC(){
    while(!HIB_CTL_WRC&HIB_CTL_R);
    HIB_RTCLD_R = 0;
}

void startMatch(uint32_t T){
    HIB_CTL_R |=~HIB_CTL_RTCEN;
    while(!HIB_CTL_WRC&HIB_CTL_R);

    // set match value
    HIB_RTCM0_R = T+HIB_RTCC_R;
    while(!HIB_CTL_WRC&HIB_CTL_R);

    // interrupt mask for alarm
    HIB_IM_R = HIB_IM_RTCALT0;
    while(!HIB_CTL_WRC&HIB_CTL_R);

    HIB_CTL_R |= HIB_CTL_RTCEN;
    while(!HIB_CTL_WRC&HIB_CTL_R);
}

void HibIsr(void){
    char str[60];
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
    while(!HIB_CTL_WRC&HIB_CTL_R);
    HIB_IC_R = 1;
}
