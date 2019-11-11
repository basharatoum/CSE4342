#include "Temp.h"
#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "Utility.h"
void initTemp(){
    SYSCTL_RCGCADC_R |= 1;       /* enable clock to ADC0 */
    waitMicrosecond(10000);

    ADC0_CC_R=ADC_CC_CS_SYSPLL;
    waitMicrosecond(10000);

    /* initialize ADC0 */
    ADC0_ACTSS_R &= ~8;          /* disable SS3 during configuration */
    ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;
    ADC0_SSMUX3_R = 0;           /* get input from channel 0 */
    waitMicrosecond(10000);

    ADC0_SSCTL3_R |= 0x0A;       /* take chip temperature, set flag at 1st sample */
    ADC0_ACTSS_R |= 8;           /* enable ADC0 sequencer 3 */
}
uint32_t getTemp(){
    uint32_t temperature;
    ADC0_PSSI_R |=ADC_PSSI_SS3;
    while(ADC0_ACTSS_R&ADC_ACTSS_BUSY);                    /* wait for conversion complete */
      temperature = 147 - (247 * ADC0_SSFIFO3_R) / 4096;
      return temperature;
}
