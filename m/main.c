// Gyroscope project
// Ulysses Aguilar
// Bashar Al Atom
//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL with LCD/Temperature Sensor
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz


// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1
//
//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "tm4c123gh6pm.h"
#include <string.h>
#include "UART0.h"
#include "RTC.h"
#include "Utility.h"
#include "I2C0.h"
#include "MPU.h"
#define  MAX_CHARS 100

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

// array to store input
char strinput[MAX_CHARS];
// array of char* to tokenize string
char* tokens[10];
// this is used to store the number of arguments
uint8_t NArgs = 0;
uint32_t RTC_Old=0;
uint16_t currPage;
uint32_t state=0,startingState=0;
uint32_t T=1;
uint8_t NSamples=0;

uint16_t daysOfEachMonth[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
struct time storedTime;
struct date storedDate;
uint8_t Para=4,LTflag,Hflag=0;
float level,H=0;
uint8_t logMask=0;
uint8_t sleepflag;
uint16_t currOffset;
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S)
            | SYSCTL_RCC_USEPWMDIV | SYSCTL_RCC_PWMDIV_2;
    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // intiate GPIOF_0

    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF;




    // Enable clocks
    initUART0();
    initI2c0();
    initRTC();
    initTemp();
    initMPU9250();
}

// function called to parse the command string
void parse_string()
{
    // Returns first token
    tokens[0] = strtok(strinput, " ,");
    NArgs =0;
    // Keep printing tokens while one of the
    // delimiters present in str[].
    while (tokens[NArgs] != NULL)
    {
        NArgs++;
        tokens[NArgs] = strtok(NULL, " ,");
    }

}
// returns are the value of the nth argument
uint16_t getValue(uint8_t argNumber)
{
    return atoi(tokens[argNumber+1]);
}

// returns the nth argument as a string
char * getString(uint8_t argNumber)
{
    return tokens[argNumber+1];
}

// checks if the strinput is a command with minArgs number of arguments
bool isCommand(const char * strcmd, uint8_t minArgs)
{
    if(strcmp(tokens[0],strcmd)==0 && NArgs>=minArgs )
    {
        return 1;
    }
    return 0;
}

void newDateFromDay(uint16_t daysToAdd){
        storedDate.day += daysToAdd;
        while(storedDate.day>daysOfEachMonth[storedDate.mth-1]){
            if((storedDate.yr%4==0)^(storedDate.yr%100==0)^(storedDate.yr%400==0)){
                        if(storedDate.mth == 2 &&storedDate.day>daysOfEachMonth[1]+1){
                            storedDate.mth+=1;
                            storedDate.day -=daysOfEachMonth[1]+1;
                        }
            }else{
                storedDate.day-=daysOfEachMonth[storedDate.mth-1];
                storedDate.mth+=1;
            }

            if(storedDate.mth>=12){
                storedDate.yr+=1;
                storedDate.mth-=12;
            }
        }
}
void setStoredTimeAndDate(uint32_t RTC){
    RTC += (storedTime.hrs*3600+storedTime.min*60+storedTime.sec);
    storedTime.hrs = (RTC/3600)%24;
    storedTime.min = (RTC/60)%60;
    storedTime.sec = (RTC)%60;
    uint16_t days =  (RTC/86400);
    newDateFromDay(days);
}


void printTime(uint32_t RTC){
    char str[60];
    RTC += (storedTime.hrs*3600+storedTime.min*60+storedTime.sec);
    sprintf(str, "%d:%d:%d\n",(RTC/3600)%24,(RTC/60)%60,(RTC)%60);
    putsUart0(str);
}
void printDate(uint32_t RTC){
    char str[60];
    uint16_t day,mth,yr;
    RTC += (storedTime.hrs*3600+storedTime.min*60+storedTime.sec);
    day = storedDate.day+ RTC/86400;
    mth = storedDate.mth;
    yr = storedDate.yr;
    while(day>daysOfEachMonth[mth-1]){
        if((yr%4==0)^(yr%100==0)^(yr%400==0)){
                    if(mth == 2 &&day>daysOfEachMonth[1]+1){
                        mth+=1;
                        day -=daysOfEachMonth[1]+1;
                    }
        }else{
            day-=daysOfEachMonth[mth-1];
            mth+=1;
        }

        if(mth>=12){
            yr+=1;
            mth-=12;
        }
    }
    sprintf(str,"%d / %d / %d\n",mth,day,yr);
    putsUart0(str);
}
int main(void)
 {
    // Initialize hardware
    initHw();
    int16_t i = 0,j;
    uint8_t add,reg,data;
    char str[60];       // str is used to be able to print the raw value to
                       // the UART
    storedDate.day = 3;
    storedDate.mth = 10;
    storedDate.yr = 2019;
    storedTime.sec = 0;
    storedTime.min = 48;
    storedTime.hrs = 11;

    while(1) // infinite loop
    {
        putsUart0("\t\t********\t\t\r\n");
        getsUart0(&strinput,100); // get the input from the UART
        putsUart0("\r\n");
        parse_string(); // tokenize the input
        if (isCommand("time",3))
        {
            if (validateTime(atoi(tokens[1]),atoi(tokens[2]),atoi(tokens[3])))
            {
            storedTime.hrs  = atoi(tokens[1]);
            storedTime.min  = atoi(tokens[2]);
            storedTime.sec  = atoi(tokens[3]);
            resetRTC();
            }
            else
            {
                sprintf(str, "Input time is wrong. Please ensure that the time is correct.\n");
                putsUart0(str);
            }
        }
        else if (isCommand("time",0)){
            printTime(HIB_RTCC_R);

        }
        else if(isCommand("date",3))  // if the command is an rgb command
        {
            if (validateDate(atoi(tokens[1]),atoi(tokens[2]),atoi(tokens[3])))
            {
            storedDate.mth = atoi(tokens[1]);
            storedDate.day = atoi(tokens[2]);
            storedDate.yr = atoi(tokens[3]);
            resetRTC();
            }
            else
            {
                sprintf(str, "Input date is wrong. Please ensure that the date is correct.\n");
                putsUart0(str);
            }
        }
        else if(isCommand("date",0)){
            printDate(HIB_RTCC_R);
        }
        else if(isCommand("reset",0)){
            NVIC_APINT_R = NVIC_APINT_SYSRESETREQ|NVIC_APINT_VECTKEY;

        }
        else if (isCommand("write",3))
        {
            add = asciiToUint8(tokens[1]);
            reg = asciiToUint8(tokens[2]);
            data = asciiToUint8(tokens[3]);
            writeI2c0Register(add, reg, data);
            sprintf(str, "Writing 0x%02hhx to address 0x%02hhx, register 0x%02hhx\n", data, add, reg);
            putsUart0(str);
        }
        else if (isCommand("read",2))
        {
            add = asciiToUint8(tokens[1]);
            reg = asciiToUint8(tokens[2]);
            data = readI2c0Register(add, reg);
            sprintf(str, "Read 0x%02hhx from address 0x%02hhx, register 0x%02hhx\n", data, add, reg);
            putsUart0(str);
        }
        else if (isCommand("poll",0))
        {
            putsUart0("Devices found: ");
            for (i = 4; i < 119; i++)
            {
                if (pollI2c0Address(i))
                {
                    sprintf(str, "0x%02x ", i);
                    putsUart0(str);
                }
            }
            putsUart0("\n");
        }
        else if (isCommand("help",0))
        {
            putsUart0("poll\n");
            putsUart0("read ADD REG\n");
            putsUart0("write ADD REG DATA\n");
        }
        else if(isCommand("temp",0)){
            uint32_t g = getTemp();
            sprintf(str, "%d\n", g);
            putsUart0(str);
        }
        else if(isCommand("read",0)){
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
        }
        else if(isCommand("periodic",1)){
            T = asciiToUint32(tokens[1]);
            if(NSamples >0 && validateInput()){
                startMatch();
            }else{
                sprintf(str, "Please check all the input parameters are correct!\r\n");
                putsUart0(str);
            }

        }
        else if(isCommand("samples",1)){
            NSamples = asciiToUint8(tokens[1]);
            sprintf(str, "# of Samples parameter: %d \r\n",NSamples);
            putsUart0(str);
        }
        else if(isCommand("trigger",0))
        {
            if(NSamples >0 && validateInput()){
                startTrigger();
            }else{
                sprintf(str, "Please check all the input parameters are correct!\r\n");
                putsUart0(str);
            }
        }
        else if (isCommand("gating",3)){

            SetGating();
            sprintf(str, "Gating parameters: %d %d %f\r\n",Para,LTflag,level);
            putsUart0(str);
        }
        else if(isCommand("log",1)){
            setLogging();
            sprintf(str, "Logging mask: %d \r\n",logMask);
            putsUart0(str);
        }else if(isCommand("hyst",1))
        {
            Hflag = 0;
            H = asciiToFloat(tokens[1]);
            sprintf(str, "Hysterisis parameter: %f \r\n",H);
            putsUart0(str);
        }else if(isCommand("stop",0)){
            endMatch();
            stopTrigger();
        }else if(isCommand("state",0)){
            state = (getRandomStart()<<2)|(0x010000);                    //asciiToUint32(tokens[1]);
            sprintf(str, "State is: %d \r\n",state);
            putsUart0(str);
            startingState = state;
            currOffset = 0x0000;
            eraseFlash(state&0x3FC00);
        }else if(isCommand("next",0)){
            nextPage();
            sprintf(str, "next parameter: %u pp\t %d \r\n",state,state);
            putsUart0(str);
        }else if(isCommand("data",0)){
            printData();
        }else if(isCommand("states",1)){
            state = asciiToUint32(tokens[1]);
            sprintf(str, "State is: %d \r\n",state);
            putsUart0(str);
            startingState = state;
            currOffset = 0x0000;
            eraseFlash(state&0x3FC00);
        }else if(isCommand("sampless",0)){
            sprintf(str, "# of Samples parameter: %d \r\n",NSamples);
                       putsUart0(str);
        }

        // nullify the input string to take other inputs
        for(j =0;j<MAX_CHARS;++j)
        {
            strinput[j]='\0';
        }

    }
}
