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
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "tm4c123gh6pm.h"
#include <string.h>

// PortB masks
#define SDA_MASK 8
#define SCL_MASK 4
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

uint16_t daysOfEachMonth[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
struct time storedTime;
struct date storedDate;
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initI2c0()
{
    // Enable clocks
    SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB;

    // Configure I2C
    GPIO_PORTB_DIR_R |= SDA_MASK | SCL_MASK;            // make bits 2 and 3 outputs
    GPIO_PORTB_DR2R_R |= SDA_MASK | SCL_MASK;           // set drive strength to 2mA
    GPIO_PORTB_DEN_R |= SDA_MASK | SCL_MASK;            // enable digital
    GPIO_PORTB_ODR_R |= SDA_MASK;                       // enable open drain outputs
    GPIO_PORTB_AFSEL_R |= SDA_MASK | SCL_MASK;          // configure auxiliary fn
    GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB2_M | GPIO_PCTL_PB3_M);
    GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB2_I2C0SCL | GPIO_PCTL_PB3_I2C0SDA;

    // Configure I2C0 peripheral
    I2C0_MCR_R = I2C_MCR_MFE;                           // master
    I2C0_MTPR_R = 19;                                   // (40MHz/2) / (19+1) = 100kbps
}

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
// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S)
            | SYSCTL_RCC_USEPWMDIV | SYSCTL_RCC_PWMDIV_2;
    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable clocks
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOF;

                                                     // turn on GPIO ports A and E
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other UARTs in same status

    // Configure UART0 pins
    GPIO_PORTA_DIR_R |= 2;                           // enable output on UART0 TX pin: default, added for clarity
    GPIO_PORTA_DEN_R |= 3;                           // enable digital on UART0 pins: default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                         // use peripheral to drive PA0, PA1: default, added for clarity
    GPIO_PORTA_PCTL_R &= 0xFFFFFF00;                 // set fields for PA0 and PA1 to zero
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;
                                                     // select UART0 to drive pins PA0 and PA1: default, added for clarity

    // Configure UART0 to 115200 baud, 8N1 format
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
                                                     // enable TX, RX, and module
    // Configure RTC
    HIB_CTL_R = HIB_CTL_CLK32EN | HIB_CTL_RTCEN ;



}
uint8_t asciiToUint8(const char str[])
{
    uint8_t data;
    if (str[0] == '0' && tolower(str[1]) == 'x')
        sscanf(str, "%hhx", &data);
    else
        sscanf(str, "%hhu", &data);
    return data;
}


void writeI2c0Register(uint8_t add, uint8_t reg, uint8_t data)
{
    I2C0_MSA_R = add*2;
    I2C0_MDR_R = reg;
    I2C0_MICR_R = I2C_MICR_IC;
    I2C0_MCS_R = I2C_MCS_START | I2C_MCS_RUN;
    while ((I2C0_MRIS_R & I2C_MRIS_RIS) == 0);
    I2C0_MDR_R = data;
    I2C0_MICR_R = I2C_MICR_IC;
    I2C0_MCS_R = I2C_MCS_STOP | I2C_MCS_RUN;
    while (!(I2C0_MRIS_R & I2C_MRIS_RIS));
}

void writeI2c0Registers(uint8_t add, uint8_t reg, uint8_t data[], uint8_t size)
{
    uint8_t i;
    I2C0_MSA_R = add*2;
    I2C0_MDR_R = reg;
    if (size == 0)
    {
        I2C0_MICR_R = I2C_MICR_IC;
        I2C0_MCS_R = I2C_MCS_STOP | I2C_MCS_START | I2C_MCS_RUN;
        while ((I2C0_MRIS_R & I2C_MRIS_RIS) == 0);
    }
    else
    {
        I2C0_MICR_R = I2C_MICR_IC;
        I2C0_MCS_R = I2C_MCS_START | I2C_MCS_RUN;
        while ((I2C0_MRIS_R & I2C_MRIS_RIS) == 0);
        for (i = 0; i < size-1; i++)
        {
            I2C0_MDR_R = data[i];
            I2C0_MICR_R = I2C_MICR_IC;
            I2C0_MCS_R = I2C_MCS_RUN;
            while ((I2C0_MRIS_R & I2C_MRIS_RIS) == 0);
        }
        I2C0_MDR_R = data[size-1];
        I2C0_MICR_R = I2C_MICR_IC;
        I2C0_MCS_R = I2C_MCS_STOP | I2C_MCS_RUN;
        while ((I2C0_MRIS_R & I2C_MRIS_RIS) == 0);
    }
}

uint8_t readI2c0Register(uint8_t add, uint8_t reg)
{
    I2C0_MSA_R = add*2;
    I2C0_MDR_R = reg;
    I2C0_MICR_R = I2C_MICR_IC;
    I2C0_MCS_R = I2C_MCS_START | I2C_MCS_RUN;
    while ((I2C0_MRIS_R & I2C_MRIS_RIS) == 0);
    I2C0_MSA_R = add*2 + 1;
    I2C0_MICR_R = I2C_MICR_IC;
    I2C0_MCS_R = I2C_MCS_STOP | I2C_MCS_START | I2C_MCS_RUN;
    while ((I2C0_MRIS_R & I2C_MRIS_RIS) == 0);
    return I2C0_MDR_R;
}

bool pollI2c0Address(uint8_t add)
{
    I2C0_MSA_R = add*2 + 1;
    I2C0_MICR_R = I2C_MICR_IC;
    I2C0_MCS_R = I2C_MCS_STOP | I2C_MCS_START | I2C_MCS_RUN;
    while ((I2C0_MRIS_R & I2C_MRIS_RIS) == 0);
    return !(I2C0_MCS_R & I2C_MCS_ERROR);
}

bool isI2c0Error()
{
    return !(I2C0_MCS_R & I2C_MCS_ERROR);
}

// utility function used for delta command, returns absolute value
int32_t abs(int32_t a)
{
    if(a<0)
        return -1*a;
    return a;
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);               // wait if uart0 tx fifo full
    UART0_DR_R = c;                                  // write character to fifo
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
      putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);               // wait if uart0 rx fifo empty
    return UART0_DR_R & 0xFF;                        // get character from fifo
}
//derived from from c strtok function to tokenize string without including the library
char * strtokk(char * str, char *comp)
{
    static int pos;
    static char *s;
    int i =0, start = pos;

    // Copying the string for further calls of strtok
    if(str!=NULL)
        s = str;

    i = 0;
    int j = 0;
    //While not end of string
    while(s[pos] != '\0')
    {
        j = 0;
        //Comparing of one of the delimiter matches the character in the string
        while(comp[j] != '\0')
        {
            //Pos point to the next location in the string that we have to read
            if(s[pos] == comp[j])
            {
                //Replace the delimter by \0 to break the string
                s[pos] = '\0';
                pos = pos+1;
                //Checking for the case where there is no relevant string before the delimeter.
                //start specifies the location from where we have to start reading the next character
                if(s[start] != '\0')
                    return (&s[start]);
                else
                {
                    // Move to the next string after the delimiter
                    start = pos;
                    // Decrementing as it will be incremented at the end of the while loop
                    pos--;
                    break;
                }
            }
            j++;
        }
        pos++;
    }//End of Outer while
    s[pos] = '\0';
    if(s[start] == '\0')
        return NULL;
    else
        return &s[start];
}
//function that recives user input from uart
void getsUart0(char * str, uint8_t maxChars)
{
    // count for the number of characters in the string
    int count =0;

    // temp is used to be a temporary place holder for the incoming character
    char temp;
    while(count<maxChars-1)
    {
        // get the character from the RX end
        temp = getcUart0();
        if (temp==8&& count!=0)
        {
            // if it is a backspace take one of the count
            // (as if we are deleting the last char)
            count--;
            continue;
        }
        else if(temp ==13)
        {
            // if we get a carriage return, then this is the end of the string
            // null terminate the string and return
            temp ='\0';
            str[count]=temp;
            return;
        }
        else if(temp >=' ')
        {
            // else if the character has an ascii code above 32
            if(temp>='A'&&temp<='Z')
            {
                // by adding 32 to a capital letter we convert it into a smaller one
                temp+=32;
            }

            // store the character after editing it into the string.
            str[count]=temp;
        }
        // if we reached this point then we added a character to the string
        count++;
    }
    // null terminate the string and return, as this is the end.
    str[count]='\0';
    return;
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

void resetRTC(){
    while(!HIB_CTL_WRC&HIB_CTL_R);
    HIB_RTCLD_R = 0;
}
int validateTime(uint16_t hrs,uint16_t min,uint16_t sec){
    if(hrs>=0 && hrs<24)
        if(min>=0 && min<60)
            if(sec >= 0 && sec<60)
                return 1;
    return 0;
}
int validateDate(uint16_t mth,uint16_t day,uint16_t yr){
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
    // Initialize I2c
    initI2c0();
    // Initialize Temp sensor
    initTemp();
    int16_t i = 0;
    uint8_t add,reg,data;
    uint16_t raw; // r,g, and b are used to set the color of the leds
                       // raw is used to get the value of the anolog input
    uint16_t T=3500;  // the threshold value used for the calibrate command
    uint16_t j;       // i and j are used for the loops inside the main
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
            sprintf(str,"this is it -> %d\n",HIB_RTCC_R);
            putsUart0(str);
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

        if(isCommand("reset",0)){
            putsUart0("resetting");
            NVIC_APINT_R = NVIC_APINT_SYSRESETREQ|NVIC_APINT_VECTKEY;
            putsUart0("reseted");

        }
        // wait for 10 milliseconds to make sure the light is on

        if (isCommand("write",3))
        {
            add = asciiToUint8(tokens[1]);
            reg = asciiToUint8(tokens[2]);
            data = asciiToUint8(tokens[3]);
            writeI2c0Register(add, reg, data);
            sprintf(str, "Writing 0x%02hhx to address 0x%02hhx, register 0x%02hhx\n", data, add, reg);
            putsUart0(str);
        }
        if (isCommand("read",2))
        {
            add = asciiToUint8(tokens[1]);
            reg = asciiToUint8(tokens[2]);
            data = readI2c0Register(add, reg);
            sprintf(str, "Read 0x%02hhx from address 0x%02hhx, register 0x%02hhx\n", data, add, reg);
            putsUart0(str);
        }
        if (isCommand("poll",0))
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
        if (isCommand("help",0))
        {
            putsUart0("poll\n");
            putsUart0("read ADD REG\n");
            putsUart0("write ADD REG DATA\n");
        }

        if(isCommand("temp",0)){
            uint32_t g = getTemp();
            sprintf(str, "%d\n", g);
            putsUart0(str);
        }
        waitMicrosecond(10000);

        // nullify the input string to take other inputs
        for(j =0;j<MAX_CHARS;++j)
        {
            strinput[j]='\0';
        }

    }
}
