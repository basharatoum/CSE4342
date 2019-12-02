#include "MPU.h"
#include "I2C0.h"
#include "tm4c123gh6pm.h"
#include "Utility.h"
void initMPU9250(){
    uint8_t c;

    writeI2c0Register(MPU9250,0x37,0xA2);

    c =     readI2c0Register(MPU9250, PWR_MGMT_1+1);
    c &=~0x3F;
    writeI2c0Register(MPU9250,PWR_MGMT_1+1,c);

    // Turn on all the sensors;
    writeI2c0Register(MPU9250,PWR_MGMT_1,0x00);

    // full scale Gyro config;
    writeI2c0Register(MPU9250,GYRO_CONFIG,0x18);
    // full scale accel config;
    writeI2c0Register(MPU9250,ACCEL_CONFIG,0x18);

    writeI2c0Register(MPU9250, CONFIG, 0x00);
    writeI2c0Register(MPU9250, SMPLRT_DIV, 0x00);

    writeI2c0Register(AK8963,0x0A,0x06);

    readI2c0Register(MPU9250, 0x3A);
}


void readAccelData(int16_t * destination)
{

  uint8_t rawData[6];  // x/y/z accel register data stored here

  // Read the six raw data registers into data array
  rawData[0] = readI2c0Register(MPU9250, ACCEL_XOUT_H);
  rawData[1] = readI2c0Register(MPU9250, ACCEL_XOUT_L);
  rawData[2] = readI2c0Register(MPU9250, ACCEL_YOUT_H);
  rawData[3] = readI2c0Register(MPU9250, ACCEL_YOUT_L);
  rawData[4] = readI2c0Register(MPU9250, ACCEL_ZOUT_H);
  rawData[5] = readI2c0Register(MPU9250, ACCEL_ZOUT_L);

  // Turn the MSB and LSB into a signed 16-bit value
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}

void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  // Read the six raw data registers sequentially into data array
  rawData[0] = readI2c0Register(MPU9250, GYRO_XOUT_H);
  rawData[1] = readI2c0Register(MPU9250, GYRO_XOUT_L);
  rawData[2] = readI2c0Register(MPU9250, GYRO_YOUT_H);
  rawData[3] = readI2c0Register(MPU9250, GYRO_YOUT_L);
  rawData[4] = readI2c0Register(MPU9250, GYRO_ZOUT_H);
  rawData[5] = readI2c0Register(MPU9250, GYRO_ZOUT_L);

  // Turn the MSB and LSB into a signed 16-bit value
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}

void readMagData(int16_t * destination){
    uint8_t rawData[7];  // x/y/z accel register data stored here

    // Start reading from X_out, 7 bytes. This will read 6 data registers and
    //  one status register which will tell whether there was an overflow
    //  in measurements
    // Move 7 registers from AK8963 reg to here
    rawData[0] = readI2c0Register(AK8963, 0x03);
    rawData[1] = readI2c0Register(AK8963, 0x04);
    rawData[2] = readI2c0Register(AK8963, 0x05);
    rawData[3] = readI2c0Register(AK8963, 0x06);
    rawData[4] = readI2c0Register(AK8963, 0x07);
    rawData[5] = readI2c0Register(AK8963, 0x08);
    rawData[6] = readI2c0Register(AK8963, 0x09);

    uint8_t flag = 0 & rawData[6];
    if(!(flag  & 0x08)){
        destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;
        destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;
        destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
    }
    destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;
    destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;
    destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}

void startTrigger(){
    waitMicrosecond(10000);
    readI2c0Register(MPU9250, 0x3A);

    uint8_t c;
    // set up wake on motion

    writeI2c0Register(MPU9250,0x37,0xA0);

    c =     readI2c0Register(MPU9250, PWR_MGMT_1+1);
    c &=~0x3F;
    c|=0x07;
    writeI2c0Register(MPU9250,PWR_MGMT_1+1,c);

    c = readI2c0Register(MPU9250, ACCEL_CONFIG2);
    c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
    c = c | 0x01;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
    // Write new ACCEL_CONFIG2 register value
    writeI2c0Register(MPU9250, ACCEL_CONFIG2, c);



    writeI2c0Register(MPU9250,0x38,0x40);

    c = readI2c0Register(MPU9250, MOT_DETECT_CTRL);
    c |= 0xC0;
    writeI2c0Register(MPU9250,MOT_DETECT_CTRL,c);


    writeI2c0Register(MPU9250,ACCEL_CONFIG2+2,0x0F);

    c = readI2c0Register(MPU9250, ACCEL_CONFIG2+1);
    c &= ~0x03;
    writeI2c0Register(MPU9250,ACCEL_CONFIG2+1,c);

    writeI2c0Register(MPU9250,PWR_MGMT_1,0x20);


    waitMicrosecond(10000);

    // Enable GPIO port F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF;
    waitMicrosecond(10000);


    GPIO_PORTF_LOCK_R = 0x4C4F434B;


    GPIO_PORTF_DEN_R|=1;
    GPIO_PORTF_DIR_R&=~(0x01);
    GPIO_PORTF_AFSEL_R = 0;
    GPIO_PORTF_CR_R|=0x01;

    GPIO_PORTF_PUR_R|=1;
    GPIO_PORTF_IS_R&=~(0x01);
    GPIO_PORTF_IBE_R&=~(0x01);
    GPIO_PORTF_IEV_R &=~0x01;
    GPIO_PORTF_ICR_R |= 0x01;
    NVIC_EN0_R |= 1<<(INT_GPIOF-16);
    waitMicrosecond(1000000);
    GPIO_PORTF_IM_R|=0x01;

    GPIO_PORTF_LOCK_R = 0;

}

void stopTrigger(){
    GPIO_PORTF_LOCK_R = 0x4C4F434B;
    GPIO_PORTF_CR_R|=0x01;
    GPIO_PORTF_IM_R&=~0x01;
    GPIO_PORTF_LOCK_R = 0;

    writeI2c0Register(MPU9250,0x38,0x00);
    writeI2c0Register(MPU9250,PWR_MGMT_1,0x00);

    readI2c0Register(MPU9250, 0x3A);
}
