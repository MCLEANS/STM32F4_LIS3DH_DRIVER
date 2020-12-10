#include "stm32f4xx.h"
#include "clockconfig.h"
#include "SPI_16bit.h"
#include "NOKIA_5110.h"
#include <stdlib.h>

#define SCK_PIN 5
#define MOSI_PIN 7
#define MISO_PIN 6

#define CS_PORT GPIOE
#define CS_PIN 3

//MOTION SENSOR COMMANDS
#define WHO_AM_I 0x0F
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24
#define STATUS_REG 0x27
#define OUT_X_L 0x28
#define OUT_X_H 0x29
#define OUT_Y_L 0x2B
#define OUT_Y_H 0x2B
#define OUT_Z_L 0X2D
#define OUT_Z_H 0X2D

#define LIS3DH_ID 63

int16_t mydata_x = 20;
int16_t mydata_y = 20;
int16_t mydata_z = 20;


custom_libraries::clock_config system_clock;
custom_libraries_1::_SPI motion_sensor(SPI1,
                                    GPIOA,
                                    SCK_PIN,
                                    MOSI_PIN,
                                    MISO_PIN,
                                    16,
                                    true,
                                    true,
                                    false);

#define NOKIA_RST_PORT GPIOD
#define NOKIA_RST_PIN 0
#define NOKIA_CS_PORT GPIOD
#define NOKIA_CS_PIN 1
#define NOKIA_DC_PORT GPIOD
#define NOKIA_DC_PIN 2

custom_libraries::NOKIA_5110 NOKIA(SPI2,
                                    GPIOB,
                                    13,
                                    15,
                                    0,
                                    64,
                                    false,
                                    false,
                                    false,
                                    NOKIA_CS_PORT,
                                    NOKIA_CS_PIN,
                                    NOKIA_RST_PORT,
                                    NOKIA_RST_PIN,
                                    NOKIA_DC_PORT,
                                    NOKIA_DC_PIN);

void set_cs_pin(){
  CS_PORT->ODR |= (1 << CS_PIN);
}

void reset_cs_pin(){
  CS_PORT->ODR &= ~(1 << CS_PIN);
}

int main(void) {
  
  system_clock.initialize();

  //Initialize CHIP SELECT PIN
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
  //CONFIGURE AS GENERAL PURPOSE OUTPUT 
  CS_PORT->MODER |= GPIO_MODER_MODER3_0;
  //SET TO VERY HIGH SPEED
  CS_PORT->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR3;
  set_cs_pin();
  
  //Initialize the sensor

  /**
   * Configure Control register 0
   * 1. Initialize the register as indicated in datasheet (This is needed for the sensor to function optimally)
   */

  reset_cs_pin();
  motion_sensor.write((0x1E << 8)| 0x10);
  set_cs_pin();

  /**
   * configure control register 1
   * 1. Set Normal mode. 
   * 2. Set 100Hz sampling rate
   */
  reset_cs_pin();
  motion_sensor.write((0x20 << 8)| 0x57);
  set_cs_pin();

  /**
   * Configure Control Register 4
   * 1. Clear HR bit to set Normal Mode
   * 2. Spi Interface mode selection (Set to 4-wire).
   * 3. Full scale selected to +-2g
   */

  reset_cs_pin();
  motion_sensor.write((0x23 << 8)| 0x00);
  set_cs_pin();

  //Read from the WHO_AM_I register
  reset_cs_pin();
  uint16_t mydata_whoami = motion_sensor.read(((0x80 | 0x0F) << 8));
  set_cs_pin();
  mydata_whoami &=  ~(0xFF << 8);

  while(1){

  /**
   *  read Raw values for the Y-AXIS
   */

  reset_cs_pin();
  mydata_y = motion_sensor.read(((0x80 | 0x2A) << 8));
  set_cs_pin();
  mydata_y &=  ~(0xFF << 8);

  reset_cs_pin();
  uint16_t mydata1_y = motion_sensor.read(((0x80 | 0x2B) << 8));
  set_cs_pin();
  uint16_t tempo_y = mydata1_y;
  mydata1_y &=  ~(0xFF80);

  mydata_y |= (mydata1_y << 8);
  if(tempo_y & (1 << 7)){
    mydata_y = 0-mydata_y;
  }

  /**
   * read Raw data from the X-AXIS
   **/
   reset_cs_pin();
  mydata_x = motion_sensor.read(((0x80 | 0x28) << 8));
  set_cs_pin();
  mydata_x &=  ~(0xFF << 8);

  reset_cs_pin();
  uint16_t mydata1_x = motion_sensor.read(((0x80 | 0x29) << 8));
  set_cs_pin();
  uint16_t tempo_x = mydata1_x;
  mydata1_x &=  ~(0xFF80);

  mydata_x |= (mydata1_x << 8);
  if(tempo_x & (1 << 7)){
    mydata_x = 0-mydata_x;
  }

    /**
   * read Raw data from the Z-AXIS
   **/
   reset_cs_pin();
  mydata_z = motion_sensor.read(((0x80 | 0x2C) << 8));
  set_cs_pin();
  mydata_z &=  ~(0xFF << 8);

  reset_cs_pin();
  uint16_t mydata1_z = motion_sensor.read(((0x80 | 0x2D) << 8));
  set_cs_pin();
  uint16_t tempo_z = mydata1_z;
  mydata1_z &=  ~(0xFF80);

  mydata_z |= (mydata1_z << 8);
  if(tempo_z & (1 << 7)){
    mydata_z = 0-mydata_z;
  }


    //read_accel_values();
    char received_y[4];
    char received_x[4];
    char received_z[4];

    itoa(mydata_y,received_y,10);
    NOKIA.print(received_y,5,2);

    itoa(mydata_x,received_x,10);
    NOKIA.print(received_x,5,1);

    itoa(mydata_z,received_z,10);
    NOKIA.print(received_z,5,3);

    for(volatile int i = 0; i < 5000000; i++){}
    NOKIA.clear();
  }
}
