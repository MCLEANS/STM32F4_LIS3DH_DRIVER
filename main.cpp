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

int16_t mydata = 20;


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

uint16_t get_device_ID(){
  uint16_t device_ID;
  reset_cs_pin();
  //Read from the WHO_AM_I register
  device_ID = motion_sensor.read(((0x80 | WHO_AM_I) << 8));
  set_cs_pin();
  //clean up the received ID by removing the higher order 8 bits
  device_ID &= ~(0xFF << 8);

  return device_ID;
}

void initialize(){
  /**
   * 1.Set Control register 1 (20h) to High resolution Mode
   * 2.Enable X,Y and Z axis
   * 3.Configure data rate to 100Hz
   */
  reset_cs_pin();
  motion_sensor.write((20 << 8)| 0x5F);
  set_cs_pin();

  /**
   * 1. Enable High resolution mode in Control register 4 (23h)
   * 2. Set full scale selection to default (+-2g)
   */
  reset_cs_pin();
  motion_sensor.write((23 << 8)| 0x08);
  set_cs_pin();

  /**
   * 1. Reboot memory content in control register 5(24h)
  
  reset_cs_pin();
  motion_sensor.write((CTRL_REG5 << 8)| 0x80);
  set_cs_pin();

  reset_cs_pin();
  motion_sensor.write((CTRL_REG5 << 8)| 0x00);
  set_cs_pin();
 */
}

void read_accel_values(){
  //Read status register
  reset_cs_pin();
  uint16_t status_register = motion_sensor.read(((0x80 | 0x0F) << 8));
  set_cs_pin();
  status_register &= ~(0xFF << 8);
  //check if new value is available
  if(status_register & (1<<3)){
  //  if(status_register & (1<<7)){
      //Read X-OUT LOW AND HIGH
      reset_cs_pin();
      uint16_t x_out_low = motion_sensor.read(((0x80 | OUT_X_L) << 8));
      set_cs_pin();
      x_out_low &= ~(0xFF << 8);
      uint8_t x_axis_low = x_out_low;
      mydata = x_axis_low;
   }
  //}


}

int main(void) {
  
  system_clock.initialize();
  NOKIA.inverted_mode();

  //Initialize CHIP SELECT PIN
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
  //CONFIGURE AS GENERAL PURPOSE OUTPUT 
  CS_PORT->MODER |= GPIO_MODER_MODER3_0;
  //SET TO VERY HIGH SPEED
  CS_PORT->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR3;
  set_cs_pin();

  

  reset_cs_pin();
  motion_sensor.write((0x20 << 8)| 0x57);
  set_cs_pin();

  reset_cs_pin();
  motion_sensor.write((0x23 << 8)| 0x00);
  set_cs_pin();
 
  


  
 // initialize();

  while(1){
    reset_cs_pin();
  //Read from the WHO_AM_I register
  mydata = motion_sensor.read(((0x80 | 0x2A) << 8));
  set_cs_pin();
  mydata &=  ~(0xFF << 8);

    reset_cs_pin();
  //Read from the WHO_AM_I register
  uint16_t mydata1 = motion_sensor.read(((0x80 | 0x2B) << 8));
  set_cs_pin();
  uint16_t tempo = mydata1;
  mydata1 &=  ~(0xFF80);

  mydata |= (mydata1 << 8);

  int32_t mydata2;

  if(tempo & (1 << 7)){
    mydata = 0-mydata;
  }



    //read_accel_values();
    char received[4];
    itoa(mydata,received,10);
    NOKIA.print(received,5,2);
    for(volatile int i = 0; i < 5000000; i++){}
    NOKIA.clear();
  }
}
