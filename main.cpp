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
#define ACTIVE_MODE CTRL_REG1 | 0b01000000
#define ENABLE_Z_AXIS CTRL_REG1 | 0b00000100
#define ENABLE_Y_AXIS CTRL_REG1 | 0b00000010
#define ENABLE_X_AXIS CTRL_REG1 | 0b00000001
#define CTRL_REG2 0x21
#define SET_REBOOT_MEMORY CTRL_REG2 | 0b01000000
#define RESET_REBOOT_MEMORY (CTRL_REG2 & ~(0b01000000))
#define STATUS_REG 0x27
#define OUT_X 0x29
#define OUT_Y 0x2B
#define OUT_Z 0X2Z
#define READ 0b10000000
#define WRITE 0b00000000


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
  motion_sensor.write((0x20 << 8)| 0x5F);
  set_cs_pin();

  reset_cs_pin();
  motion_sensor.write((0x23 << 8)| 0x08);
  set_cs_pin();
 
  reset_cs_pin();
  uint16_t data = get_device_ID();
  set_cs_pin();

  char received[4];
  itoa(data,received,10);
  NOKIA.print(received,5,2);


  while(1){

  }
}
