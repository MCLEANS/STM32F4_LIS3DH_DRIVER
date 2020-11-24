#include "stm32f4xx.h"
#include "clockconfig.h"
#include "SPI.h"

#define SCK_PIN 5
#define MOSI_PIN 7
#define MISO_PIN 6

#define CS_PORT GPIOD
#define CS_PIN 13

//MOTION SENSOR COMMANDS
#define WHO_AM_I 0x0F
#define CTRL_REG1 0x20
#define ACTIVE_MODE CTRL_REG1 | 0b01000000
#define ENABLE_Z_AXIS CTRL_REG1 | 0b00000100
#define ENABLE_Y_AXIS CTRL_REG1 | 0b00000010
#define ENABLE_X_AXIS CTRL_REG1 | 0b00000001
#define CTRL_REG2 0x21
#define SET_REBOOT_MEMORY CTRL_REG2 | 0b10000000
#define RESET_REBOOT_MEMORY (CTRL_REG2 & ~(0b01000000))
#define STATUS_REG 0x27
#define OUT_X 0x29
#define OUT_Y 0x2B
#define OUT_Z 0X2Z
#define READ 0b10000000
#define WRITE 0b00000000


custom_libraries::clock_config system_clock;
custom_libraries::_SPI motion_sensor(SPI1,
                                    GPIOA,
                                    SCK_PIN,
                                    MOSI_PIN,
                                    MISO_PIN,
                                    16,
                                    false,
                                    true,
                                    false);
void set_cs_pin(){
  CS_PORT->ODR |= (1 << CS_PIN);
}

void reset_cs_pin(){
  CS_PORT->ODR &= ~(1 << CS_PIN);
}

int main(void) {
  
  system_clock.initialize();
  
  //Initialize CHIP SELECT PIN
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
  //CONFIGURE AS GENERAL PURPOSE OUTPUT 
  CS_PORT->MODER |= GPIO_MODER_MODER13_0;
  //SET TO VERY HIGH SPEED
  //CS_PORT->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR3;
  set_cs_pin();

  reset_cs_pin();
  motion_sensor.read(READ|WHO_AM_I);
  set_cs_pin();





  while(1){

  }
}
