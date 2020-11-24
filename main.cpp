#include "stm32f4xx.h"
#include "clockconfig.h"
#include "SPI.h"

#define SCK_PIN 5
#define MOSI_PIN 7
#define MISO_PIN 6

#define CS_PORT GPIOE
#define CS_PIN 3



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

int main(void) {
  
  system_clock.initialize();


  while(1){

  }
}
