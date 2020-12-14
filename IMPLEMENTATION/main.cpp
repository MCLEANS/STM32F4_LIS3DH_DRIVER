#include "stm32f4xx.h"
#include "clockconfig.h"
#include "NOKIA_5110.h"
#include <stdlib.h>
#include "LIS3DH.h"

#define SCK_PIN 5
#define MOSI_PIN 7
#define MISO_PIN 6

#define CS_PORT GPIOE
#define CS_PIN 3

char failed_init[] = "Init Failed";
char success_init[] = "Init Success";

custom_libraries::clock_config system_clock;
custom_libraries::LIS3DH motion_sensor(SPI1,
                                        GPIOA,
                                        SCK_PIN,
                                        MOSI_PIN,
                                        MISO_PIN,
                                        CS_PORT,
                                        CS_PIN);

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

custom_libraries::Angle_values angle_values;

int main(void) {
  
  system_clock.initialize();
  bool is_verifed = motion_sensor.initialize();
  if(!is_verifed){
    NOKIA.print(failed_init,5,3);
  }
  else{
    NOKIA.print(success_init,5,3);
  }

  for(volatile int i = 0; i < 50000000; i++){}

  NOKIA.clear();

  while(1){

    angle_values = motion_sensor.read_angles();
    char received_y[4];
    char received_x[4];
    char received_z[4];  

    char zero[]= "0";

    itoa(angle_values.x_axis,received_x,10);
    itoa(angle_values.y_axis,received_y,10);

    NOKIA.print("CLK",25,0);
    NOKIA.print("A-CLK",55,0);

    NOKIA.print("X : ",0,2);
    NOKIA.print("Y : ",0,4);

    if(angle_values.x_clockwise){
      NOKIA.print(received_x,30,2);
      NOKIA.print(zero,60,2);
    }  
    if(!angle_values.x_clockwise){
      NOKIA.print(received_x,60,2);
      NOKIA.print(zero,30,2);
    }

    if(angle_values.y_clockwise){
      NOKIA.print(received_y,30,4);
      NOKIA.print(zero,60,4);
    }  
    if(!angle_values.y_clockwise){
      NOKIA.print(received_y,60,4);
      NOKIA.print(zero,30,4);
    }

  for(volatile int i = 0; i < 5000000; i++){}
    NOKIA.clear();
  }
}
