#include "LIS3DH.h"

namespace custom_libraries{

LIS3DH::LIS3DH(GPIO_TypeDef *CS_PORT,
                uint8_t CS_PIN,
                SPI_TypeDef *SPI,
                GPIO_TypeDef *GPIO,
                uint8_t SCK_PIN,
                uint8_t MOSI_PIN,
                uint8_t MISO_PIN):CS_PORT(CS_PORT),
                                    CS_PIN(CS_PIN),
                                    _SPI_16(SPI,
                                            GPIO,
                                            SCK_PIN,
                                            MOSI_PIN,
                                            MISO_PIN,
                                            SPI_PRESCALER,
                                            true,
                                            true,
                                            false){
    //SET RESET, CHIP SELECT AND DC PIN DIRECTION (OUTPUT)                 
    if(CS_PORT == GPIOA) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    if(CS_PORT == GPIOB) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    if(CS_PORT == GPIOC) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    if(CS_PORT == GPIOD) RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    if(CS_PORT == GPIOE) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
    if(CS_PORT == GPIOF) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
    if(CS_PORT == GPIOG) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
    if(CS_PORT == GPIOH) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
    if(CS_PORT == GPIOI) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOIEN;
    //CONFIGURE AS GENERAL PURPOSE OUTPUT 
    this->CS_PORT->MODER |= (1 << (this->CS_PIN*2));
    this->CS_PORT->MODER &= ~(1 << ((this->CS_PIN*2)+1));

    /**
   * Configure Control register 0
   * 1. Initialize the register as indicated in datasheet (This is needed for the sensor to function optimally)
   */
    reset_cs_pin();
    write((CTRL_REG0 << 8)| 0x10);
    set_cs_pin();

    /**
     * configure control register 1
     * 1. Set Normal mode. 
     * 2. Set 100Hz sampling rate
     */
    reset_cs_pin();
    write((CTRL_REG1 << 8)| 0x57);
    set_cs_pin();

    /**
     * Configure Control Register 4
     * 1. Clear HR bit to set Normal Mode
     * 2. Spi Interface mode selection (Set to 4-wire).
     * 3. Full scale selected to +-2g
     */
    reset_cs_pin();
    write((CTRL_REG4 << 8)| 0x00);
    set_cs_pin();

}

void LIS3DH::set_cs_pin(){
  this->CS_PORT->ODR |= (1 << this->CS_PIN);
}

void LIS3DH::reset_cs_pin(){
  this->CS_PORT->ODR &= ~(1 << this->CS_PIN);
}

bool LIS3DH::initialize(){
    //Read from the WHO_AM_I register
    reset_cs_pin();
    uint16_t whoami = read(((0x80 | WHO_AM_I) << 8));
    set_cs_pin();
    whoami &=  ~(0xFF << 8);
    if(whoami == LIS3DH_ID) return true;

    return false;
}

}
                                                