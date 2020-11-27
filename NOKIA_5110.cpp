#include "NOKIA_5110.h"

namespace custom_libraries{

NOKIA_5110::NOKIA_5110(SPI_TypeDef *_SPI_,
                    GPIO_TypeDef *_GPIO,
                    uint8_t _SCK_PIN,
                    uint8_t _MOSI_PIN,
                    uint8_t _MISO_PIN,
                    uint8_t prescaler,
                    bool phase,
                    bool polarity,
                    bool _LSBFIRST,
                    GPIO_TypeDef* CS_PORT,
                    uint8_t CS_PIN,
                    GPIO_TypeDef* RST_PORT,
                    uint8_t RST_PIN,
                    GPIO_TypeDef* DC_PORT,
                    uint8_t DC_PIN):_SPI(_SPI_,
                                        _GPIO,
                                        _SCK_PIN,
                                        _MOSI_PIN,
                                        _MISO_PIN,
                                        prescaler,
                                        phase,
                                        polarity,
                                        _LSBFIRST),
                                    CS_PORT(CS_PORT),
                                    CS_PIN(CS_PIN),
                                    RST_PORT(RST_PORT),
                                    RST_PIN(RST_PIN),
                                    DC_PORT(DC_PORT),
                                    DC_PIN(DC_PIN){
                                        
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

    if(RST_PORT == GPIOA) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    if(RST_PORT == GPIOB) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    if(RST_PORT == GPIOC) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    if(RST_PORT == GPIOD) RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    if(RST_PORT == GPIOE) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
    if(RST_PORT == GPIOF) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
    if(RST_PORT == GPIOG) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
    if(RST_PORT == GPIOH) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
    if(RST_PORT == GPIOI) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOIEN;

    if(DC_PORT == GPIOA) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    if(DC_PORT == GPIOB) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    if(DC_PORT == GPIOC) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    if(DC_PORT == GPIOD) RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    if(DC_PORT == GPIOE) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
    if(DC_PORT == GPIOF) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
    if(DC_PORT == GPIOG) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
    if(DC_PORT == GPIOH) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
    if(DC_PORT == GPIOI) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOIEN;

    CS_PORT->MODER |= (1 << (CS_PIN*2));
    CS_PORT->MODER &= ~(1 << ((CS_PIN*2)+1));

    RST_PORT->MODER |= (1 << (RST_PIN*2));
    RST_PORT->MODER &= ~(1 << ((RST_PIN*2)+1));

    DC_PORT->MODER |= (1 << (DC_PIN*2));
    DC_PORT->MODER &= ~(1 << ((DC_PIN*2)+1));

    //SET CHIP SELECT, RESET AND DS PINS HIGH
    set_cs_pin();
    set_rst_pin();
    set_dc_pin();

    reset_pulse(); //SEND RESET PULSE TO THE LCD FOR INITIALIZATION

    send_command(EXTENDED_INSTRUCTION); //SET CHIP TO ACTIVE AND HORIZONTAL ADDRESSING
	send_command(MUX_48); //SET BIAS SYSTEM MUX RATIO of 1:48
	send_command(SET_VOP | 0x20);//SET LCD CONTRUST WITH OPERATING VOLTAGE

    normal_mode(); //SET THE LCD TO NORMAL MODE (WHITE BACKGROUNG, BLACK TEXT);

    clear();   

}

void NOKIA_5110::set_cs_pin(){
    CS_PORT->ODR |= (1 << CS_PIN);
}

void NOKIA_5110::reset_cs_pin(){
    CS_PORT->ODR &= ~(1 << CS_PIN);
}

void NOKIA_5110::set_rst_pin(){
    RST_PORT->ODR |= (1 << RST_PIN);
}
void NOKIA_5110::reset_rst_pin(){
    RST_PORT->ODR &= ~(1 << RST_PIN);
}
void NOKIA_5110::set_dc_pin(){
    DC_PORT->ODR |= (1 << DC_PIN);

}
void NOKIA_5110::reset_dc_pin(){
    DC_PORT->ODR &= ~(1 << DC_PIN);
}

void NOKIA_5110::reset_pulse(){
    reset_rst_pin();
    for(volatile int i = 0; i < 1000000; i++){}
    set_rst_pin();
}

//SEND A COMMAND TO THE LCD
void NOKIA_5110::send_command(char cmd){
    set_cs_pin();
    reset_dc_pin(); //SET TO COMMAND MODE
    reset_cs_pin();
    write(cmd);
    set_cs_pin();
}

//SEND DATA TO THE LCD
void NOKIA_5110::send_data(char data){
    set_cs_pin();
    set_dc_pin(); //SET TO DATA WRITE MODE
    reset_cs_pin();
    write(data);
    set_cs_pin();
}

//SEND ASCII CHARACTER TO THE LCD
void NOKIA_5110::send_char(char data){
    for(volatile int i = 0; i < 5; i++){
        send_data(ASCII[data-0x20][i]);
    }
    send_data(0x00); //SEND A  SPACE BETWEEN CHARACTERS
}
 
 //PRINT A STRING TO THE LCD
void NOKIA_5110::print(char *data, uint8_t x, uint8_t y){
    set_cursor(x,y);
    for(; *data; data++) send_char(*data);
 }

 void NOKIA_5110::clear(){
    send_command(SET_X | 0x00);
	send_command(SET_Y | 0x00);
	for(volatile int i = 0; i < 504; i++) send_data(0x00);
 }

 void NOKIA_5110::set_cursor(uint8_t x, uint8_t y){
    send_command(SET_X | x);
    send_command(SET_Y | y);
 }

 void NOKIA_5110::normal_mode(){
    send_command(BASIC_INSTRUCTION); //SET TO BASIC INSTRUCTION MODE
	send_command(NORMAL_MODE); //SET DISPLAY MODE TO NORMAL MODE
 }

 void NOKIA_5110::inverted_mode(){
    send_command(BASIC_INSTRUCTION); //SET TO BASIC INSTRUCTION MODE
	send_command(INVERTED_MODE); //SET DISPLAY MODE TO NORMAL MODE
 }


}
