#ifndef _SPI_H
#define _SPI_H

#include "stm32f4xx.h"

namespace custom_libraries{

    class _SPI{
        private:
            SPI_TypeDef *SPI_;
            GPIO_TypeDef *GPIO;
            uint8_t SCK_PIN;
            uint8_t MOSI_PIN;
            uint8_t MISO_PIN;
            uint8_t SPI_PRESCALER;
            bool CPHA;
            bool CPOL;
            bool LSBFIRST;
            
        private:
        public:
        public:
            _SPI(SPI_TypeDef *SPI,
                GPIO_TypeDef *GPIO,
                uint8_t SCK_PIN,
                uint8_t MOSI_PIN,
                uint8_t MISO_PIN,
                uint8_t SPI_PRESCALER,
                bool CPHA,
                bool CPOL,
                bool LSBFIRST);
            void write(char data);
            char read(char junk);
            ~_SPI();

    };
}



#endif //_SPI_H