#ifndef _LIS3DH_H
#define _LIS3DH_H

#include "SPI_16bit.h"

/**
 * LIS3DH REGISTERS
 */
#define WHO_AM_I 0x0F
#define CTRL_REG0 0x1E
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24
#define STATUS_REG 0x27
#define OUT_X_L 0x28
#define OUT_X_H 0x29
#define OUT_Y_L 0x2A
#define OUT_Y_H 0x2B
#define OUT_Z_L 0X2C
#define OUT_Z_H 0X2D

#define LIS3DH_ID 63

namespace custom_libraries{
    class LIS3DH : public _SPI_16{
    private:
    private:
    public:
    public:

    };
}





#endif //_LIS3DH_H