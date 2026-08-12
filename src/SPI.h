#pragma once

#include "Arduino.h"

#define SPI_MODE0 0
#define SPI_MODE1 1
#define SPI_MODE2 2
#define SPI_MODE3 3

class SPISettings
{
public:
    SPISettings(uint32_t clock = 1000000U, uint8_t bitOrder = MSBFIRST, uint8_t dataMode = SPI_MODE0)
        : clock_(clock), bit_order_(bitOrder), data_mode_(dataMode)
    {
    }

    uint32_t clock_;
    uint8_t bit_order_;
    uint8_t data_mode_;
};

class SPIClass
{
public:
    void begin() {}
    void end() {}
    void beginTransaction(SPISettings settings) { (void) settings; }
    uint16_t transfer16(uint16_t data) { return data; }
    uint8_t transfer(uint8_t data) { return data; }
    void endTransaction() {}
};

extern SPIClass SPI;
