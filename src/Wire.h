#pragma once

#include "Arduino.h"

#ifdef __cplusplus
extern "C" {
#endif
#include "hal_data.h"
#ifdef __cplusplus
}
#endif

class TwoWire
{
public:
    explicit TwoWire(int bus = 0);
    explicit TwoWire(const i2c_master_instance_t *instance);

    void begin();
    void begin(int sda, int scl, uint32_t frequency = 100000U);
    void setClock(uint32_t frequency);
    void beginTransmission(uint8_t address);
    size_t write(uint8_t data);
    size_t write(const uint8_t *data, size_t quantity);
    uint8_t endTransmission(bool sendStop = true);
    uint8_t requestFrom(uint8_t address, uint8_t quantity, bool sendStop = true);
    int available();
    int read();

private:
    const i2c_master_instance_t *instance_;
    uint8_t address_;
    uint8_t tx_buffer_[32];
    uint8_t rx_buffer_[32];
    size_t tx_length_;
    size_t rx_length_;
    size_t rx_index_;
    bool begun_;
};

extern TwoWire Wire;

