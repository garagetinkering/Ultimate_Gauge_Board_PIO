#pragma once
#include <Wire.h> 

#define I2C_SCL_PIN       7
#define I2C_SDA_PIN       15

#define I2C_OK            true
#define I2C_FAIL          false

bool i2c_scan_address(uint8_t address);
void i2c_init(void);

bool i2c_read(uint8_t driver_addr, uint8_t reg_addr, uint8_t *reg_data, uint32_t length);
bool i2c_write(uint8_t driver_addr, uint8_t reg_addr, const uint8_t *reg_data, uint32_t length);