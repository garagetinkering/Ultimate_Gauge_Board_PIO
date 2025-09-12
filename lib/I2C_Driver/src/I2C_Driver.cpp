#include "I2C_Driver.h"

bool wait_for_expander(uint8_t addr, uint16_t timeout_ms = 500) {
  uint32_t start = millis();
  while (millis() - start < timeout_ms) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) return true;
    delay(10);
  }
  return false;
}

bool i2c_scan_address(uint8_t address) {
  Wire.beginTransmission(address);
  uint8_t error = Wire.endTransmission();
  return error == 0;
}

void i2c_init(void) {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  delay(300);
}

bool i2c_read(uint8_t driver_addr, uint8_t reg_addr, uint8_t *reg_data, uint32_t length) {
  Wire.beginTransmission(driver_addr);
  Wire.write(reg_addr); 
  
  if ( Wire.endTransmission(true)){
    printf("The I2C transmission fails. - I2C Read\r\n");
    return I2C_FAIL;
  }

  if (Wire.requestFrom(driver_addr, length) != length) {
    printf("I2C read length mismatch\r\n");
    return I2C_FAIL;
  }

  for (int i = 0; i < length; i++) {
    *reg_data++ = Wire.read();
  }

  return I2C_OK;
}

bool i2c_write(uint8_t driver_addr, uint8_t reg_addr, const uint8_t *reg_data, uint32_t length) {
  Wire.beginTransmission(driver_addr);
  Wire.write(reg_addr);

  for (int i = 0; i < length; i++) {
    Wire.write(*reg_data++);
  }

  if ( Wire.endTransmission(true)) {
    printf("The I2C transmission fails. - I2C Write\r\n");
    return I2C_FAIL;
  }
  return I2C_OK;
}