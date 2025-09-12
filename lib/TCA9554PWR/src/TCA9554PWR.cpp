#include "TCA9554PWR.h"

/*****************************************************  Operation register REG   ****************************************************/   
uint8_t i2c_read_exio(uint8_t REG)                             // Read the value of the TCA9554PWR register REG
{
  Wire.beginTransmission(TCA9554_ADDRESS);                
  Wire.write(REG);                                        
  uint8_t result = Wire.endTransmission();               
  if (result != 0) {                                     
    printf("The I2C transmission fails. - I2C Read EXIO\r\n");
  }
  Wire.requestFrom(TCA9554_ADDRESS, 1); 
  uint8_t bitsStatus;
  if (Wire.available()) {                  
    bitsStatus = Wire.read(); 
  }                       
  return bitsStatus;                                     
}
uint8_t i2c_write_exio(uint8_t REG,uint8_t Data)              // Write Data to the REG register of the TCA9554PWR
{
  Wire.beginTransmission(TCA9554_ADDRESS);                
  Wire.write(REG);                                        
  Wire.write(Data);                                       
  uint8_t result = Wire.endTransmission();                  

  if (result != 0) {    
    printf("I2C transmission fails. - I2C Write EXIO\r\n");
    return -1;
  }
  printf("I2C transmission successful\r\n");
  return 0;                                             
}
/********************************************************** Set EXIO mode **********************************************************/       
void mode_exio(uint8_t Pin,uint8_t State)                 // Set the mode of the TCA9554PWR Pin. The default is Output mode (output mode or input mode). State: 0= Output mode 1= input mode   
{
  uint8_t bitsStatus = i2c_read_exio(TCA9554_CONFIG_REG);      
  uint8_t Data = (0x01 << (Pin-1)) | bitsStatus;   
  uint8_t result = i2c_write_exio(TCA9554_CONFIG_REG,Data); 
  if (result != 0) { 
    printf("I/O Configuration Failure !!!\r\n");
  }
  printf("I/O Configuration successful\r\n");
}
void mode_exios(uint8_t PinState)                         // Set the mode of the 7 pins from the TCA9554PWR with PinState   
{
  uint8_t result = i2c_write_exio(TCA9554_CONFIG_REG, PinState);
  if (result != 0) {   
    printf("I/O Configuration Failure !!!\r\n");
  }
  printf("I/Os Configuration successful\r\n");
}
/********************************************************** Read EXIO status **********************************************************/       
uint8_t read_exio(uint8_t Pin)                            // Read the level of the TCA9554PWR Pin
{
  uint8_t inputBits = i2c_read_exio(TCA9554_INPUT_REG);          
  uint8_t bitStatus = (inputBits >> (Pin-1)) & 0x01; 
  return bitStatus;                                  
}
uint8_t read_exios(uint8_t REG = TCA9554_INPUT_REG)       // Read the level of all pins of TCA9554PWR, the default read input level state, want to get the current IO output state, pass the parameter TCA9554_OUTPUT_REG, such as read_exios(TCA9554_OUTPUT_REG);
{
  uint8_t inputBits = i2c_read_exio(REG);                     
  return inputBits;     
}

/********************************************************** Set the EXIO output status **********************************************************/  
void set_exio(uint8_t Pin,uint8_t State)                  // Sets the level state of the Pin without affecting the other pins
{
  uint8_t Data;
  if(State < 2 && Pin < 8 && Pin > 0){  
  uint8_t bitsStatus = read_exios(TCA9554_OUTPUT_REG);
    if(State == 1)                                     
      Data = (0x01 << (Pin-1)) | bitsStatus; 
    else if(State == 0)                  
      Data = (~(0x01 << (Pin-1))) & bitsStatus;      
  uint8_t result = i2c_write_exio(TCA9554_OUTPUT_REG,Data);  
    if (result != 0) {                         
      printf("Failed to set GPIO!!!\r\n");
    }
    printf("Set the GPIO\r\n");
  }
  else                                           
    printf("Parameter error, please enter the correct parameter!\r\n");
}
void set_exios(uint8_t PinState)                          // Set 7 pins to the PinState state such as :PinState=0x23, 0010 0011 state (the highest bit is not used)
{
  uint8_t result = i2c_write_exio(TCA9554_OUTPUT_REG,PinState); 
  if (result != 0) {                  
    printf("Failed to set GPIO!!!\r\n");
  }
}
/********************************************************** Flip EXIO state **********************************************************/  
void set_toggle(uint8_t Pin)                              // Flip the level of the TCA9554PWR Pin
{
  uint8_t bitsStatus = read_exio(Pin);                 
  set_exio(Pin,(bool)!bitsStatus); 
}
/********************************************************* TCA9554PWR Initializes the device ***********************************************************/  
void tca9554pwr_init(uint8_t PinState)                  // Set the seven pins to PinState state, for example :PinState=0x23, 0010 0011 State  (Output mode or input mode) 0= Output mode 1= Input mode. The default value is output mode
{                  
  mode_exios(PinState);      
}
