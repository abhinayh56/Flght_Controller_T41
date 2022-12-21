#include "I2Cdev1.h"
#include "Wire.h"

I2Cdev1::I2Cdev1(){
  //Wire1.begin();
  //TWBR = 12;
}

void I2Cdev1::initialize(){
  Wire1.begin();
  //TWBR = 12;
  Wire1.setClock(400000);
}

uint8_t I2Cdev1::read_register(uint8_t i2c_dev1_addr, uint8_t reg_addr){
  Wire1.beginTransmission(i2c_dev1_addr);
  Wire1.write(reg_addr);
  Wire1.endTransmission(false);
  Wire1.requestFrom(i2c_dev1_addr,1,true);
  while(Wire1.available() < 1);
  return Wire1.read();
}

void I2Cdev1::read_registers(uint8_t i2c_dev1_addr, uint8_t reg_addr, uint8_t n_bytes, uint8_t buff_[], uint8_t buff_i){
  Wire1.beginTransmission(i2c_dev1_addr);
  Wire1.write(reg_addr);
  Wire1.endTransmission();
  Wire1.requestFrom(i2c_dev1_addr,n_bytes);
  while(Wire1.available() < n_bytes);
  for(int i=buff_i;i<buff_i+n_bytes;i++){
    buff_[i] = Wire1.read();
  }
}

void I2Cdev1::write_register(uint8_t i2c_dev1_addr, uint8_t reg_addr, uint8_t register_val){
  Wire1.beginTransmission(i2c_dev1_addr);
  Wire1.write(reg_addr);
  Wire1.write(register_val);
  Wire1.endTransmission();
}

void I2Cdev1::sendCommand(uint8_t i2c_dev1_addr, uint8_t cmd)
{
  Wire1.beginTransmission(i2c_dev1_addr);
  Wire1.write(cmd);
  Wire1.endTransmission();
}
