#include "HMC5883L_driver.h"

HMC5883L_driver::HMC5883L_driver(){
  hmc5883l_addr = HMC5883L_ARRDRESS;
}

void HMC5883L_driver::initialize(){
  // set zero at 7th bit of REG_CONFIG_A (written in datasheet)
  register_val = i2cdev2.read_register(hmc5883l_addr, REG_CONFIG_A);
  register_val = (register_val & (~BITM_7)) | (SET_0 << SEVEN);
  i2cdev2.write_register(hmc5883l_addr, REG_CONFIG_A, register_val);
  
  // set zero at 0 to 4th bit of REG_CONFIG_B (written in datasheet)
  register_val = i2cdev2.read_register(hmc5883l_addr, REG_CONFIG_B);
  register_val = (register_val & (~BITM_04)) | (SET_0 << ZERO);
  i2cdev2.write_register(hmc5883l_addr, REG_CONFIG_B, register_val);
}

bool HMC5883L_driver::testConnection(){
  return 1;
}

// Register Name: REG_CONFIG_A (Configuration register A)
// number of samples averaged before output
uint8_t HMC5883L_driver::get_MA(){
  register_val = i2cdev2.read_register(hmc5883l_addr, REG_CONFIG_A);
  return (register_val & MA_LCN_1) >> MA_LCN_START;
}

void HMC5883L_driver::set_MA(uint8_t val){
  register_val = i2cdev2.read_register(hmc5883l_addr, REG_CONFIG_A);
  register_val = (register_val & MA_LCN_0) | (val << MA_LCN_START);
  i2cdev2.write_register(hmc5883l_addr, REG_CONFIG_A, register_val);
}

// location information of DO function
// Data output rate
uint8_t HMC5883L_driver::get_DO(){
  register_val = i2cdev2.read_register(hmc5883l_addr, REG_CONFIG_A);
  return (register_val & DO_LCN_1) >> DO_LCN_START;
}

void HMC5883L_driver::set_DO(uint8_t val){
  register_val = i2cdev2.read_register(hmc5883l_addr, REG_CONFIG_A);
  register_val = (register_val & DO_LCN_0) | (val << DO_LCN_START);
  i2cdev2.write_register(hmc5883l_addr, REG_CONFIG_A, register_val);
}

// Define measurement flow to incorporate an applied bias into the measurement
uint8_t HMC5883L_driver::get_MS(){
  register_val = i2cdev2.read_register(hmc5883l_addr, REG_CONFIG_A);
  return (register_val & MS_LCN_1) >> MS_LCN_START;
}

void HMC5883L_driver::set_MS(uint8_t val){
  register_val = i2cdev2.read_register(hmc5883l_addr, REG_CONFIG_A);
  register_val = (register_val & MS_LCN_0) | (val << MS_LCN_START);
  i2cdev2.write_register(hmc5883l_addr, REG_CONFIG_A, register_val);
}

// Register Name: REG_CONFIG_B (Configuration register A)
// functions for gain feature for device
uint8_t HMC5883L_driver::get_GN(){
  register_val = i2cdev2.read_register(hmc5883l_addr, REG_CONFIG_B);
  return (register_val & GN_LCN_1) >> GN_LCN_START;
}

void HMC5883L_driver::set_GN(uint8_t val){
  register_val = i2cdev2.read_register(hmc5883l_addr, REG_CONFIG_B);
  register_val = (register_val & GN_LCN_0) | (val << GN_LCN_START);
  i2cdev2.write_register(hmc5883l_addr, REG_CONFIG_B, register_val);
}

// Register Name: REG_MODE (Mode Register)
uint8_t HMC5883L_driver::get_MD(){
  register_val = i2cdev2.read_register(hmc5883l_addr, REG_MODE);
  return (register_val & MD_LCN_1) >> MD_LCN_START;
}

void HMC5883L_driver::set_MD(uint8_t val){
  register_val = i2cdev2.read_register(hmc5883l_addr, REG_MODE);
  register_val = (register_val & MD_LCN_0) | (val << MD_LCN_START);
  i2cdev2.write_register(hmc5883l_addr, REG_MODE, register_val);
}

// Reading sensor data register 
uint8_t HMC5883L_driver::get_MAG_XOUT_H(){
  return i2cdev2.read_register(hmc5883l_addr, REG_DATAX_MSB);
}

uint8_t HMC5883L_driver::get_MAG_XOUT_L(){
  return i2cdev2.read_register(hmc5883l_addr, REG_DATAX_LSB);
}

uint8_t HMC5883L_driver::get_MAG_YOUT_H(){
  return i2cdev2.read_register(hmc5883l_addr, REG_DATAY_MSB);
}

uint8_t HMC5883L_driver::get_MAG_YOUT_L(){
  return i2cdev2.read_register(hmc5883l_addr, REG_DATAY_LSB);
}

uint8_t HMC5883L_driver::get_MAG_ZOUT_H(){
  return i2cdev2.read_register(hmc5883l_addr, REG_DATAZ_MSB);
}

uint8_t HMC5883L_driver::get_MAG_ZOUT_L(){
  return HMC5883L_driver::i2cdev2.read_register(hmc5883l_addr, REG_DATAZ_LSB);
}

int16_t HMC5883L_driver::get_MAG_XOUT(){
  i2cdev2.read_registers(hmc5883l_addr, REG_DATAX_MSB, 2, buff, buff_MAG_XOUT_H);
  return (((int16_t)buff[buff_MAG_XOUT_H]) << 8) | buff[buff_MAG_XOUT_L];
}

int16_t HMC5883L_driver::get_MAG_YOUT(){
  i2cdev2.read_registers(hmc5883l_addr, REG_DATAY_MSB, 2, buff, buff_MAG_YOUT_H);
  return (((int16_t)buff[buff_MAG_YOUT_H]) << 8) | buff[buff_MAG_YOUT_L];
}

int16_t HMC5883L_driver::get_MAG_ZOUT(){
  i2cdev2.read_registers(hmc5883l_addr, REG_DATAZ_MSB, 2, buff, buff_MAG_ZOUT_H);
  return (((int16_t)buff[buff_MAG_ZOUT_H]) << 8) | buff[buff_MAG_ZOUT_L];
}

void HMC5883L_driver::get_MAG_OUT(int16_t* MAG_X, int16_t* MAG_Y, int16_t* MAG_Z){
  i2cdev2.read_registers(hmc5883l_addr, REG_DATAX_MSB, 6, buff, buff_MAG_XOUT_H);
  *MAG_X = (((int16_t)buff[buff_MAG_XOUT_H]) << 8) | buff[buff_MAG_XOUT_L];
  *MAG_Z = (((int16_t)buff[buff_MAG_ZOUT_H]) << 8) | buff[buff_MAG_ZOUT_L];
  *MAG_Y = (((int16_t)buff[buff_MAG_YOUT_H]) << 8) | buff[buff_MAG_YOUT_L];
}
