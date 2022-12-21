#include "MS5611_driver.h"

MS5611_driver::MS5611_driver(){
  ms5611_addr = MS5611_ADDR;
}

void MS5611_driver::initialize(){
  reset_MS5611();
}

bool MS5611_driver::testConnection(){
  return 1;
}

void MS5611_driver::reset_MS5611(){
  i2cdev3.sendCommand(ms5611_addr, 0x1E);
}

void MS5611_driver::set_OSR(uint16_t val){
  switch(val){
    case 256:
      osr = 0;
      break;
    case 512:
      osr = 2;
      break;
    case 1024:
      osr = 4;
      break;
    case 2048:
      osr = 6;
      break;
    case 4096:
      osr = 8;
      break;
    default:
      osr = 8;
  }
}

uint16_t MS5611_driver::get_OSR(){
  switch(osr){
    case 0:
      return 256;
      break;
    case 2:
      return 512;
      break;
    case 4:
      return 1024;
      break;
    case 6:
      return 2048;
      break;
    case 8:
      return 4096;
      break;
    default:
      return 1;
  }
}

void MS5611_driver::read_calibration_data(){
  uint8_t buff_temp[2];
  
  i2cdev3.read_registers(ms5611_addr, C1_addr, 2, buff_temp, 0);
  C1 = (uint16_t)buff_temp[0]<<8  | buff_temp[1];

  i2cdev3.read_registers(ms5611_addr, C2_addr, 2, buff_temp, 0);
  C2 = (uint16_t)buff_temp[0]<<8  | buff_temp[1];

  i2cdev3.read_registers(ms5611_addr, C3_addr, 2, buff_temp, 0);
  C3 = (uint16_t)buff_temp[0]<<8  | buff_temp[1];

  i2cdev3.read_registers(ms5611_addr, C4_addr, 2, buff_temp, 0);
  C4 = (uint16_t)buff_temp[0]<<8  | buff_temp[1];

  i2cdev3.read_registers(ms5611_addr, C5_addr, 2, buff_temp, 0);
  C5 = (uint16_t)buff_temp[0]<<8  | buff_temp[1];

  i2cdev3.read_registers(ms5611_addr, C6_addr, 2, buff_temp, 0);
  C6 = (uint16_t)buff_temp[0]<<8  | buff_temp[1];
}

uint16_t MS5611_driver::get_C1(){
  return C1;
}

uint16_t MS5611_driver::get_C2(){
  return C2;
}

uint16_t MS5611_driver::get_C3(){
  return C3;
}

uint16_t MS5611_driver::get_C4(){
  return C4;
}

uint16_t MS5611_driver::get_C5(){
  return C5;
}

uint16_t MS5611_driver::get_C6(){
  return C6;
}

void MS5611_driver::get_calibration_data(uint16_t* c1, uint16_t* c2, uint16_t* c3, uint16_t* c4, uint16_t* c5, uint16_t* c6){
  *c1 = C1;
  *c2 = C2;
  *c3 = C3;
  *c4 = C4;
  *c5 = C5;
  *c6 = C6;
}

// read pressure
void MS5611_driver::request_raw_pressure(){
  i2cdev3.sendCommand(ms5611_addr, CONV_D1_OSR256+osr);
}

uint32_t MS5611_driver::get_raw_pressure(){
  uint8_t buff_temp[3];
  i2cdev3.read_registers(ms5611_addr, CMD_ADC_READ, 3, buff_temp, 0);
  D1 = ((uint32_t)buff_temp[0])<<16 | ((uint32_t)buff_temp[1])<<8 | buff_temp[2];
  return D1;
}

// read temperature
void MS5611_driver::request_raw_temperature(){
  i2cdev3.sendCommand(ms5611_addr, CONV_D2_OSR256+osr);
}

uint32_t MS5611_driver::get_raw_temperature(){
  uint8_t buff_temp[3];
  i2cdev3.read_registers(ms5611_addr, CMD_ADC_READ, 3, buff_temp, 0);
  D2 = ((uint32_t)buff_temp[0])<<16 | ((uint32_t)buff_temp[1])<<8 | buff_temp[2];
  return D2;
}

void MS5611_driver::cal_temp(){
  dT = D2 - (uint32_t)C5*256;
  TEMP = 2000 + (dT*(int32_t)C6)/8388608.0;
}

int32_t MS5611_driver::get_temp(){
  return TEMP;
}

double MS5611_driver::get_temp_C(){
  return TEMP*0.01;
}

void MS5611_driver::cal_temp_compensated_pressure(){
  OFF  = (double)C2*65536.0 + ((double)C4*dT)/128.0;
  SENS = (double)C1*32768.0 + ((double)C3*dT)/256.0;
  second_order_temperature_compensation();
  P = (D1*SENS/2097152.0 - OFF)/32768.0;
}

void MS5611_driver::second_order_temperature_compensation(){
  double T2 = 0;
  double OFF2 = 0;
  double SENS2 = 0;
  
  if(TEMP<2000){
    // low tempearture
    T2 = dT*dT/2147483648.0;
    OFF2 = 5.0*(TEMP-2000.0)*(TEMP-2000.0)/2.0;
    SENS2 = 5.0*(TEMP-2000.0)*(TEMP-2000.0)/4.0;
    
    if(TEMP<-1500){
      OFF2 = OFF2 + 7.0*(TEMP+1500.0)*(TEMP+1500.0);
      SENS2 = SENS2 + 11.0*((TEMP+1500.0)*(TEMP+1500.0))/2.0;
    }
  }
  else{
    // high temperature
    T2 = 0;
    OFF2 = 0;
    SENS2 = 0;
  }
  
  TEMP = TEMP - T2;
  OFF = OFF - OFF2;
  SENS = SENS - SENS2;
}

double MS5611_driver::get_pressure(){
  return P;
}

double MS5611_driver::get_pressure_mbar(){
  return P*0.01;
}
