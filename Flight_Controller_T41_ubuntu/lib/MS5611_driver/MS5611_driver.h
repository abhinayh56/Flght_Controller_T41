#ifndef MS5611_DRIVER_H
#define MS5611_DRIVER_H

#include "I2Cdev1.h"

// make PS pin high for I2C mode 
// and low for SPI mode default is I2C

// device address
#define MS5611_ADDR 0x77

#define MS5611_RESET 0x1E

// Pressure conversion
#define CONV_D1_OSR256  0x40
#define CONV_D1_OSR512  0x42
#define CONV_D1_OSR1024 0x44
#define CONV_D1_OSR2048 0x46
#define CONV_D1_OSR4096 0x48

// Temperature conversion
#define CONV_D2_OSR256  0x50
#define CONV_D2_OSR512  0x52
#define CONV_D2_OSR1024 0x54
#define CONV_D2_OSR2048 0x56
#define CONV_D2_OSR4096 0x58

#define CMD_ADC_READ    0x00

// PROM Read 0xA0 to 0xAE
#define C0_addr 0xA0
#define C1_addr 0xA2
#define C2_addr 0xA4
#define C3_addr 0xA6
#define C4_addr 0xA8
#define C5_addr 0xAA
#define C6_addr 0xAC
#define C7_addr 0xAE

/*  8-bit mask for data access and manipulation of registers
    mask for data bit location for a functionality
    it contains locations of bits inside a register for particular
    feature / function / behaviour / mode etc.
    A functionality can be activated / deactivated using a register
    and some bits of registers needs to be set (i.e. assign 1) or
    reset / clear (i.e. assign 0) depending upon the guidelines of
    the datasheet. Number of bits to be manipulated may be 0, 1, ... , 8.
    So the length of bits can be from zero to 8.
    The below variables contains mask to extract those bits for particular
    functionality
*/
// mask / address of data of one unit length
#define BITM_0 0x01 //0b00000001 (1)
#define BITM_1 0x02 //0b00000010 (2)
#define BITM_2 0x04 //0b00000100 (4)
#define BITM_3 0x08 //0b00001000 (8)
#define BITM_4 0x10 //0b00010000 (16)
#define BITM_5 0x20 //0b00100000 (32)
#define BITM_6 0x40 //0b01000000 (64)
#define BITM_7 0x80 //0b10000000 (128)

// mask / address of data of two unit length
#define BITM_01 (BITM_0 | BITM_1)
#define BITM_12 (BITM_1 | BITM_2)
#define BITM_23 (BITM_2 | BITM_3)
#define BITM_34 (BITM_3 | BITM_4)
#define BITM_45 (BITM_4 | BITM_5)
#define BITM_56 (BITM_5 | BITM_6)
#define BITM_67 (BITM_6 | BITM_7)

// mask / address of data of three unit length
#define BITM_02 (BITM_01 | BITM_2)
#define BITM_13 (BITM_12 | BITM_3)
#define BITM_24 (BITM_23 | BITM_4)
#define BITM_35 (BITM_34 | BITM_5)
#define BITM_46 (BITM_45 | BITM_6)
#define BITM_57 (BITM_56 | BITM_7)

// mask / address of data of fours unit length
#define BITM_03 (BITM_02 | BITM_3)
#define BITM_14 (BITM_13 | BITM_4)
#define BITM_25 (BITM_24 | BITM_5)
#define BITM_36 (BITM_35 | BITM_6)
#define BITM_47 (BITM_46 | BITM_7)

// mask / address of data of five unit length
#define BITM_04 (BITM_03 | BITM_4)
#define BITM_15 (BITM_14 | BITM_5)
#define BITM_26 (BITM_25 | BITM_6)
#define BITM_37 (BITM_36 | BITM_7)

// mask / address of data of six unit length
#define BITM_05 (BITM_04 | BITM_5)
#define BITM_16 (BITM_15 | BITM_6)
#define BITM_27 (BITM_26 | BITM_7)

// mask / address of data of seven unit length
#define BITM_06 (BITM_05 | BITM_6)
#define BITM_17 (BITM_16 | BITM_7)

// mask / address of data of eight unit length
#define BITM_07 (BITM_06 | BITM_7)

// values at bit location
#define SET_0 0x00
#define SET_1 0x01

// number of bits (data length) (bytes) OR location of bit in register
#define ZERO  0x00
#define ONE   0x01
#define TWO   0x02
#define THREE 0x03
#define FOUR  0x04
#define FIVE  0x05
#define SIX   0x06
#define SEVEN 0x07
#define EIGHT 0x08

class MS5611_driver{
  public:
    MS5611_driver();
    void initialize();
    bool testConnection();
    void reset_MS5611();
    void set_OSR(uint16_t val);
    uint16_t get_OSR();
    void read_calibration_data();
    uint16_t get_C1();
    uint16_t get_C2();
    uint16_t get_C3();
    uint16_t get_C4();
    uint16_t get_C5();
    uint16_t get_C6();
    void get_calibration_data(uint16_t* c1, uint16_t* c2, uint16_t* c3, uint16_t* c4, uint16_t* c5, uint16_t* c6);
    // read pressure
    void request_raw_pressure();
    uint32_t get_raw_pressure();
    // read temperature
    void request_raw_temperature();
    uint32_t get_raw_temperature();

    void cal_temp();
    void cal_temp_compensated_pressure();
    void second_order_temperature_compensation();

    int32_t get_temp();
    double get_temp_C();
    double get_pressure();
    double get_pressure_mbar();
    
  private:
    I2Cdev1 i2cdev3;

    uint8_t ms5611_addr;
    uint8_t osr = 0;
    
    // calibration values
    uint16_t C1;
    uint16_t C2;
    uint16_t C3;
    uint16_t C4;
    uint16_t C5;
    uint16_t C6;
    
    uint32_t D1;
    uint32_t D2;
    
    double dT;
    double TEMP;
    
    double OFF;
    double SENS;
    double P;
};

#endif
