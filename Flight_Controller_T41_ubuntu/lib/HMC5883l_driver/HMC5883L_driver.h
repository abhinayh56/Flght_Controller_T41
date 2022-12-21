#ifndef HMC5883L_DRIVER_H
#define HMC5883L_DRIVER_H

#include "I2Cdev1.h"

// device address
#define HMC5883L_ARRDRESS 0x1E

// registers address
#define REG_CONFIG_A  0x00
#define REG_CONFIG_B  0x01
#define REG_MODE      0x02
#define REG_DATAX_MSB 0x03
#define REG_DATAX_LSB 0x04
#define REG_DATAZ_MSB 0x05
#define REG_DATAZ_LSB 0x06
#define REG_DATAY_MSB 0x07
#define REG_DATAY_LSB 0x08
#define REG_STATUS    0x09
#define REG_ID_A      0x0A
#define REG_ID_B      0x0B
#define REG_ID_C      0x0C

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

class HMC5883L_driver{
  public:
    HMC5883L_driver();

    void initialize();
    
    bool testConnection();

    // Register Name: REG_CONFIG_A (Configuration register A)
    // location information of MA function
    #define MA_LCN_1      BITM_56 // data bit location mask indicated by 1
    #define MA_LCN_0     ~BITM_56 // data bit location mask indicated by 0
    #define MA_LEN        TWO
    #define MA_LCN_START  FIVE
    #define MA_LCN_END    SIX

    // values to be set in MA bit location
    #define MA_0 0 // average of 1 samples (i.e. no averaging)
    #define MA_1 1 // average of 2 samples
    #define MA_2 2 // average of 4 samples
    #define MA_3 3 // average of 8 samples

    // number of samples averaged before output
    uint8_t get_MA();
    void set_MA(uint8_t val);

    // location information of DO function
    #define DO_LCN_1      BITM_24 // data bit location mask indicated by 1
    #define DO_LCN_0     ~BITM_24 // data bit location mask indicated by 0
    #define DO_LEN        THREE
    #define DO_LCN_START  TWO
    #define DO_LCN_END    FOUR

    // values to be set in DO bit location
    #define DO_0 0 // 0.75 Hz
    #define DO_1 1 // 1.5  Hz
    #define DO_2 2 // 3    Hz
    #define DO_3 3 // 7.5  Hz
    #define DO_4 4 // 15   Hz (default)
    #define DO_5 5 // 30   Hz
    #define DO_6 6 // 75   Hz
    #define DO_7 7 // Reserved

    // Data output rate
    uint8_t get_DO();
    void set_DO(uint8_t val);
    
    // location information of MS function
    #define MS_LCN_1      BITM_01 // data bit location mask indicated by 1
    #define MS_LCN_0     ~BITM_01 // data bit location mask indicated by 0
    #define MS_LEN        TWO
    #define MS_LCN_START  ZERO
    #define MS_LCN_END    ONE

    // values to be set in DO bit location
    #define MS_0 0
    #define MS_1 1
    #define MS_2 2
    #define MS_3 3 // Reserved
    
    // Define measurement flow to incorporate an applied bias into the measurement
    uint8_t get_MS();
    void set_MS(uint8_t val);

    // Register Name: REG_CONFIG_B (Configuration register A)
    // location information of GN function
    #define GN_LCN_1      BITM_57 // data bit location mask indicated by 1
    #define GN_LCN_0     ~BITM_57 // data bit location mask indicated by 0
    #define GN_LEN        THREE
    #define GN_LCN_START  FIVE
    #define GN_LCN_END    SEVEN

    // values to be set in GN bit location
    #define GN_0 0 // 1370 LSB/Gauss, recommended for +-0.88 Ga
    #define GN_1 1 // 1090 LSB/Gauss, recommended for +-0.88 Ga (default)
    #define GN_2 2 // 820 LSB/Gauss, recommended for +-0.88 Ga
    #define GN_3 3 // 660 LSB/Gauss, recommended for +-0.88 Ga
    #define GN_4 4 // 440 LSB/Gauss, recommended for +-0.88 Ga
    #define GN_5 5 // 390 LSB/Gauss, recommended for +-0.88 Ga
    #define GN_6 6 // 330 LSB/Gauss, recommended for +-0.88 Ga
    #define GN_7 7 // 230 LSB/Gauss, recommended for +-0.88 Ga

    // functions for gain feature for device
    uint8_t get_GN();
    void set_GN(uint8_t val);
    
/*  Register Name: REG_MODE (Mode Register)
    Register Address (Hex / Decimal): 0x02 / 2
    Type : Read / Write
    Description: This register is used to select the operating mode of the device.
    Mode register default is 0x01.
    
    Parameters:
        HS: Set this pin to enable High Speed I2C, 3400kHz.
        MD: Mode Select Bits. These bits select the operation mode of the device.
*/
    // location information of MD function
    #define MD_LCN_1      BITM_01 // data bit location mask indicated by 1
    #define MD_LCN_0     ~BITM_01 // data bit location mask indicated by 0
    #define MD_LEN        TWO
    #define MD_LCN_START  ZERO
    #define MD_LCN_END    ONE

    // values to be set in MD bit location
    #define MD_0 0 // Continuous-Measurement Mode
    #define MD_1 1 // Single-Measurement Mode (Default)
    #define MD_2 2 // Idle Mode. Device is placed in idle mode.
    #define MD_3 3 // Idle Mode. Device is placed in idle mode.

    // functions for gain feature for device
    uint8_t get_MD();
    void set_MD(uint8_t val);
    
    // Reading sensor data register 
    uint8_t get_MAG_XOUT_H();
    uint8_t get_MAG_XOUT_L();
    uint8_t get_MAG_YOUT_H();
    uint8_t get_MAG_YOUT_L();
    uint8_t get_MAG_ZOUT_H();
    uint8_t get_MAG_ZOUT_L();
    int16_t get_MAG_XOUT();
    int16_t get_MAG_YOUT();
    int16_t get_MAG_ZOUT();
    void get_MAG_OUT(int16_t* MAG_X, int16_t* MAG_Y, int16_t* MAG_Z);
    
  private:
    I2Cdev1 i2cdev2;

    uint8_t hmc5883l_addr;
    uint8_t register_val;
    uint8_t buff[6];
    /*
      buff[0]  = MAG_XOUT_H
      buff[1]  = MAG_XOUT_L
      buff[2]  = MAG_YOUT_H
      buff[3]  = MAG_YOUT_L
      buff[4]  = MAG_ZOUT_H
      buff[5]  = MAG_ZOUT_L
    */
    // index number of register values in buff array
    #define buff_MAG_XOUT_H 0
    #define buff_MAG_XOUT_L 1
    #define buff_MAG_ZOUT_H 2
    #define buff_MAG_ZOUT_L 3
    #define buff_MAG_YOUT_H 4
    #define buff_MAG_YOUT_L 5
};

#endif
