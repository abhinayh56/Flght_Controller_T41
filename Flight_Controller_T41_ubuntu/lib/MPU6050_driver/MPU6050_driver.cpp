#include "MPU6050_driver.h"

MPU6050_driver::MPU6050_driver(){
  mpu6050_addr = MPU6050_ADDR_DEFAULT;
}

MPU6050_driver::MPU6050_driver(uint8_t address){
  mpu6050_addr = address;
}

void MPU6050_driver::initialize(){
  set_SLEEP(0);
}

bool MPU6050_driver::testConnection(){
  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////
/* Register Name: SMPRT_DIV (Sample Rate Divider)
   Register Address (Hex / Decimal): 0x19 / 25
   Type : Read / Write   
   Description: This register specifies the divider from the gyroscope output
   rate used to generate the Sample Rate for the MPU-60X0. The sensor register
   output, FIFO output, and DMP sampling are all based on the Sample Rate.
   The Sample Rate is generated by dividing the gyroscope output rate by SMPLRT_DIV.
        Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
   where Gyroscope Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or 7),
   and 1kHz when the DLPF is enabled (see Register 26).
   Note: The accelerometer output rate is 1kHz. This means that for a Sample Rate 
   greater than 1kHz, the same accelerometer sample may be output to the FIFO, DMP,
   and sensor registers more than once.
   
   Parameters:
       SMPLRT_DIV 8-bit unsigned value. The Sample Rate is determined by dividing
       the gyroscope output rate by this value.
*/

// configuring SMPLRT_DIV pin sampling
uint8_t MPU6050_driver::get_SMPLRT_DIV(){
  register_val = i2cdev1.read_register(mpu6050_addr, SMPRT_DIV);
  return (register_val & SMPLRT_DIV_LCN_1) >> SMPLRT_DIV_LCN_START;
}

void MPU6050_driver::set_SMPLRT_DIV(uint8_t val){
  register_val = i2cdev1.read_register(mpu6050_addr, SMPRT_DIV);
  register_val = (register_val & SMPLRT_DIV_LCN_0) | (val << SMPLRT_DIV_LCN_START);
  i2cdev1.write_register(mpu6050_addr, SMPRT_DIV, register_val);
}

/* Register Name: CONFIG
   Register Address (Hex / Decimal): 0x1A / 26
   Type : Read / Write   
   Description: It is a "configureation" register. It is used to configure
   external Frame Synchronization (FSYNC) pin sampling and the Digital Low
   Pass Filter (DLPF) setting for both the gyroscopes and accelerometers.
   The DLPF is configured by DLPF_CFG.
   The accelerometer and gyroscope are filtered according to the value of DLPF_CFG.
   
   Parameters:
       EXT_SYNC_SET: 3-bit unsigned value. Configures the FSYNC pin sampling.
       DLPF_CFG:     3-bit unsigned value. Configures the DLPF setting.
*/

// configuring FSYNC pin sampling
uint8_t MPU6050_driver::get_ext_FSYNC(){
  register_val = i2cdev1.read_register(mpu6050_addr, CONFIG);
  return (register_val & EXT_SYNC_SET_LCN_1) >> EXT_SYNC_SET_LCN_START;
}

void MPU6050_driver::set_ext_FSYNC(uint8_t val){
  register_val = i2cdev1.read_register(mpu6050_addr, CONFIG);
  register_val = (register_val & EXT_SYNC_SET_LCN_0) | (val << EXT_SYNC_SET_LCN_START);
  i2cdev1.write_register(mpu6050_addr, CONFIG, register_val);
}

// configuring DLPF setting (both accelerometer and gyroscope is changed)
uint8_t MPU6050_driver::get_DLPF_CFG(){
  register_val = i2cdev1.read_register(mpu6050_addr, CONFIG);
  return (register_val & DLPF_CFG_LCN_1) >> DLPF_CFG_LCN_START;
}

void MPU6050_driver::set_DLPF_CFG(uint8_t val){
  register_val = i2cdev1.read_register(mpu6050_addr, CONFIG);
  register_val = (register_val & DLPF_CFG_LCN_0) | (val << DLPF_CFG_LCN_START);
  i2cdev1.write_register(mpu6050_addr, CONFIG, register_val);
}

/* Register Name: GYRO_CONFIG (Gyroscope Configuration)
   Register Address (Hex / Decimal): 0x1B / 27
   Type : Read / Write   
   Description: This register is used to trigger gyroscope self-test and configure
   the gyroscopes’ full scale range.
   
   Parameters:
       XG_ST:  Setting this bit causes the X axis gyroscope to perform self test.
       YG_ST:  Setting this bit causes the Y axis gyroscope to perform self test.
       ZG_ST:  Setting this bit causes the Z axis gyroscope to perform self test.
       FS_SEL: 2-bit unsigned value. Selects the full scale range of gyroscopes.
*/

// configuring XG_ST
uint8_t MPU6050_driver::get_XG_ST(){
  register_val = i2cdev1.read_register(mpu6050_addr, GYRO_CONFIG);
  return (register_val & XG_ST_LCN_1) >> XG_ST_LCN_START;
}

void MPU6050_driver::set_XG_ST(uint8_t val){
  register_val = i2cdev1.read_register(mpu6050_addr, GYRO_CONFIG);
  register_val = (register_val & XG_ST_LCN_0) | (val << XG_ST_LCN_START);
  i2cdev1.write_register(mpu6050_addr, GYRO_CONFIG, register_val);
}

// configuring YG_ST
uint8_t MPU6050_driver::get_YG_ST(){
  register_val = i2cdev1.read_register(mpu6050_addr, GYRO_CONFIG);
  return (register_val & YG_ST_LCN_1) >> YG_ST_LCN_START;
}

void MPU6050_driver::set_YG_ST(uint8_t val){
  register_val = i2cdev1.read_register(mpu6050_addr, GYRO_CONFIG);
  register_val = (register_val & YG_ST_LCN_0) | (val << YG_ST_LCN_START);
  i2cdev1.write_register(mpu6050_addr, GYRO_CONFIG, register_val);
}

// configuring ZG_ST
uint8_t MPU6050_driver::get_ZG_ST(){
  register_val = i2cdev1.read_register(mpu6050_addr, GYRO_CONFIG);
  return (register_val & ZG_ST_LCN_1) >> ZG_ST_LCN_START;
}

void MPU6050_driver::set_ZG_ST(uint8_t val){
  register_val = i2cdev1.read_register(mpu6050_addr, GYRO_CONFIG);
  register_val = (register_val & ZG_ST_LCN_0) | (val << ZG_ST_LCN_START);
  i2cdev1.write_register(mpu6050_addr, GYRO_CONFIG, register_val);
}

// configuring FS_SEL
uint8_t MPU6050_driver::get_FS_SEL(){
  register_val = i2cdev1.read_register(mpu6050_addr, GYRO_CONFIG);
  return (register_val & FS_SEL_LCN_1) >> FS_SEL_LCN_START;
}

void MPU6050_driver::set_FS_SEL(uint8_t val){
  register_val = i2cdev1.read_register(mpu6050_addr, GYRO_CONFIG);
  register_val = (register_val & FS_SEL_LCN_0) | (val << FS_SEL_LCN_START);
  i2cdev1.write_register(mpu6050_addr, GYRO_CONFIG, register_val);
}

/* Register Name: ACCEL_CONFIG (Accelerometer Configuration)
   Register Address (Hex / Decimal): 0x1C / 28
   Type : Read / Write
   Description: This register is used to trigger accelerometer self test and 
   configure the accelerometer full scale range. This register also configures
   the Digital High Pass Filter (DHPF).
   
   Parameters:
       XA_ST:   When set to 1, the X- Axis accelerometer performs self test.
       YA_ST:   When set to 1, the Y- Axis accelerometer performs self test.
       ZA_ST:   When set to 1, the Z- Axis accelerometer performs self test.
       AFS_SEL: 2-bit unsigned value. Selects the full scale range of accelerometers.
*/

// configuring XA_ST
uint8_t MPU6050_driver::get_XA_ST(){
  register_val = i2cdev1.read_register(mpu6050_addr, ACCEL_CONFIG);
  return (register_val & XA_ST_LCN_1) >> XA_ST_LCN_START;
}

void MPU6050_driver::set_XA_ST(uint8_t val){
  register_val = i2cdev1.read_register(mpu6050_addr, ACCEL_CONFIG);
  register_val = (register_val & XA_ST_LCN_0) | (val << XA_ST_LCN_START);
  i2cdev1.write_register(mpu6050_addr, ACCEL_CONFIG, register_val);
}

// configuring YA_ST
uint8_t MPU6050_driver::get_YA_ST(){
  register_val = i2cdev1.read_register(mpu6050_addr, ACCEL_CONFIG);
  return (register_val & YA_ST_LCN_1) >> YA_ST_LCN_START;
}

void MPU6050_driver::set_YA_ST(uint8_t val){
  register_val = i2cdev1.read_register(mpu6050_addr, ACCEL_CONFIG);
  register_val = (register_val & YA_ST_LCN_0) | (val << YA_ST_LCN_START);
  i2cdev1.write_register(mpu6050_addr, ACCEL_CONFIG, register_val);
}

// configuring ZA_ST
uint8_t MPU6050_driver::get_ZA_ST(){
  register_val = i2cdev1.read_register(mpu6050_addr, ACCEL_CONFIG);
  return (register_val & ZA_ST_LCN_1) >> ZA_ST_LCN_START;
}

void MPU6050_driver::set_ZA_ST(uint8_t val){
  register_val = i2cdev1.read_register(mpu6050_addr, ACCEL_CONFIG);
  register_val = (register_val & ZA_ST_LCN_0) | (val << ZA_ST_LCN_START);
  i2cdev1.write_register(mpu6050_addr, ACCEL_CONFIG, register_val);
}

// configuring AFS_SEL
uint8_t MPU6050_driver::get_AFS_SEL(){
  register_val = i2cdev1.read_register(mpu6050_addr, ACCEL_CONFIG);
  return (register_val & AFS_SEL_LCN_1) >> AFS_SEL_LCN_START;
}

void MPU6050_driver::set_AFS_SEL(uint8_t val){
  register_val = i2cdev1.read_register(mpu6050_addr, ACCEL_CONFIG);
  register_val = (register_val & AFS_SEL_LCN_0) | (val << AFS_SEL_LCN_START);
  i2cdev1.write_register(mpu6050_addr, ACCEL_CONFIG, register_val);
}

/* Register Name: PWR_MGMT_1 (Power Management 1)
   Register Address (Hex / Decimal): 0x6B / 107
   Type : Read / Write
   Description: This register allows the user to configure the power mode and 
   clock source. It also provides a bit for resetting the entire device, and
   a bit for disabling the temperature sensor.
   
   Parameters:
       DEVICE_RESET: When set to 1, this bit resets all internal registers to their default values.
                     The bit automatically clears to 0 once the reset is done.
                     The default values for each register can be found in Section 3.
                     Note: After device reset wait for 100 ms
       SLEEP:        When set to 1, this bit puts the MPU-60X0 into sleep mode.
       CYCLE:        When this bit is set to 1 and SLEEP is disabled, the MPU-60X0 will cycle between
                     sleep mode and waking up to take a single sample of data from active sensors at
                     a rate determined by LP_WAKE_CTRL (register 108).
       TEMP_DIS:     When set to 1, this bit disables the temperature sensor.
       CLKSEL:       3-bit unsigned value. Specifies the clock source of the device
                     Note: Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
                     However, it is highly recommended that the device be configured to use one of the
                     gyroscopes (or an external clock source) as the clock reference for improved stability.
*/

// using DEVICE_RESET feature
uint8_t MPU6050_driver::get_DEVICE_RESET(){
  register_val = i2cdev1.read_register(mpu6050_addr, PWR_MGMT_1);
  return (register_val & DEVICE_RESET_LCN_1) >> DEVICE_RESET_LCN_START;
}

void MPU6050_driver::set_DEVICE_RESET(uint8_t val){
  register_val = i2cdev1.read_register(mpu6050_addr, PWR_MGMT_1);
  register_val = (register_val & DEVICE_RESET_LCN_0) | (val << DEVICE_RESET_LCN_START);
  i2cdev1.write_register(mpu6050_addr, PWR_MGMT_1, register_val);
}

// using SLEEP feature
uint8_t MPU6050_driver::get_SLEEP(){
  register_val = i2cdev1.read_register(mpu6050_addr, PWR_MGMT_1);
  return (register_val & SLEEP_LCN_1) >> SLEEP_LCN_START;
}

void MPU6050_driver::set_SLEEP(uint8_t val){
  register_val = i2cdev1.read_register(mpu6050_addr, PWR_MGMT_1);
  register_val = (register_val & SLEEP_LCN_0) | (val << SLEEP_LCN_START);
  i2cdev1.write_register(mpu6050_addr, PWR_MGMT_1, register_val);
}

// using CYCLE feature
uint8_t MPU6050_driver::get_CYCLE(){
  register_val = i2cdev1.read_register(mpu6050_addr, PWR_MGMT_1);
  return (register_val & CYCLE_LCN_1) >> CYCLE_LCN_START;
}

void MPU6050_driver::set_CYCLE(uint8_t val){
  register_val = i2cdev1.read_register(mpu6050_addr, PWR_MGMT_1);
  register_val = (register_val & CYCLE_LCN_0) | (val << CYCLE_LCN_START);
  i2cdev1.write_register(mpu6050_addr, PWR_MGMT_1, register_val);
}

// using TEMP_DIS feature
uint8_t MPU6050_driver::get_TEMP_DIS(){
  register_val = i2cdev1.read_register(mpu6050_addr, PWR_MGMT_1);
  return (register_val & TEMP_DIS_LCN_1) >> TEMP_DIS_LCN_START;
}

void MPU6050_driver::set_TEMP_DIS(uint8_t val){
  register_val = i2cdev1.read_register(mpu6050_addr, PWR_MGMT_1);
  register_val = (register_val & TEMP_DIS_LCN_0) | (val << TEMP_DIS_LCN_START);
  i2cdev1.write_register(mpu6050_addr, PWR_MGMT_1, register_val);
}

// using CLKSEL feature
uint8_t MPU6050_driver::get_CLKSEL(){
  register_val = i2cdev1.read_register(mpu6050_addr, PWR_MGMT_1);
  return (register_val & CLKSEL_LCN_1) >> CLKSEL_LCN_START;
}

void MPU6050_driver::set_CLKSEL(uint8_t val){
  register_val = i2cdev1.read_register(mpu6050_addr, PWR_MGMT_1);
  register_val = (register_val & CLKSEL_LCN_0) | (val << CLKSEL_LCN_START);
  i2cdev1.write_register(mpu6050_addr, PWR_MGMT_1, register_val);
}

/* Register Name: PWR_MGMT_2 (Power Management 2)
   Register Address (Hex / Decimal): 0x6C / 108
   Type : Read / Write
   Description: This register allows the user to configure the frequency of
   wake-ups in Accelerometer Only Low Power Mode. This register also allows
   the user to put individual axes of the accelerometer and gyroscope into 
   standby mode. The MPU-60X0 can be put into Accelerometer Only Low Power
   Mode. In this mode, the device will power off all devices except for the
   primary I2C interface, waking only the accelerometer at fixed intervals to
   take a single measurement.
   Note: The user can put individual accelerometer and gyroscopes axes into
   standby mode by using this register. If the device is using a gyroscope
   axis as the clock source and this axis is put into standby mode, the clock
   source will automatically be changed to the internal 8MHz oscillator.
   
   Parameters:
       LP_WAKE_CTRL: 2-bit unsigned value. Specifies the frequency of wake-ups
                     during Accelerometer Only Low Power Mode.
       STBY_XA:      When set to 1, this bit puts the X axis accelerometer into standby mode.
       STBY_YA:      When set to 1, this bit puts the Y axis accelerometer into standby mode.
       STBY_ZA:      When set to 1, this bit puts the Z axis accelerometer into standby mode.
       STBY_XG:      When set to 1, this bit puts the X axis gyroscope into standby mode.
       STBY_YG:      When set to 1, this bit puts the Y axis gyroscope into standby mode.
       STBY_ZG:      When set to 1, this bit puts the Z axis gyroscope into standby mode.
*/

// using LP_WAKE_CTRL feature
uint8_t MPU6050_driver::get_LP_WAKE_CTRL(){
  register_val = i2cdev1.read_register(mpu6050_addr, PWR_MGMT_2);
  return (register_val & LP_WAKE_CTRL_LCN_1) >> LP_WAKE_CTRL_LCN_START;
}

void MPU6050_driver::set_LP_WAKE_CTRL(uint8_t val){
  register_val = i2cdev1.read_register(mpu6050_addr, PWR_MGMT_2);
  register_val = (register_val & LP_WAKE_CTRL_LCN_0) | (val << LP_WAKE_CTRL_LCN_START);
  i2cdev1.write_register(mpu6050_addr, PWR_MGMT_2, register_val);
}

// using STBY_XA feature
uint8_t MPU6050_driver::get_STBY_XA(){
  register_val = i2cdev1.read_register(mpu6050_addr, PWR_MGMT_2);
  return (register_val & STBY_XA_LCN_1) >> STBY_XA_LCN_START;
}

void MPU6050_driver::set_STBY_XA(uint8_t val){
  register_val = i2cdev1.read_register(mpu6050_addr, PWR_MGMT_2);
  register_val = (register_val & STBY_XA_LCN_0) | (val << STBY_XA_LCN_START);
  i2cdev1.write_register(mpu6050_addr, PWR_MGMT_2, register_val);
}

// using STBY_YA feature
uint8_t MPU6050_driver::get_STBY_YA(){
  register_val = i2cdev1.read_register(mpu6050_addr, PWR_MGMT_2);
  return (register_val & STBY_YA_LCN_1) >> STBY_YA_LCN_START;
}

void MPU6050_driver::set_STBY_YA(uint8_t val){
  register_val = i2cdev1.read_register(mpu6050_addr, PWR_MGMT_2);
  register_val = (register_val & STBY_YA_LCN_0) | (val << STBY_YA_LCN_START);
  i2cdev1.write_register(mpu6050_addr, PWR_MGMT_2, register_val);
}

// using STBY_ZA feature
uint8_t MPU6050_driver::get_STBY_ZA(){
  register_val = i2cdev1.read_register(mpu6050_addr, PWR_MGMT_2);
  return (register_val & STBY_ZA_LCN_1) >> STBY_ZA_LCN_START;
}

void MPU6050_driver::set_STBY_ZA(uint8_t val){
  register_val = i2cdev1.read_register(mpu6050_addr, PWR_MGMT_2);
  register_val = (register_val & STBY_ZA_LCN_0) | (val << STBY_ZA_LCN_START);
  i2cdev1.write_register(mpu6050_addr, PWR_MGMT_2, register_val);
}

// using STBY_XG feature
uint8_t MPU6050_driver::get_STBY_XG(){
  register_val = i2cdev1.read_register(mpu6050_addr, PWR_MGMT_2);
  return (register_val & STBY_XG_LCN_1) >> STBY_XG_LCN_START;
}

void MPU6050_driver::set_STBY_XG(uint8_t val){
  register_val = i2cdev1.read_register(mpu6050_addr, PWR_MGMT_2);
  register_val = (register_val & STBY_XG_LCN_0) | (val << STBY_XG_LCN_START);
  i2cdev1.write_register(mpu6050_addr, PWR_MGMT_2, register_val);
}

// using STBY_YG feature
uint8_t MPU6050_driver::get_STBY_YG(){
  register_val = i2cdev1.read_register(mpu6050_addr, PWR_MGMT_2);
  return (register_val & STBY_YG_LCN_1) >> STBY_YG_LCN_START;
}

void MPU6050_driver::set_STBY_YG(uint8_t val){
  register_val = i2cdev1.read_register(mpu6050_addr, PWR_MGMT_2);
  register_val = (register_val & STBY_YG_LCN_0) | (val << STBY_YG_LCN_START);
  i2cdev1.write_register(mpu6050_addr, PWR_MGMT_2, register_val);
}

// using STBY_ZG feature
uint8_t MPU6050_driver::get_STBY_ZG(){
  register_val = i2cdev1.read_register(mpu6050_addr, PWR_MGMT_2);
  return (register_val & STBY_ZG_LCN_1) >> STBY_ZG_LCN_START;
}

void MPU6050_driver::set_STBY_ZG(uint8_t val){
  register_val = i2cdev1.read_register(mpu6050_addr, PWR_MGMT_2);
  register_val = (register_val & STBY_ZG_LCN_0) | (val << STBY_ZG_LCN_START);
  i2cdev1.write_register(mpu6050_addr, PWR_MGMT_2, register_val);
}

/* Register Name: WHO_AM_I (Who Am I)
   Register Address (Hex / Decimal): 0x75 / 117
   Type : Read Only
   Description: This register is used to verify the identity of the device. The contents of
   WHO_AM_I are the upper 6 bits of the MPU-60X0’s 7-bit I2C address. The least significant
   bit of the MPU-60X0’s I2C address is determined by the value of the AD0 pin. The value
   of the AD0 pin is not reflected in this register. The default value of the register is 0x68.
   Bits 0 and 7 are reserved. (Hard coded to 0)
   
   Parameters:
       WHO_AM_I: Contains the 6-bit I2C address of the MPU-60X0.
                 The Power-On-Reset value of Bit6:Bit1 is 110 100.
*/

// using WHO_AM_I feature
uint8_t MPU6050_driver::get_WHO_AM_I(){
  register_val = i2cdev1.read_register(mpu6050_addr, WHO_AM_I);
  return (register_val & WHO_AM_I_LCN_1) >> WHO_AM_I_LCN_START;
}

/* Accelerometer raw data registers
   Register Name, address (Hex / Decimal): ACCEL_XOUT_H, (0x3B / 59)
                                           ACCEL_XOUT_L, (0x3C / 60)
                                           ACCEL_YOUT_H, (0x3D / 61)
                                           ACCEL_YOUT_L, (0x3E / 62)
                                           ACCEL_ZOUT_H, (0x3F / 63)
                                           ACCEL_ZOUT_L, (0x40 / 64)
       Note: H means 8 most significant bits & L means 8 least significant bits
   Type : Read only
   Description: These registers store the most recent accelerometer measurements.
   Accelerometer measurements are written to these registers at the Sample Rate
   as defined in Register 25. Each 16-bit accelerometer measurement has a full
   scale defined in ACCEL_FS (Register 28). For each full scale setting, the
   accelerometers’ have different sensitivity per LSB in ACCEL_xOUT.
   
   Parameters:
       ACCEL_XOUT: 16-bit 2’s complement value.
                   Stores the most recent X axis accelerometer measurement.
       ACCEL_YOUT: 16-bit 2’s complement value.
                   Stores the most recent Y axis accelerometer measurement.
       ACCEL_ZOUT: 16-bit 2’s complement value.
                   Stores the most recent Z axis accelerometer measurement
*/

uint8_t MPU6050_driver::get_ACCEL_XOUT_H(){
  return i2cdev1.read_register(mpu6050_addr, ACCEL_XOUT_H);
}

uint8_t MPU6050_driver::get_ACCEL_XOUT_L(){
  return i2cdev1.read_register(mpu6050_addr, ACCEL_XOUT_L);
}

uint8_t MPU6050_driver::get_ACCEL_YOUT_H(){
  return i2cdev1.read_register(mpu6050_addr, ACCEL_YOUT_H);
}

uint8_t MPU6050_driver::get_ACCEL_YOUT_L(){
  return i2cdev1.read_register(mpu6050_addr, ACCEL_YOUT_L);
}

uint8_t MPU6050_driver::get_ACCEL_ZOUT_H(){
  return i2cdev1.read_register(mpu6050_addr, ACCEL_ZOUT_H);
}

uint8_t MPU6050_driver::get_ACCEL_ZOUT_L(){
  return i2cdev1.read_register(mpu6050_addr, ACCEL_ZOUT_L);
}

int16_t MPU6050_driver::get_ACCEL_XOUT(){
  i2cdev1.read_registers(mpu6050_addr, ACCEL_XOUT_H, 2, buff, buff_ACCEL_XOUT_H);
  return (((int16_t)buff[buff_ACCEL_XOUT_H]) << 8) | buff[buff_ACCEL_XOUT_L];
}

int16_t MPU6050_driver::get_ACCEL_YOUT(){
  i2cdev1.read_registers(mpu6050_addr, ACCEL_YOUT_H, 2, buff, buff_ACCEL_YOUT_H);
  return (((int16_t)buff[buff_ACCEL_YOUT_H]) << 8) | buff[buff_ACCEL_YOUT_L];
}

int16_t MPU6050_driver::get_ACCEL_ZOUT(){
  i2cdev1.read_registers(mpu6050_addr, ACCEL_ZOUT_H, 2, buff, buff_ACCEL_ZOUT_H);
  return (((int16_t)buff[buff_ACCEL_ZOUT_H]) << 8) | buff[buff_ACCEL_ZOUT_L];
}

void MPU6050_driver::get_ACCEL_OUT(int16_t* AC_X, int16_t* AC_Y, int16_t* AC_Z){
    i2cdev1.read_registers(mpu6050_addr, ACCEL_XOUT_H, 6, buff, buff_ACCEL_XOUT_H);
    *AC_X = (((int16_t)buff[buff_ACCEL_XOUT_H]) << 8) | buff[buff_ACCEL_XOUT_L];
    *AC_Y = (((int16_t)buff[buff_ACCEL_YOUT_H]) << 8) | buff[buff_ACCEL_YOUT_L];
    *AC_Z = (((int16_t)buff[buff_ACCEL_ZOUT_H]) << 8) | buff[buff_ACCEL_ZOUT_L];
}

/* Temperature raw data registers
   Register Name, address (Hex / Decimal): TEMP_OUT_H, (0x41 / 65)
                                           TEMP_OUT_L, (0x42 / 66)
       Note: H means 8 most significant bits & L means 8 least significant bits
   Type : Read only
   Description: These registers store the most recent temperature sensor measurement.
   Temperature measurements are written to these registers at the Sample Rate as defined
   in Register 25. The scale factor and offset for the temperature sensor are found in
   the Electrical Specifications table (Section 6.4 of the MPU-6000/MPU-6050 Product
   Specification document).
   
   Parameters:
       TEMP_OUT: 16-bit signed value.
                 Stores the most recent temperature sensor measurement.
*/

uint8_t MPU6050_driver::get_TEMP_OUT_H(){
  return i2cdev1.read_register(mpu6050_addr, TEMP_OUT_H);
}

uint8_t MPU6050_driver::get_TEMP_OUT_L(){
  return i2cdev1.read_register(mpu6050_addr, TEMP_OUT_L);
}

int16_t MPU6050_driver::get_TEMP_OUT(){
  i2cdev1.read_registers(mpu6050_addr, TEMP_OUT_H, 2, buff, buff_TEMP_OUT_H);
  return (((int16_t)buff[buff_TEMP_OUT_H]) << 8) | buff[buff_TEMP_OUT_L];
}

/* Gyroscope raw data registers
   Register Name, address (Hex / Decimal): GYRO_XOUT_H, (0x43 / 67)
                                           GYRO_XOUT_L, (0x44 / 68)
                                           GYRO_YOUT_H, (0x45 / 69)
                                           GYRO_YOUT_L, (0x46 / 70)
                                           GYRO_ZOUT_H, (0x47 / 71)
                                           GYRO_ZOUT_L, (0x48 / 72)
       Note: H means 8 most significant bits & L means 8 least significant bits
   Type : Read only
   Description: These registers store the most recent gyroscope measurements.
   Gyroscope measurements are written to these registers at the Sample Rate as defined
   in Register 25. Each 16-bit gyroscope measurement has a full scale defined in FS_SEL
   (Register 27). For each full scale setting, the gyroscopes’ has a different sensitivity
   per LSB in GYRO_xOUT.
   
   Parameters:
       GYRO_XOUT: 16-bit 2’s complement value.
                  Stores the most recent X axis gyroscope measurement.
       GYRO_YOUT: 16-bit 2’s complement value.
                  Stores the most recent Y axis gyroscope measurement.
       GYRO_ZOUT: 16-bit 2’s complement value.
                  Stores the most recent Z axis gyroscope measurement.
*/

uint8_t MPU6050_driver::get_GYRO_XOUT_H(){
  return i2cdev1.read_register(mpu6050_addr, GYRO_XOUT_H);
}

uint8_t MPU6050_driver::get_GYRO_XOUT_L(){
  return i2cdev1.read_register(mpu6050_addr, GYRO_XOUT_L);
}

uint8_t MPU6050_driver::get_GYRO_YOUT_H(){
  return i2cdev1.read_register(mpu6050_addr, GYRO_YOUT_H);
}

uint8_t MPU6050_driver::get_GYRO_YOUT_L(){
  return i2cdev1.read_register(mpu6050_addr, GYRO_YOUT_L);
}

uint8_t MPU6050_driver::get_GYRO_ZOUT_H(){
  return i2cdev1.read_register(mpu6050_addr, GYRO_ZOUT_H);
}

uint8_t MPU6050_driver::get_GYRO_ZOUT_L(){
  return i2cdev1.read_register(mpu6050_addr, GYRO_ZOUT_L);
}

int16_t MPU6050_driver::get_GYRO_XOUT(){
  i2cdev1.read_registers(mpu6050_addr, GYRO_XOUT_H, 2, buff, buff_GYRO_XOUT_H);
  return (((int16_t)buff[buff_GYRO_XOUT_H]) << 8) | buff[buff_GYRO_XOUT_L];
}

int16_t MPU6050_driver::get_GYRO_YOUT(){
  i2cdev1.read_registers(mpu6050_addr, GYRO_YOUT_H, 2, buff, buff_GYRO_YOUT_H);
  return (((int16_t)buff[buff_GYRO_YOUT_H]) << 8) | buff[buff_GYRO_YOUT_L];
}

int16_t MPU6050_driver::get_GYRO_ZOUT(){
  i2cdev1.read_registers(mpu6050_addr, GYRO_ZOUT_H, 2, buff, buff_GYRO_ZOUT_H);
  return (((int16_t)buff[buff_GYRO_ZOUT_H]) << 8) | buff[buff_GYRO_ZOUT_L];
}

void MPU6050_driver::get_GYRO_OUT(int16_t* GY_X, int16_t* GY_Y, int16_t* GY_Z){
    i2cdev1.read_registers(mpu6050_addr, GYRO_XOUT_H, 6, buff, buff_GYRO_XOUT_H);
    *GY_X = (((int16_t)buff[buff_GYRO_XOUT_H]) << 8) | buff[buff_GYRO_XOUT_L];
    *GY_Y = (((int16_t)buff[buff_GYRO_YOUT_H]) << 8) | buff[buff_GYRO_YOUT_L];
    *GY_Z = (((int16_t)buff[buff_GYRO_ZOUT_H]) << 8) | buff[buff_GYRO_ZOUT_L];
}

void MPU6050_driver::get_MPU6050_OUT(int16_t* AC_X, int16_t* AC_Y, int16_t* AC_Z, int16_t* TEMP, int16_t* GY_X, int16_t* GY_Y, int16_t* GY_Z){
    i2cdev1.read_registers(mpu6050_addr, ACCEL_XOUT_H, 14, buff, buff_ACCEL_XOUT_H);
    *AC_X = (((int16_t)buff[buff_ACCEL_XOUT_H]) << 8) | buff[buff_ACCEL_XOUT_L];
    *AC_Y = (((int16_t)buff[buff_ACCEL_YOUT_H]) << 8) | buff[buff_ACCEL_YOUT_L];
    *AC_Z = (((int16_t)buff[buff_ACCEL_ZOUT_H]) << 8) | buff[buff_ACCEL_ZOUT_L];
    *TEMP = (((int16_t)buff[buff_TEMP_OUT_H])   << 8) | buff[buff_TEMP_OUT_L];
    *GY_X = (((int16_t)buff[buff_GYRO_XOUT_H])  << 8) | buff[buff_GYRO_XOUT_L];
    *GY_Y = (((int16_t)buff[buff_GYRO_YOUT_H])  << 8) | buff[buff_GYRO_YOUT_L];
    *GY_Z = (((int16_t)buff[buff_GYRO_ZOUT_H])  << 8) | buff[buff_GYRO_ZOUT_L];
}