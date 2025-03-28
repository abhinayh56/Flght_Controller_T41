#include <Arduino.h>
#include "RC_input.h"
#include "MB_1040_driver.h"
#include "I2Cdev.h"
#include "I2Cdev1.h"
#include "MPU6050_driver.h"
#include "HMC5883L_driver.h"
#include "MS5611_driver.h"
#include "PWM_driver.h"
#include "Estimate_angle.h"
#include "Barometric_height.h"
#include "Math_functions.h"
#include "Prop_sys_char.h"
#include "Actuator_mixing.h"
#include "PID_controller.h"
#include "Filter.h"
#include "Map_lin_out_2_ang_in.h"
#include "Kalman_1D.h"
#include "Navigation.h"

// test 1

Navigation wp_nav;
float t_strt_nav;

// time variable starts ------------------------------------------------------------------------------------
float SampleFrequency = 400, dt = 1.0/SampleFrequency, loop_timer = 1000000.0*dt, t;
float tn;
int16_t counter_bar = 0;
int16_t counter_mag = 0;
bool mag_available = 0;
bool bar_p_available = 0;
bool bar_t_available = 0;
// time variables ends -------------------------------------------------------------------------------------


// RC input variables starts -------------------------------------------------------------------------------
uint8_t mode;
uint8_t arm_status;
float throttle;
float roll;
float pitch;
float yaw;
RC_input rc;
// RC input variables ends ---------------------------------------------------------------------------------


// Sonar variables variables starts ------------------------------------------------------------------------
MB_1040_driver son;
// Sonar variables variables ends --------------------------------------------------------------------------


// I2C driver starts ---------------------------------------------------------------------------------------
I2Cdev i2c_master;
I2Cdev1 i2c_master1;
// I2C driver ends -----------------------------------------------------------------------------------------


// MPU6050 variables variables starts ----------------------------------------------------------------------
MPU6050_driver mpu6050(MPU6050_ADDR_DEFAULT);
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
float ax_b, ay_b, az_b, temp_imu, wx_b, wy_b, wz_b; // assuming body frame and sensor frame is same
// MPU6050 variables variables starts ----------------------------------------------------------------------


// HMC5883L variables variables starts ---------------------------------------------------------------------
HMC5883L_driver hmc5883l;
int16_t mx_b, my_b, mz_b;
// HMC5883L variables variables ends -----------------------------------------------------------------------


// PWM variables variables starts --------------------------------------------------------------------------
PWM_driver pwm;
// PWM variables variables ends ----------------------------------------------------------------------------


// MB1040 sonar data starts --------------------------------------------------------------------------------
float h_sonar;
float h_sonar_pre = 0;
// MB1040 sonar data ends ----------------------------------------------------------------------------------

// TF mini plus variables starts ---------------------------------------------------------------------------
int dist;
int strength;
float temprature;
int check;
uint8_t uart[9];
const uint8_t HEADER = 0x59;
bool lidar_data = false;
bool lidar_data_valid = false;
unsigned long t_lidar;
bool new_data = false;
uint16_t counter_100Hz_lidar = 0;

void tfmp_restore_factory_set();
void tfmp_set_frame_rate(uint16_t frame_rate=100);
void tfmp_set_output_format();
void tfmp_save_settings();
float get_lidar_data();
// TF mini plus variables starts ---------------------------------------------------------------------------


// MS5611 variables variables starts -----------------------------------------------------------------------
MS5611_driver bar;
double temp;
double pressure;
// MS5611 variables variables ends -------------------------------------------------------------------------


// Orientation estimation variables starts -----------------------------------------------------------------
Estimate_angle angle_state;
float phi, th, psi;

Filter_LP lpf_wx;
Filter_LP lpf_wy;
Filter_LP lpf_wz;
// Orientation estimation variables ends -------------------------------------------------------------------

// Height estimation variables starts ----------------------------------------------------------------------
Barometric_height atm_thickness;
Filter_LP lpf_bar_z;
Filter_LP lpf_son;
// Height estimation variables ends ------------------------------------------------------------------------

// Controller variables starts -----------------------------------------------------------------------------
float u1=0, u2=0, u3=0, u4=0;

// Rate controller
PID_controller rate_x_controller;
PID_controller rate_y_controller;
PID_controller rate_z_controller;

// Angle controller
PID_controller angle_x_controller;
PID_controller angle_y_controller;
PID_controller angle_z_controller;

// Height controller
uint16_t counter_20Hz = 0;
uint16_t counter_100Hz = 0;
PID_controller velocity_z_controller;
PID_controller position_z_controller;
float u1_0 = 0;

// data logging
uint16_t counter_100Hz_data_log = 0;

// position controller
PID_controller velx_controller;
PID_controller vely_controller;
PID_controller position_x_controller;
PID_controller position_y_controller;
Map_lin_out_2_ang_in acc_2_angle;
// Controller variables ends -------------------------------------------------------------------------------

// Thrust & moments to rotor speed and motor driving variables starts --------------------------------------
Actuator_mixing mma;

struct Motor{
  float rpm = 0;
  float pwm = 1000;
};
struct Motor motor1, motor2, motor3, motor4;

#define PWM_MIN 1000.0F
#define PWM_MAX 1950.0F
// Thrust & moments to rotor speed and motor driving variables ends ----------------------------------------

// Xtras variables starts ----------------------------------------------------------------------------------
Math_functions math;
// Xtras variables ends ------------------------------------------------------------------------------------

Prop_sys_char prop_sys;

struct Data_packet{
  byte init = 0x98;
  unsigned long seq = 0;
  unsigned long t = 0;
  byte payload[10] = {0,0,0,0,0,0,0,0,0,0};
  byte check_sum = 0;
  byte fin = 0x99;
};

struct Data_packet data_packet;

const char UBLOX_INIT[] PROGMEM = {
  // Disable NMEA
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x23, // GxGGA off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x2A, // GxGLL off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x31, // GxGSA off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x38, // GxGSV off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x3F, // GxRMC off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x46, // GxVTG off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x4D, // GxGRS off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x06,0x54, // GxGST off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x5B, // GxZDA off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x09,0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x62, // GxGBS off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x0A,0x00,0x00,0x00,0x00,0x00,0x00,0x09,0x69, // GxDTM off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x0D,0x00,0x00,0x00,0x00,0x00,0x00,0x0C,0x7E, // GxGNS off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x0E,0x8C, // GxVLW off

  // Disable UBX
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x16,0xD5, //NAV-SOL off
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x17,0xDC, //NAV-PVT off
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0xB9, //NAV-POSLLH off
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x13,0xC0, //NAV-STATUS off

  // Enable UBX (I2C, UART1, USB, SPI)
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x04,0x01,0x01,0x00,0x01,0x01,0x00,0x18,0xD7, //NAV-DOP
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x01,0x01,0x01,0x00,0x01,0x01,0x00,0x15,0xC2, //NAV-POSECEF
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x01,0x01,0x00,0x01,0x01,0x00,0x16,0xC9, //NAV-POSLLH
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x01,0x01,0x00,0x01,0x01,0x00,0x1B,0xEC, //NAV-PVT
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x06,0x01,0x01,0x00,0x01,0x01,0x00,0x1A,0xE5, //NAV-SOL
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x11,0x01,0x01,0x00,0x01,0x01,0x00,0x25,0x32, //NAV-VELECEF
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x12,0x01,0x01,0x00,0x01,0x01,0x00,0x26,0x39, //NAV-VELNED
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x01,0x01,0x00,0x01,0x01,0x00,0x17,0xD0, //NAV-STATUS

  // Enable UBX (UART1)
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x04,0x00,0x01,0x00,0x00,0x00,0x00,0x15,0xCC, //NAV-DOP
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x01,0x00,0x01,0x00,0x00,0x00,0x00,0x12,0xB7, //NAV-POSECEF
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x01,0x00,0x00,0x00,0x00,0x13,0xBE, //NAV-POSLLH
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x18,0xE1, //NAV-PVT
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x06,0x00,0x01,0x00,0x00,0x00,0x00,0x17,0xDA, //NAV-SOL
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x11,0x00,0x01,0x00,0x00,0x00,0x00,0x22,0x27, //NAV-VELECEF
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x12,0x00,0x01,0x00,0x00,0x00,0x00,0x23,0x2E, //NAV-VELNED
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x01,0x00,0x00,0x00,0x00,0x14,0xC5, //NAV-STATUS

  // Rate
  0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12, //(10Hz)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0x7D,0x00,0x01,0x00,0x01,0x00,0x93,0xA8, //(8Hz)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A, //(5Hz)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0xFA,0x00,0x01,0x00,0x01,0x00,0x10,0x96, //(4Hz)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0xF4,0x01,0x01,0x00,0x01,0x00,0x0B,0x77, //(2Hz)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39, //(1Hz)

  // Baud
  //0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x80,0x25,0x00,0x00,0x07,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0xA2,0xB5, //9600
  //0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0xE1,0x00,0x00,0x07,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0xDE,0xC9, //57600
  0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0xC2,0x01,0x00,0x07,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0xC0,0x7E, //115200
};

const unsigned char UBX_HEADER[]     = {0xB5,0x62};
const unsigned char NAV_SOL_HEADER[] = {0x01,0x06};
const unsigned char NAV_PVT_HEADER[] = {0x01,0x07};

enum _ubxMsgType {
  MT_NONE,
  MT_NAV_SOL,
  MT_NAV_PVT
};

#define U1 unsigned char
#define I1 char
#define X1 unsigned char
#define U2 unsigned short
#define I2 short
#define X2 unsigned short
#define U4 unsigned long
#define I4 long
#define X4 unsigned long
#define R4 float
#define CH char

struct NAV_SOL { //length(Bytes) : 56
  unsigned char cls; //1 Byte
  unsigned char id; //1 Byte
  unsigned short len; //2 Byte //52 Bytes of payload below //total = 1+1+2+52=56
  U4 iTOW; //ms
  I4 fTow; //ns
  I2 week; //weeks
  U1 gpsFix; 
  X1 flags;
  I4 ecefX; //cm
  I4 ecefY; //cm
  I4 ecefZ; //cm
  U4 pAcc; //cm
  I4 ecefVX; //cm/s
  I4 ecefVY; //cm/s
  I4 ecefVZ; //cm/s
  U4 sAcc; //cm/s
  U2 pDOP;
  U1 reserved1;
  U1 numSV;
  U4 reserved2;
};

struct NAV_PVT { //length(Bytes) : 92
  unsigned char cls; //1 Byte
  unsigned char id; //1 Byte
  unsigned short len; //2 Byte //84 Bytes of payload below //total = 1+1+2+84=88
  U4 iTOW; //ms
  U2 year; //y
  U1 month; //month
  U1 day; //d
  U1 hour; //h
  U1 min; //min
  U1 sec; //s
  X1 valid;
  U4 tAcc; //ns
  I4 nano; //ns
  U1 fixType;
  X1 flags;
  X1 flag2;
  U1 numSV;
  I4 lon; //deg // scaling 1e-7
  I4 lat; //deg // scaling 1e-7
  I4 height; //mm
  I4 hMSL; //mm
  U4 hAcc; //mm
  U4 vAcc; // mm
  I4 velN; //mm/s
  I4 velE; //mm/s
  I4 velD; //mm/s`
  I4 gSpeed; //mm/s
  I4 headMot; //deg // scaling 1e-5
  U4 sAcc; //mm/s
  U4 headAcc; //deg // scaling 1e-5
  U2 pDOP; // scaling 0.01
  X2 flags3;
  U4 reserved1;
  I4 headVeh;
  I2 magDec;
  U2 magAcc;
};

union UBXMessage {
  NAV_SOL navSol;
  NAV_PVT navPvt;
};

UBXMessage ubxMessage;

U1 numsv;
U1 fixType;

// The last two bytes of the message is a checksum value, used to confirm that the received payload is valid.
// The procedure used to calculate this is given as pseudo-code in the uBlox manual.
void calcChecksum(unsigned char* CK, int msgSize) {
  memset(CK, 0, 2);
  for (int i = 0; i < msgSize; i++) {
    CK[0] += ((unsigned char*)(&ubxMessage))[i];
    CK[1] += CK[0];
  }
}

// Compares the first two bytes of the ubxMessage struct with a specific message header.
// Returns true if the two bytes match.
boolean compareMsgHeader(const unsigned char* msgHeader) {
  unsigned char* ptr = (unsigned char*)(&ubxMessage);
  return ptr[0] == msgHeader[0] && ptr[1] == msgHeader[1];
}

// Reads in bytes from the GPS module and checks to see if a valid message has been constructed.
// Returns the type of the message found if successful, or MT_NONE if no message was found.
// After a successful return the contents of the ubxMessage union will be valid, for the 
// message type that was found. Note that further calls to this function can invalidate the
// message content, so you must use the obtained values before calling this function again.
int processGPS() {
  static int fpos = 0;
  static unsigned char checksum[2];
  
  static byte currentMsgType = MT_NONE;
  static int payloadSize = sizeof(UBXMessage);

  while(Serial3.available()){
    byte c = Serial3.read(); //Serial.write(c);
    if (fpos<2){
      // For the first two bytes we are simply looking for a match with the UBX header bytes (0xB5,0x62)
      if (c==UBX_HEADER[fpos])
        fpos++;
      else
        fpos = 0; // Reset to beginning state.
    }
    else{
      // If we come here then fpos >= 2, which means we have found a match with the UBX_HEADER
      // and we are now reading in the bytes that make up the payload.
      
      // Place the incoming byte into the ubxMessage struct. The position is fpos-2 because
      // the struct does not include the initial two-byte header (UBX_HEADER).
      if((fpos-2)<payloadSize)
        ((unsigned char*)(&ubxMessage))[fpos-2] = c;
      fpos++;
      if (fpos==4){
        // We have just received the second byte of the message type header, 
        // so now we can check to see what kind of message it is.
        if (compareMsgHeader(NAV_SOL_HEADER)){
          currentMsgType = MT_NAV_SOL;
          payloadSize = sizeof(NAV_SOL);
        }
        else if(compareMsgHeader(NAV_PVT_HEADER)){
          currentMsgType = MT_NAV_PVT;
          payloadSize = sizeof(NAV_PVT);
        }
        else{
          // unknown message type, bail
          fpos = 0;
          continue;
        }
      }

      if(fpos==(payloadSize+2)){
        // All payload bytes have now been received, so we can calculate the 
        // expected checksum value to compare with the next two incoming bytes.
        calcChecksum(checksum, payloadSize);
      }
      else if(fpos==(payloadSize+3)){
        // First byte after the payload, ie. first byte of the checksum.
        // Does it match the first byte of the checksum we calculated?
        if (c!= checksum[0]){
          // Checksum doesn't match, reset to beginning state and try again.
          fpos = 0; 
        }
      }
      else if(fpos==(payloadSize+4)){
        // Second byte after the payload, ie. second byte of the checksum.
        // Does it match the second byte of the checksum we calculated?
        fpos = 0; // We will reset the state regardless of whether the checksum matches.
        if(c==checksum[1]){
          // Checksum matches, we have a valid message.
          return currentMsgType; 
        }
      }
      else if(fpos>(payloadSize+4)){
        // We have now read more bytes than both the expected payload and checksum 
        // together, so something went wrong. Reset to beginning state and try again.
        fpos = 0;
      }
    }
  }
  return MT_NONE;
}

long x_ecef0;
long y_ecef0;
long z_ecef0;

long x_ecef;
long y_ecef;
long z_ecef;

long x_ecef_x_ecef0;
long y_ecef_y_ecef0;
long z_ecef_z_ecef0;

float x_nwu;
float y_nwu;

long lat; //deg // scaling 1e-7
long lon; //deg // scaling 1e-7
long X; //cm
long Y; //cm
long Z; //cm
long vx; //mm/s
long vy; //mm/s

float phi0;
float lam0;
float sin_phi0;
float cos_phi0;
float sin_lam0;
float cos_lam0;

int start_gps_var = 1;
bool gps_available = 0;
bool gps_vel_available = 0;
bool gps_pos_available = 0;
uint16_t acc_index = 39;
uint16_t counter_10Hz = 0;

#define NN 160  //delta_t*400*4/3
float ax_buff[NN];
float ay_buff[NN];
float vx_buff[NN];
float vy_buff[NN];

// Function prototypes -------------------------------------------------------------------------------------
void wait();
void print_pilot_input_data();
void print_raw_mpu6050_data();
void print_mpu6050_physical_data();
void print_raw_hmc5883l_data();
void calibrate_gyro();
void calibrate_gyro2();
void print_angles();
void print_angular_velocity();
void calibrate_acc_out();
void print_rpm();
void print_pwm();
void drive_motors();
void set_to_5Hz();
void set_to_8Hz();
void set_to_10Hz();
void set_baud_115200();
void disable_GPGSV();
void compensate_delay();

struct States{
  float x = 0;
  float y = 0;
  float z = 0;
  float vx = 0;
  float vy = 0;
  float vz = 0;
  float ax = 0;
  float ay = 0;
  float az = 0;
  float phi = 0;
  float th = 0;
  float psi = 0;
  float wx = 0;
  float wy = 0;
  float wz = 0;
};

struct States_des{
  float x0 = 0;
  float y0 = 0;
  float z0 = 0;
  float vx0 = 0;
  float vy0 = 0;
  float vz0 = 0;
  float ax0 = 0;
  float ay0 = 0;
  float az0 = 0;
  float phi0 = 0;
  float th0 = 0;
  float psi0 = 0;
  float wx0 = 0;
  float wy0 = 0;
  float wz0 = 0;
};

struct States state;
struct States_des state_des;

struct datastore {
  float phi;
  float phi0;
  float freq;
};
struct datastore myData;

// microcontroller 2 for battery voltage starts
float V_bat = 12.25;
uint8_t receive_buffer_uc2[7] = {0,0,0,0,0,0,0};

union BytesToFloat{
  char buff[4];
  float val;
};

union BytesToFloat b2f;
// microcontroller 2 for battery voltage ends

PID_controller roll_controller;
PID_controller pitch_controller;
PID_controller yaw_controller;

Kalman_1D height_estimator;

void setup(){
  delay(1000);
  // Initialize PWM generation -----------------------------------------------------------------------------
  pwm.initialize_PWM(motor1.pwm, motor2.pwm, motor3.pwm, motor4.pwm);

  // Initialize serial communication -----------------------------------------------------------------------
  Serial.begin(115200); // for usb comunication
  Serial1.begin(57600); // for xbee communication
  Serial3.begin(9600); // for GPS communication
  Serial2.begin(115200); // for lidar communication
  Serial7.begin(115200); // for arduino Nano (battery voltage and buzzer)

  // Initialize receiver ppm detection ---------------------------------------------------------------------
  rc.initialize_rc_ppm();

  // Initialize sonar pwm detection ------------------------------------------------------------------------
  son.initialize_sonar();

  // initialize I2C communication on master arduino --------------------------------------------------------
  i2c_master.initialize();
  i2c_master1.initialize();

  // initialize and configure MPU6050 (accelerometer + gyroscope) ------------------------------------------
  mpu6050.initialize(); //imu.set_SLEEP(SET_0);
  mpu6050.set_DLPF_CFG(DLPF_CFG_1);
  mpu6050.set_SMPLRT_DIV(SMPLRT_DIV);
  mpu6050.set_FS_SEL(FS_SEL_1);
  mpu6050.set_AFS_SEL(AFS_SEL_1);

  // initialize and confiure HMC5883L (magnetometer) -------------------------------------------------------
  //Serial.println("Initializing hmc5883l!");
  hmc5883l.initialize();
  //Serial.println("MD:");
  //Serial.println(hmc5883l.get_MD());
  hmc5883l.set_MD(0); // 0 (continuous meas.) to 1 (single meas. mode)
  //Serial.println(hmc5883l.get_MD());
  //Serial.println("MS:");
  //Serial.println(hmc5883l.get_MS());
  hmc5883l.set_MS(0); // measurement mode (refers to direction of current flowing inside magnetoresistive element) 0 to 3 (set it to 0)
  //Serial.println(hmc5883l.get_MS());
  //Serial.println("DO:");
  //Serial.println(hmc5883l.get_DO());
  hmc5883l.set_DO(6); //0 to 6 75 Hz data output rate in Hz
  //Serial.println(hmc5883l.get_DO());
  //Serial.println("MA:");
  //Serial.println(hmc5883l.get_MA());
  hmc5883l.set_MA(0); // set for averaging 0,1,2,3:1,2,4,8
  //Serial.println(hmc5883l.get_MA());
  //Serial.println("GN:");
  //Serial.println(hmc5883l.get_GN());
  hmc5883l.set_GN(1); // 0 to 8 (magnetic field on surface of the earth 0.3,0.6 gauss)
  //Serial.println(hmc5883l.get_GN());
  
  // initialize and confiure MS5611 (barometer) ------------------------------------------------------------
  //Serial.println("1. bar initializing!");
  bar.initialize();
  delay(100);
  //Serial.println("2. bar initialized!");

  //Serial.println("3. reading calibration!");
  bar.read_calibration_data();
  delay(100);
  //Serial.println("4. reading calibration registers successful!");
  //Serial.println("5. Calibration data:");
  //Serial.println(bar.get_C1()); // 49247
  //Serial.println(bar.get_C2()); // 50280
  //Serial.println(bar.get_C3()); // 30245
  //Serial.println(bar.get_C4()); // 27125
  //Serial.println(bar.get_C5()); // 31094
  //Serial.println(bar.get_C6()); // 27458

  //Serial.println(bar.get_OSR());
  bar.set_OSR(4096); // set over-sampling rate
  //Serial.println(bar.get_OSR());

  // orientation estimation starts -------------------------------------------------------------------------
  //angle_state.set_sensitivity_ac(8192.0); // comment this to use calibrated scale factor
  angle_state.set_sensitivity_gy(65.5);
  angle_state.set_sensitivity_mg(1090.0);
  //calibrate_gyro(); // using exponential filter
  //calibrate_gyro2(); // using average
  //calibrate_acc_out(); // it just gives the range of values, manually calculate bias and sensitivity using it
  lpf_wx.set_alpha_LPF(0.718);
  lpf_wy.set_alpha_LPF(0.718);
  lpf_wz.set_alpha_LPF(0.718);

  // height estimatin starts
  lpf_son.set_alpha_LPF(0.325);
  lpf_bar_z.set_alpha_LPF(0.75);

  // configure GPS
  // send configuration data in UBX protocol
  delay(1500);
  for(uint32_t i = 0; i < sizeof(UBLOX_INIT); i++) {                        
    Serial3.write(pgm_read_byte(UBLOX_INIT+i) );
    delay(5); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }
  Serial3.begin(115200);

  // rate controller starts
  u1 = 0;
  u2 = 0;
  u3 = 0;
  u4 = 0;

  // integral should work only when the vehicle is in the air, on the ground only PD controller should work
  // otherwise even the slightest the incorrection in neutral angle will cause the vechicle to topple
  rate_x_controller.set_gains(225,30,14); //200, 30, 11.60 // 228.6,30,14.25
  rate_y_controller.set_gains(225,30,14);
  rate_z_controller.set_gains(85,40,0);

  rate_x_controller.set_I_max(9910000); // maximum torque in gm.cm all four motor can produce using integral term
  rate_y_controller.set_I_max(9910000);
  rate_z_controller.set_I_max(10000);

  rate_x_controller.set_output_max(9920000);
  rate_y_controller.set_output_max(20000);
  rate_z_controller.set_output_max(20000);

  rate_x_controller.set_lpf_alpha(0.8642); // 0.8642
  rate_y_controller.set_lpf_alpha(0.8642);
  rate_z_controller.set_lpf_alpha(0);

  // angle controller starts
  angle_x_controller.set_gains(6,0,0); //8.5 // 10 is very good but agressive
  angle_y_controller.set_gains(6,0,0);
  angle_z_controller.set_gains(2.57,0,0); //Kp=2.5, Ki = NA, Kd = Not mandatary (integeal term in rate control)

  angle_x_controller.set_I_max(225);
  angle_y_controller.set_I_max(225);
  angle_z_controller.set_I_max(125);

  angle_x_controller.set_output_max(225);
  angle_y_controller.set_output_max(225);
  angle_z_controller.set_output_max(125);

  angle_x_controller.set_lpf_alpha(0);
  angle_y_controller.set_lpf_alpha(0);
  angle_z_controller.set_lpf_alpha(0);

  // XY controller starts
  velx_controller.set_gains(0,0,0);
  velx_controller.set_I_max(4.0);
  velx_controller.set_output_max(4.0);
  velx_controller.set_lpf_alpha(0);

  vely_controller.set_gains(0,0,0);
  vely_controller.set_I_max(4.0);
  vely_controller.set_output_max(4.0);
  vely_controller.set_lpf_alpha(0);

  position_x_controller.set_gains(0,0,0);
  position_x_controller.set_I_max(100.0);
  position_x_controller.set_output_max(100);
  position_x_controller.set_lpf_alpha(0);

  position_y_controller.set_gains(0,0,0);
  position_y_controller.set_I_max(100.0);
  position_y_controller.set_output_max(100);
  position_y_controller.set_lpf_alpha(0);

  // position controller starts
  position_z_controller.set_gains(0,0,0);
  position_z_controller.set_I_max(1000.0);
  position_z_controller.set_output_max(2000.0);
  position_z_controller.set_lpf_alpha(0);

  // velocity controller
  velocity_z_controller.set_gains(1200,0,0);
  velocity_z_controller.set_I_max(1000);
  velocity_z_controller.set_output_max(1200);
  velocity_z_controller.set_lpf_alpha(0);
  
  /* // accelerometer calibration parameter calculations
  float ax_max =  8350.9;
  float ax_min = -7958.7;
  float ay_max =  9149.0;
  float ay_min = -7074.0;
  float az_max =  8596.9;
  float az_min = -8013.0;

  float sx = (ax_max - ax_min)*0.5;
  float sy = (ay_max - ay_min)*0.5;
  float sz = (az_max - az_min)*0.5;

  float bx = (ax_max + ax_min)*0.5/sx;
  float by = (ay_max + ay_min)*0.5/sy;
  float bz = (az_max + az_min)*0.5/sz;

  Serial.print(sx,6); Serial.print('\t');
  Serial.print(sy,6); Serial.print('\t');
  Serial.print(sz,6); Serial.print('\t');
  Serial.print(bx,6); Serial.print('\t');
  Serial.print(by,6); Serial.print('\t');
  Serial.println(bz,6);*/

  for(int i=0;i<NN;i++){
    ax_buff[i] = 0;
    ay_buff[i] = 0;
    vx_buff[i] = 0;
    vy_buff[i] = 0;
  }

  // initialize TF mini plus lidar
  tfmp_restore_factory_set();
  tfmp_set_frame_rate(100);
  tfmp_set_output_format();
  tfmp_save_settings();
  
  for(int i=0;i<9;i++){
    uart[i] = 0;
  }

  /////////////////////////////////// TEMP ////////////////////////////////////
  roll_controller.set_gains(0,0,0);
  roll_controller.set_I_max(99999999);
  roll_controller.set_output_max(99999999);
  roll_controller.set_lpf_alpha(0);

  pitch_controller.set_gains(0,0,0);
  pitch_controller.set_I_max(99999999);
  pitch_controller.set_output_max(99999999);
  pitch_controller.set_lpf_alpha(0);

  yaw_controller.set_gains(0,0,0);
  yaw_controller.set_I_max(99999999);
  yaw_controller.set_output_max(99999999);
  yaw_controller.set_lpf_alpha(0);

  roll_controller.initialize();
  pitch_controller.initialize();
  yaw_controller.initialize();
  /////////////////////////////////////////////////////////////////////////////

  height_estimator.set_dt(dt);
  height_estimator.set_sen_res(0.5);
  height_estimator.set_input_noise(200.0); // 200 cm/sec^2

  height_estimator.init_states(0,0,0);
  height_estimator.init_states_cov(9999,9999,9999);

  delay(5000);

  wp_nav.init(0.5);

  t = micros();
}

float ax_ = 0;
float ay_ = 0;
float phi0_ = 0;
float th0_ = 0;
float I_x = 0;
float I_y = 0;

float I_vx = 0;
float I_vy = 0;

void loop(){
  Serial.print(micros()); Serial.print('\t');
  // SECTION 1: RC INPUT -----------------------------------------------------------------------------------
  rc.int_pilot_cmd(&mode, &arm_status, &throttle, &roll, &pitch, &yaw);
  if(mode==0){ // attitude control
    state_des.phi0 = roll;
    state_des.th0 = pitch;
    state_des.psi0 = yaw;

    if(arm_status==0 || arm_status==1){
      rc.set_yaw_cmd_angle(state.psi*57.29578);
      state_des.psi0 = state.psi*57.29578;
    }
  }
  else if(mode==1){ // height control
    state_des.phi0 = roll;
    state_des.th0  = pitch;
    state_des.psi0 = yaw;
  }
  else{ // mode=2 // position control
    state_des.phi0 = roll;
    state_des.th0  = pitch;
    state_des.psi0 = yaw;
  }

  // print_pilot_input_data();

  // SECTION 2: READ IMU DATA ------------------------------------------------------------------------------
  mpu6050.get_MPU6050_OUT(&AcX, &AcY, &AcZ, &Tmp, &GyX, &GyY, &GyZ);
  int16_t tempAcX = AcX;
  AcX = -AcY;
  AcY = tempAcX;
  int16_t tempGyX = GyX;
  GyX = -GyY;
  GyY = tempGyX;
  //print_raw_mpu6050_data();
  print_mpu6050_physical_data();

  mag_available = 0;
  if(counter_mag==0){
    hmc5883l.get_MAG_OUT(&mx_b, &my_b, &mz_b);
    //int16_t temp = mx_b;
    mx_b = mx_b;
    my_b = -my_b;
    mz_b = -mz_b;
    mag_available = 1;
  }
  if(counter_mag==4) counter_mag = -1;
  counter_mag++;
  Serial.print(mag_available); Serial.print('\t');
  print_raw_hmc5883l_data();


  // SECTION 3: READ SONAR DATA ----------------------------------------------------------------------------
  if(counter_20Hz==20){
    h_sonar = ((float)son.get_sonar_pw()/147.0)*2.54; // sonar height in cm
    //Serial.print(((float)son.get_sonar_pw()/147.0)*2.54); Serial.print('\t');
  }
  //Serial.print(((float)son.get_sonar_pw()/147.0)*2.54); Serial.print('\t');

  // SECTION 4: READ MS5611 DATA ---------------------------------------------------------------------------
  bar_p_available = 0;
  bar_t_available = 0;
  if(counter_bar==0){
    bar.request_raw_pressure();
  }
  else if((counter_bar==4) || (counter_bar==8)){
    bar.get_raw_pressure();
    bar_p_available = 1;
    bar.request_raw_pressure();
  }
  else if(counter_bar==12){
    bar.get_raw_pressure();
    bar_p_available = 1;
    bar.request_raw_temperature();
  }
  else if(counter_bar==16){
    bar.get_raw_temperature();
    bar_t_available = 1;
    counter_bar = -1;
  }
  counter_bar++;

  if(bar_t_available==1){
    bar.cal_temp();
    temp = bar.get_temp_C();
  }

  if(bar_p_available==1){
    bar.cal_temp_compensated_pressure();
    pressure = bar.get_pressure_mbar();
  }
  Serial.print(bar_t_available); Serial.print('\t');
  Serial.print(bar_p_available); Serial.print('\t');
  Serial.print(temp,HEX); Serial.print('\t');
  Serial.print(pressure,HEX); Serial.print('\t');
  
  // SECTION 5: READ GPS DATA ------------------------------------------------------------------------------
  gps_available = 0;
  gps_vel_available = 0;
  gps_pos_available = 0;
  int msgType = processGPS();

  if(msgType==MT_NAV_SOL){
    gps_available = 1;
    gps_pos_available = 1;
    //Serial.println("nav sol");
    X = ubxMessage.navSol.ecefX;
    Y = ubxMessage.navSol.ecefY;
    Z = ubxMessage.navSol.ecefZ;
    if(start_gps_var<=50){
      x_ecef0 = X;
      y_ecef0 = Y;
      z_ecef0 = Z;
      start_gps_var++;
    }
    else{
      x_ecef = X;
      y_ecef = Y;
      z_ecef = Z;
      x_ecef_x_ecef0 = x_ecef - x_ecef0;
      y_ecef_y_ecef0 = y_ecef - y_ecef0;
      z_ecef_z_ecef0 = z_ecef - z_ecef0;

      x_nwu = (-float(x_ecef_x_ecef0) * cos_lam0 * sin_phi0) + (-float(y_ecef_y_ecef0) * sin_lam0 * sin_phi0) + (float(z_ecef_z_ecef0) * cos_phi0); //cm
      y_nwu = (float(x_ecef_x_ecef0) * sin_lam0) + (-float(y_ecef_y_ecef0) * cos_lam0); //cm
    }
  }
  else if ( msgType == MT_NAV_PVT ){
    gps_available = 1;
    gps_vel_available = 1;
    //Serial.println("nav pvt");
    lat = ubxMessage.navPvt.lat;
    lon = ubxMessage.navPvt.lon;
    vx = ubxMessage.navPvt.velN; //mm/s
    vy = -ubxMessage.navPvt.velE; //mm/s
    numsv = ubxMessage.navPvt.numSV;
    fixType = ubxMessage.navPvt.fixType;
    if(start_gps_var<=50){
      phi0 = float(lat)*(PI/180)/10000000; // lat0
      lam0 = float(lon)*(PI/180)/10000000; // lon0
      sin_phi0 = sin(phi0);
      cos_phi0 = cos(phi0);
      sin_lam0 = sin(lam0);
      cos_lam0 = cos(lam0);
      start_gps_var++;
    }
    else{
    }
  }

  Serial.print(gps_available); Serial.print('\t');
  Serial.print(gps_pos_available); Serial.print('\t');
  Serial.print(gps_vel_available); Serial.print('\t');

  Serial.print(numsv); Serial.print('\t');
  Serial.print(fixType); Serial.print('\t');
  
  Serial.print(x_ecef); Serial.print('\t');
  Serial.print(y_ecef); Serial.print('\t');
  Serial.print(z_ecef); Serial.print('\t');

  Serial.print(lat); Serial.print('\t');
  Serial.print(lon); Serial.print('\t');

  Serial.print(x_nwu); Serial.print('\t');
  Serial.print(y_nwu); Serial.print('\t');

  Serial.print(vx); Serial.print('\t');
  Serial.print(vy); Serial.print('\t');

  /*Serial.print(x_nwu); Serial.print('\t');
  Serial.print(y_nwu); Serial.print('\t');
  Serial.print(vx); Serial.print('\t');
  Serial.print(vy); Serial.print('\t');
  Serial.print(state.ax*981,3); Serial.print('\t');
  Serial.print(state.ay*981,3); Serial.print('\t');
  Serial.print(state.az*981,3); Serial.print('\t');*/
  //Serial.print(numsv); Serial.print('\t');
  //Serial.print(fixType); Serial.print('\t');
  
  /*bool newGPSdata = false;
  while(Serial3.available()){
    char c = Serial3.read();
    if (gps.encode(c)) {
      newGPSdata = true;
    }
  }*/

  /*gps.get_position(&lat, &lon, &age); // Lat/Long(10^-5 deg)
  gps.f_get_position(&flat, &flon, &age);
  gps.get_datetime(&date, &time, &age);
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  float gps_altitude = gps.altitude(); // in cm
  float gps_course = gps.course(); // in 10^-2 deg
  float gps_speed = gps.speed(); // 10^-2 knots
  float gps_faltitude = gps.f_altitude();
  float gps_fcourse = gps.f_course();
  float gps_fspeed_kmph = gps.f_speed_kmph(); // kmph
  gps.stats(&chars, &sentences, &failed);*/
  
  //Serial.print(state.ax*981,3); Serial.print('\t');
  //Serial.print(state.ay*981,3); Serial.print('\t');
  //Serial.print(state.az*981,3); Serial.print('\t');
  //Serial.print(flat,10); Serial.print('\t');
  //Serial.print(flon,10); Serial.print('\t');
  //Serial.print(gps_faltitude); Serial.print('\t');
  //Serial.print(gps_fspeed_mps*100.0); Serial.print('\t');
  //Serial.print(gps_fcourse); Serial.print('\t');
  //Serial.print(atm_thickness.get_barometric_height2(pressure)*100); Serial.print('\t');
  //Serial.print(bar_p_available); Serial.print('\t');

  // SECTION 6: READ BATTERY VOLTAGE -----------------------------------------------------------------------
  V_bat = 11.5;
  while(Serial7.available()){
    uint8_t c = Serial7.read();
    //Serial.println(c,HEX);

    for(int i=0;i<6;i++){
      receive_buffer_uc2[i] = receive_buffer_uc2[i+1];
    }
    receive_buffer_uc2[6] = c;

    bool v_bat_data_valid = false;
    if((receive_buffer_uc2[0]==0x75) && receive_buffer_uc2[1]==0x01){
      uint8_t check_sum_uc2 = (receive_buffer_uc2[0] + receive_buffer_uc2[1] + receive_buffer_uc2[2] + receive_buffer_uc2[3] + receive_buffer_uc2[4] + receive_buffer_uc2[5]) & 0xFF;
      if(receive_buffer_uc2[6] == check_sum_uc2){
        v_bat_data_valid = true;
      }
    }
    if(v_bat_data_valid==true){
      b2f.buff[3] = receive_buffer_uc2[2];
      b2f.buff[2] = receive_buffer_uc2[3];
      b2f.buff[1] = receive_buffer_uc2[4];
      b2f.buff[0] = receive_buffer_uc2[5];

      V_bat = b2f.val;

      /*for(int i=0;i<7;i++){
        Serial.print(receive_buffer_uc2[i],HEX); Serial.print(" ");
      }
      Serial.print('\t');
      Serial.print(V_bat);
      Serial.println();*/
    }
  }
  //Serial.println(V_bat);
  
  // SECTION 7: ESTIMATE ORIENTATION -----------------------------------------------------------------------
  if(mag_available==1){
    angle_state.update_angles(AcX,AcY,AcZ,GyX,GyY,GyZ,mx_b,my_b,mz_b);
    //angle_state.update_angles(AcX,AcY,AcZ,GyX,GyY,GyZ);
  }
  else{
    angle_state.update_angles(AcX,AcY,AcZ,GyX,GyY,GyZ);
  }

  angle_state.get_angles(&state.phi, &state.th, &state.psi);
  angle_state.get_wxyz_b(&state.wx, &state.wy, &state.wz);
  angle_state.get_axyz_F(&state.ax,&state.ay,&state.az);
  
  state.wx = lpf_wx.apply_LPF(state.wx);
  state.wy = lpf_wy.apply_LPF(state.wy);
  state.wz = lpf_wz.apply_LPF(state.wz);
  print_angles();
  print_angular_velocity();

  // SECTION 8: ESTIMATE HEIGHT ----------------------------------------------------------------------------
  // height from lidar starts here
  float z_lidar = get_lidar_data();
  Serial.print(z_lidar); Serial.print('\t');
  //Serial.println(z_lidar*cos(state.phi)*cos(state.th)); // lidar data is valid even at angle ~70-80 degree
  z_lidar = z_lidar*cos(state.phi)*cos(state.th);
  height_estimator.predict(angle_state.get_az_F()*9.81*100.0);

  float bias_az = 0;
  height_estimator.get_estimates(&state.z,&state.vz,&bias_az);
  state.az = angle_state.get_az_F()*9.81*100.0 - bias_az;

  if(counter_100Hz_lidar==4){ // for lidar outputs data at 100 Hz
    height_estimator.correct(z_lidar);
    
    float bias_az = 0;
    height_estimator.get_estimates(&state.z,&state.vz,&bias_az);
    state.az = angle_state.get_az_F()*9.81*100.0 - bias_az;
    
    counter_100Hz_lidar = 0;
  }
  counter_100Hz_lidar++;

  /*Serial.print(state.z); Serial.print('\t');
  Serial.print(state.vz); Serial.print('\t');
  Serial.print(state.az); Serial.print('\t');
  Serial.print(bias_az); Serial.print('\t');
  Serial.println();*/
  // height from lidar ends here
  
  /*
  // height from sonar starts here
  if(counter_20Hz==20){ // for sonar as MB1040 outputs data at 20 Hz
    if(h_sonar-h_sonar_pre>25){ // (500)cm/sec*(1/20)s
      h_sonar = h_sonar_pre + 25;
    }
    else if(h_sonar-h_sonar_pre<-25){
      h_sonar = h_sonar_pre - 25;
    }
    else{
      h_sonar = h_sonar;
    }
    //Serial.println(h_sonar-h_sonar_pre);
    h_sonar_pre = h_sonar;

    state.z = h_sonar*0.01;
    //Serial.print(state.z*100.0); Serial.print('\t');
    state.z = lpf_son.apply_LPF(state.z);
    //Serial.println(state.z*100.0);
    height_kf.correct(state.z);
    counter_20Hz = 0;
  }
  counter_20Hz++;
  height_kf.predict(angle_state.get_az_F()*9.81);
  state.z = height_kf.get_z();
  state.vz = height_kf.get_vz();
  //Serial.print(state.z*100); Serial.print('\t');
  //Serial.print(state.vz*100); Serial.print('\t');
  // height from sonar ends here
  */
  

  /*
  // height from baro starts here ---------------------------------------------------
  //Serial.print(atm_thickness.get_barometric_height2(pressure)*100); Serial.print('\t');
  if(bar_p_available==1){
    state.z = atm_thickness.get_barometric_height2(pressure);
    if(isnan(state.z)) state.z = 0;
    //Serial.print(state.z*100); Serial.print('\t');
    state.z = lpf_bar_z.apply_LPF(state.z);
    //Serial.print(state.z*100); Serial.print('\t');
    height_kf.correct(state.z);
  }
  //Serial.print(state.z*100); Serial.print('\t');
  height_kf.estimate_height(angle_state.get_az_F(),state.z);
  height_kf.predict(angle_state.get_az_F()*9.81);
  state.z = height_kf.get_z();
  state.vz = height_kf.get_vz();
  //Serial.print(state.z*100); Serial.print('\t');
  //Serial.print(state.vz*100); Serial.print('\t');
  // height from baro ends here ---------------------------------------------------
  */

  // SECTION 9: ESTIMATE HORIZONTAL POSITION ---------------------------------------------------------------
  // delay compensation (delay is 0.3 sec)
  
  state.x = x_nwu; //cm
  state.y = y_nwu; //cm
  state.vx = vx*0.1; //cm/sec
  state.vy = vy*0.1; //cm/sec
  /*Serial.print(state.x); Serial.print('\t');
  Serial.print(state.y); Serial.print('\t');
  Serial.print(state.vx); Serial.print('\t');
  Serial.print(state.vy); Serial.print('\t');*/
  //float vx_backup = vx*0.1;
  //float vy_backup = vy*0.1;

  // delay compensation in GPS
  compensate_delay();
  Serial.print(state.x); Serial.print('\t');
  Serial.print(state.y); Serial.print('\t');
  Serial.print(state.vx); Serial.print('\t');
  Serial.print(state.vy); Serial.print('\t');
  
  // SECTION 10: WAY POINT PLANNER -------------------------------------------------------------------------
  // SECTION 11: TRAJECTORY PLANNER ------------------------------------------------------------------------
  // SECTION 12: POSITION CONTROLLER -----------------------------------------------------------------------
  // SECTION 13: VELOCITY CONTROLLER -----------------------------------------------------------------------
  // SECTION 1X: LINEAR CONTROLLER OUTPUT TO ANGLE CONTROLLER INPUT MAPPING --------------------------------
  // SECTION 14: ALTITUDE CONTROLLER -----------------------------------------------------------------------  
  if(counter_100Hz==4){
    u1 = 0;
    if(mode==0){
      u1 = (throttle/9.81)*1000.0; // gm
      state_des.z0 = state.z;
      state_des.vz0 = 0;
      u1_0 = u1;
      position_z_controller.reset();
      velocity_z_controller.reset();
      velx_controller.reset();
      vely_controller.reset();
      phi0_ = 0;
      th0_ = 0;
      I_x = 0;
      I_y = 0;
      state_des.x0 = state.x;
      state_des.y0 = state.y;
      I_vx = 0;
      I_vy = 0;
    }
    else if(mode==1){
      // height controller starts
      position_z_controller.set_gains(1200,200,1100);

      /*float xyz = 0;
      xyz = math.linear_map((float)rc.get_aux2_cmd(),967,2074,-0.01,1.0);
      xyz = math.saturate(xyz,0,1.0);
      Serial.println(xyz);*/

      position_z_controller.set_lpf_alpha(0.35);
      //u1 = u1_0 + position_z_controller.calculate_output2(state_des.z0,state.z,0.01);
      
      /*
      state_des.vz0 = 0;
      if((state.z*0.01>=5) || (state.z*0.01<0.65)){
        state_des.vz0 = 0;
      }
      else{
        if((float)rc.get_aux2_cmd()<1400){
          state_des.vz0 = math.linear_map((float)rc.get_aux2_cmd(),967,1400,-0.65,0);
        }
        else if((float)rc.get_aux2_cmd()>1560){
          state_des.vz0 = math.linear_map((float)rc.get_aux2_cmd(),1560,2074,0,1.5);
        }
        else{
          state_des.vz0 = 0;
        }
      }
      state_des.vz0 = math.saturate(state_des.vz0,-0.75,1.5);
      //Serial.print(state.vz*0.01*100,3); Serial.print('\t');
      //Serial.println(state_des.vz0*100.0,3);
      */

      state_des.vz0 = 1.0*(state_des.z0*0.01 - state.z*0.01);
      state_des.vz0 = math.saturate(state_des.vz0,-0.75,2.5);

      velocity_z_controller.set_gains(1100,1100,0);
      float kaz = 10;
      u1 = u1_0 + velocity_z_controller.calculate_output3(state_des.vz0,state.vz*0.01,dt*4.0) + kaz*(state_des.az0-state.az*0.01);
      u1 = u1/(cos(state.phi)*cos(state.th));
      u1 = math.saturate(u1,0.5*u1_0,2.0*u1_0);
      phi0_ = 0;
      th0_ = 0;
      I_x = 0;
      I_y = 0;
      state_des.x0 = state.x;
      state_des.y0 = state.y;
      // height controller ends
      I_vx = 0;
      I_vy = 0;
      velx_controller.reset();
      vely_controller.reset();
      t_strt_nav = micros();
    }
    else if(mode==2){
      // height controller starts
      position_z_controller.set_gains(1200,200,1100);
      position_z_controller.set_lpf_alpha(0.35);
      state_des.vz0 = 1.0*(state_des.z0*0.01 - state.z*0.01);
      state_des.vz0 = math.saturate(state_des.vz0,-0.75,2.5);

      velocity_z_controller.set_gains(1100,1100,0);
      float kaz = 10;
      u1 = u1_0 + velocity_z_controller.calculate_output3(state_des.vz0,state.vz*0.01,dt*4.0) + kaz*(state_des.az0-state.az*0.01);
      u1 = u1/(cos(state.phi)*cos(state.th));
      u1 = math.saturate(u1,0.5*u1_0,2.0*u1_0);
      // height controller ends
      
      ////////////////////////////////////////////////////////////////////////////////////
      /*if(rc.get_aux3_cmd()<1400){
        t_strt_nav = micros();
      }
      else{
        float t_nav_now = (micros() - t_strt_nav)*0.000001;
        Serial.print(t_nav_now); Serial.print('\t');
        wp_nav.update(t_nav_now);
        state_des.x0 = wp_nav.get_x()*100.0;
        state_des.y0 = wp_nav.get_y()*100.0;
      }*/
      
      //Serial.print(state_des.x0); Serial.print('\t');
      //Serial.println(state_des.y0);
      ////////////////////////////////////////////////////////////////////////////////////
      
      // xy controller starts
      /*state_des.vx0 = state_des.th0*(2.25/30.0);
      state_des.vy0 = -state_des.phi0*(2.25/30.0);
      
      state_des.x0 += state_des.vx0*100.0*dt*4;
      state_des.y0 += state_des.vy0*100.0*dt*4;*/

      float kp_xy_temp = 0.25;
      state_des.vx0 = kp_xy_temp*(state_des.x0*0.01 - state.x*0.01);
      state_des.vy0 = kp_xy_temp*(state_des.y0*0.01 - state.y*0.01);

      state_des.vx0 = math.saturate(state_des.vx0,-2.5,2.5);
      state_des.vy0 = math.saturate(state_des.vy0,-2.5,2.5);

      state_des.phi0 = 0;
      state_des.th0 = 0;

      float kp_v = 1.15;//1.65;
      float ki_v = 0.0625;
      float kd_v = 0.125; //<0.175

      //kp_v = math.linear_map((float)rc.get_aux3_cmd(),967,2074,-0.025,1.0);
      //kp_v = math.saturate(kp_v,0,1.0);

      //ki_v = math.linear_map((float)rc.get_aux3_cmd(),967,2074,2,15);
      //ki_v = math.saturate(ki_v,0,15);

      velx_controller.set_gains(kp_v,ki_v,kd_v);
      vely_controller.set_gains(kp_v,ki_v,kd_v);
      
      velx_controller.set_lpf_alpha(0.75);
      vely_controller.set_lpf_alpha(0.75);

      ax_ = velx_controller.calculate_output2(state_des.vx0,state.vx*0.01,dt*4.0);
      ay_ = vely_controller.calculate_output2(state_des.vy0,state.vy*0.01,dt*4.0);

      phi0_ = acc_2_angle.acxy_2_angle_phi0(ax_,ay_,state.psi)*180.0/PI;
      th0_  = acc_2_angle.acxy_2_angle_th0(ax_,ay_,state.psi)*180.0/PI;

      phi0_ = math.saturate(phi0_,-35,35);
      th0_  = math.saturate(th0_,-35,35);

      /*Serial.print(state_des.vx0); Serial.print('\t');
      Serial.print(ax_); Serial.print('\t');
      Serial.print(phi0_); Serial.print('\t');

      Serial.print(state_des.vy0); Serial.print('\t');
      Serial.print(ay_); Serial.print('\t');
      Serial.println(th0_);*/

      //Serial.print(phi0_); Serial.print('\t');
      //Serial.print(th0_); Serial.println('\t');
    }
    counter_100Hz = 0;
  }
  counter_100Hz++;
  /*Serial.print(state_des.z0); Serial.print('\t');
  Serial.print(state.z); Serial.print('\t');
  Serial.print(u1_0); Serial.print('\t');
  Serial.println(u1);*/

  if(counter_100Hz_data_log==1){
    /*Serial1.print(state.phi*180.0/PI); Serial1.print('\t');
    Serial1.print(state.th*180.0/PI); Serial1.print('\t');
    Serial1.print(state.psi*180.0/PI); Serial1.print('\t');*/
    /*Serial1.print(state.wx*180.0/PI); Serial1.print('\t');
    Serial1.print(state.wy*180.0/PI); Serial1.print('\t');
    Serial1.print(state.wz*180.0/PI); Serial1.print('\t');*/
    /*Serial1.print(state.ax*981.0); Serial1.print('\t'); // cm/sec^2
    Serial1.print(state.ay*981.0); Serial1.print('\t');
    Serial1.print(state.az*981.0); Serial1.print('\t');*/

    //Serial1.print(vx_backup); Serial1.print('\t'); // cm/sec
    //Serial1.print(vy_backup); Serial1.print('\t'); // cm/sec

    //Serial1.print(state.vx); Serial1.print('\t'); // cm/sec
    /*Serial1.println(state.vy*0.1); Serial1.print('\t');
    Serial1.print(state.vz*100.0); Serial1.print('\t');
    
    Serial1.print(state.x*1.0); Serial1.print('\t'); // cm
    Serial1.print(state.y*1.0); Serial1.print('\t');
    Serial1.println(state.z*100.0);*/ //Serial1.print('\t');
    //Serial1.println();

    //Serial1.print(state.ax*981.0); Serial1.print('\t');
    //Serial1.print(vx_backup); Serial1.print('\t');
    //Serial1.print(state.vx); Serial1.print('\t');

    //Serial1.print(state.ay*981.0); Serial1.print('\t');
    //Serial1.print(vy_backup); Serial1.print('\t');
    //Serial1.print(state.vy); Serial1.print('\t');

    /*Serial1.print(state.ax*981.0); Serial1.print('\t');
    Serial1.print(x_nwu); Serial1.print('\t');
    Serial1.print(state.x); Serial1.print('\t');

    Serial1.print(state.ay*981.0); Serial1.print('\t');
    Serial1.print(y_nwu); Serial1.print('\t');
    Serial1.println(state.y); //Serial1.print('\t');*/

    /*Serial1.print(phi0_); Serial1.print('\t');
    Serial1.print(th0_); Serial1.print('\t');
    Serial1.print(state.vx); Serial1.print('\t');
    Serial1.print(state.vy); Serial1.print('\t');*/
    //Serial1.println();

    //Serial1.print(state.ax*981.0); Serial1.print('\t');
    //Serial1.print(state.vx); Serial1.print('\t');
    //Serial1.print(state.x); Serial1.print('\t');
    //Serial1.println(state.y);
    
    /*Serial1.print(state.phi*180.0/PI); Serial1.print('\t');
    Serial1.print(state.th*180.0/PI); Serial1.print('\t');*/

    //Serial1.print(state.z*100); Serial1.print('\t');
    //Serial1.print(state.z*100*(cos(state.phi)*cos(state.th))); Serial1.print('\t');

    /*Serial1.print(state_des.phi0); Serial1.print('\t');
    Serial1.print(state.phi*180.0/PI); Serial1.print('\t');
    Serial1.print(state_des.th0); Serial1.print('\t');
    Serial1.println(state.th*180.0/PI); //Serial1.print('\t');*/

    unsigned int csk = 0;

    data_packet.init = 0x98;

    data_packet.seq = data_packet.seq + 1;
    csk += data_packet.seq;
    
    data_packet.t = millis();
    csk += data_packet.t;

    for(int i=0; i<10; i++){
      data_packet.payload[i] = i;
      csk += data_packet.payload[i];
    }
    data_packet.check_sum = csk;
    data_packet.fin  = 0x99;

    Serial1.print(data_packet.init);
    Serial1.print(data_packet.seq);
    Serial1.print(',');
    Serial1.print(data_packet.t);
    Serial1.print(',');
    Serial1.print(data_packet.payload[0]);
    Serial1.print(data_packet.payload[1]);
    Serial1.print(data_packet.payload[2]);
    Serial1.print(data_packet.payload[3]);
    Serial1.print(data_packet.payload[4]);
    Serial1.print(data_packet.payload[5]);
    Serial1.print(data_packet.payload[6]);
    Serial1.print(data_packet.payload[7]);
    Serial1.print(data_packet.payload[8]);
    Serial1.print(data_packet.payload[9]);
    Serial1.print(',');
    Serial1.print(data_packet.check_sum);
    Serial1.print(',');
    Serial1.println(data_packet.fin);
    
  // byte init = 0x98;
  // unsigned long seq = 0;
  // unsigned long t = 0;
  // char payload[10] = "Payload10";
  // byte check_sum = 0;
  // byte fin = 0x99;
    

    /*Serial1.print(state.x); Serial1.print('\t');
    Serial1.print(state.y); Serial1.print('\t');
    Serial1.print(state.z); Serial1.print('\t');
    Serial1.println();*/
    
    //Serial1.print(state_des.vz0*0.01*100,3); Serial1.print('\t');
    //Serial1.println(state.vz*0.01,3);

    /*Serial1.print(state_des.x0); Serial1.print('\t');
    Serial1.print(state_des.y0); Serial1.print('\t');
    Serial1.print(state_des.z0); Serial1.print('\t');
    Serial1.print(state.x); Serial1.print('\t');
    Serial1.print(state.y); Serial1.print('\t');
    Serial1.println(state.z);*/
  }
  counter_100Hz_data_log++;
  if(counter_100Hz_data_log>8){
    counter_100Hz_data_log = 1;
  }

  // SECTION 15: ATTITUDE CONTROLLER -----------------------------------------------------------------------
  u2 = 0; //50*16; // gm-cm
  u3 = 0; // gm-cm
  u4 = 0; // gm-cm
  
  if((arm_status==0) || (arm_status==1)){
    angle_x_controller.reset();
    angle_y_controller.reset();
    angle_z_controller.reset();
    
    rate_x_controller.reset();
    rate_y_controller.reset();
    rate_z_controller.reset();

    roll_controller.reset();
    pitch_controller.reset();

    velx_controller.reset();
    vely_controller.reset();
  }
  else{
    /*float kp_temp = 9; // 7.45
    float ki_temp = 0; // 100
    float kd_temp = 0; // 7.87

    kp_temp = math.linear_map((float)rc.get_aux2_cmd(),967,2074,-2,15);
    kp_temp = math.saturate(kp_temp,0,15);

    kd_temp = math.linear_map((float)rc.get_aux3_cmd(),967,2074,-2,5);
    kd_temp = math.saturate(kd_temp,0,5);*/
    
    float phi_neutral = -2.75;
    float th_neutral  = 1.86;

    if(mode==2){
      state_des.phi0 = 0;
      state_des.th0  = 0;
    }

    /*Serial.print(state_des.vx0); Serial.print('\t');
    Serial.print(state.vx); Serial.print('\t');
    Serial.println(state_des.th0);*/

    float e_phi = phi_neutral + state_des.phi0 + phi0_ - state.phi*180.0/PI;
    float e_th  = th_neutral  + state_des.th0  + th0_  - state.th *180.0/PI;
    float e_psi = state_des.psi0 - state.psi*180.0/PI;
    
    e_phi = math.wrap(e_phi,-180,180);
    e_th  = math.wrap(e_th ,-180,180);
    e_psi = math.wrap(e_psi,-180,180);

    //float kp_temp1 = 0; //250, 300, 400
    //kp_temp1 = math.linear_map((float)rc.get_aux2_cmd(),967,2074,-5,5);
    //kp_temp1 = 7.25 + math.saturate(kp_temp1,-5,5);

    angle_x_controller.set_gains(7.25,0,0);
    angle_y_controller.set_gains(7.25,0,0);

    state_des.wx0 = angle_x_controller.get_Kp()*e_phi;
    state_des.wy0 = angle_y_controller.get_Kp()*e_th;
    state_des.wz0 = angle_z_controller.get_Kp()*e_psi;

    state_des.wx0 = math.saturate(state_des.wx0,-225,225);
    state_des.wy0 = math.saturate(state_des.wy0,-225,225);
    state_des.wz0 = math.saturate(state_des.wz0,-125,125);

    //Serial.print(kp_temp1); Serial.print('\t');
    //Serial.print(ki_temp); Serial.print('\t');
    //Serial.print(kd_temp); Serial.print('\t');
    //Serial.print(-225); Serial.print('\t');
    //Serial.print(225); Serial.print('\t');
    //Serial.print(phi_neutral + state_des.phi0 + phi0_); Serial.print('\t');
    //Serial.print(state.phi*57.29578); Serial.println('\t');
    //Serial.println(state_des.wx0);// Serial.print('\t');
    //Serial.print(state.wx*57.29578); Serial.print('\t');
    //Serial.println(u2);

    // SECTION 16: RATE CONTROLLER ---------------------------------------------------------------------------
    //float kp_temp = 350; //250, 300, 400
    //float ki_temp = 250;
    //float kd_temp = 13.85; //12.25, 13, 13.5

    //kp_temp = math.linear_map((float)rc.get_aux2_cmd(),967,2074,0,150);
    //kp_temp = 250.0 + math.saturate(kp_temp,0,150);

    //ki_temp = math.linear_map((float)rc.get_aux3_cmd(),967,2074,-50,500);
    //ki_temp = math.saturate(ki_temp,0,500);
    
    //rate_x_controller.set_gains(350,250,13.85); // small high frequency oscillations at ths gain
    //rate_y_controller.set_gains(350,250,13.85);

    //rate_x_controller.set_gains(325,200,13); // good gain values but vehicle is slight slugish at Kp 300 so 325 is used
    //rate_y_controller.set_gains(325,200,13);
    /*
    (210,700,12)     -> Works very good in flight
    (250,0,12.25)
    (300,0,13)       -> A little bit of high frequency oscillations in slight
    (325,200,13)     -> Lot of high frequency oscillations in flight
    (350,250,13.85)  -> Lot of high frequency oscillations in flight
    (400,0,13.5)
    */

    rate_x_controller.set_gains(210,700,12); //(250,0,12.25),(300,0,13),(325,200,13),(350,250,13.85),(400,0,13.5)
    rate_y_controller.set_gains(210,700,12);
    
    //state_des.wx0 = state_des.phi0*7.0;

    //Serial.print(state_des.wx0); Serial.print('\t');
    //Serial.print(state.wx*57.29578); Serial.print('\t');
    //Serial.print(kp_temp); Serial.print('\t');
    //Serial.print(ki_temp); Serial.print('\t');
    //Serial.print(kd_temp); Serial.print('\t');
    //Serial.println();

    u2 = rate_x_controller.calculate_output3(state_des.wx0,state.wx*57.29578,dt);
    u3 = rate_y_controller.calculate_output3(state_des.wy0,state.wy*57.29578,dt);
    u4 = rate_z_controller.calculate_output2(state_des.wz0,state.wz*57.29578,dt);

    //u2 = 0;
    //u3 = 0;
    //u4 = 0;
  }
  
  // SECTION 16: THRUST & MOMENTS TO ROTOR RPM -------------------------------------------------------------
  mma.calc_U_to_rpm(u1,u2,u3,u4);
  mma.get_rpm1234(&motor1.rpm, &motor2.rpm, &motor3.rpm, &motor4.rpm); // here rpm in rps (i.e. rotation per seconds)
  //print_rpm();

  // SECTION 17: ROTOR RPM TO PWM --------------------------------------------------------------------------
  motor1.pwm = prop_sys.rotor_speed_to_pwm(motor1.rpm, V_bat);
  motor2.pwm = prop_sys.rotor_speed_to_pwm(motor2.rpm, V_bat);
  motor3.pwm = prop_sys.rotor_speed_to_pwm(motor3.rpm, V_bat);
  motor4.pwm = prop_sys.rotor_speed_to_pwm(motor4.rpm, V_bat);
  //print_pwm();

  // SECTION 18: DRIVE MOTOR -------------------------------------------------------------------------------
  drive_motors();
  //print_pwm();
  /*Serial.print(motor1.pwm); Serial.print('\t');
  Serial.print(motor4.pwm); Serial.print('\t');
  Serial.print(PWM_MIN); Serial.print('\t');
  Serial.print(PWM_MIN+50); Serial.print('\t');
  Serial.print(PWM_MAX-50); Serial.print('\t');
  Serial.println(PWM_MAX);*/

  tn = t;
  Serial.println(1000000.0/(micros()-tn)); //Serial.print("\t");
  wait();
  //Serial.println(1000000.0/(micros()-tn));
  //Serial.println();
}

void wait(){
  while(micros()-t<loop_timer){}
  t = micros();
}

void print_pilot_input_data(){
  Serial.print(throttle); Serial.print('\t');
  Serial.print(roll); Serial.print('\t');
  Serial.print(pitch); Serial.print('\t');
  Serial.print(yaw); Serial.print('\t');
  Serial.print(mode); Serial.print('\t');
  Serial.print(arm_status); Serial.print('\t');
  Serial.print(rc.get_aux1_cmd()); Serial.print('\t');
  Serial.print(rc.get_aux2_cmd()); Serial.print('\t');
  Serial.print(rc.get_aux3_cmd()); Serial.print('\t');
}

void print_raw_mpu6050_data(){
  Serial.print(AcX); Serial.print('\t');
  Serial.print(AcY); Serial.print('\t');
  Serial.print(AcZ); Serial.print('\t');
  Serial.print(Tmp); Serial.print('\t');
  Serial.print(GyX); Serial.print('\t');
  Serial.print(GyY); Serial.print('\t');
  Serial.print(GyZ); Serial.print('\t');
}

void print_mpu6050_physical_data(){
  ax_b = (float)AcX/8192;
  ay_b = (float)AcY/8192;
  az_b = (float)AcZ/8192;
  temp_imu = (float)Tmp/340.0 + 36.53; // in degree C
  wx_b = (float)GyX/65.5;
  wy_b = (float)GyY/65.5;
  wz_b = (float)GyZ/65.5;

  Serial.print(ax_b,HEX); Serial.print('\t');
  Serial.print(ay_b,HEX); Serial.print('\t');
  Serial.print(az_b,HEX); Serial.print('\t');
  Serial.print(temp_imu,HEX); Serial.print('\t');
  Serial.print(wx_b,HEX); Serial.print('\t');
  Serial.print(wy_b,HEX); Serial.print('\t');
  Serial.print(wz_b,HEX); Serial.print('\t');
}

void print_raw_hmc5883l_data(){
  Serial.print(mx_b); Serial.print('\t'); //445,-342
  Serial.print(my_b); Serial.print('\t'); //200, -596
  Serial.print(mz_b); Serial.print('\t');
}

void calibrate_gyro(){
  for(int i=0;i<15000;i++){
    mpu6050.get_MPU6050_OUT(&AcX, &AcY, &AcZ, &Tmp, &GyX, &GyY, &GyZ);
    int16_t tempGyX = GyX;
    GyX = -GyY;
    GyY = tempGyX;
    angle_state.calibrate_gyro(GyX, GyY, GyZ);
    //Serial.print(millis()); Serial.print('\t');
    if(i%10==0){
      Serial.print(angle_state.get_bias_gy_x(),4); Serial.print('\t');
      Serial.print(angle_state.get_bias_gy_y(),4); Serial.print('\t');
      Serial.println(angle_state.get_bias_gy_z(),4);
    }
    wait();
  }
}

void calibrate_gyro2(){
  uint32_t N = 5000;
  for(uint32_t i=0;i<N;i++){
    mpu6050.get_MPU6050_OUT(&AcX, &AcY, &AcZ, &Tmp, &GyX, &GyY, &GyZ);
    int16_t tempGyX = GyX;
    GyX = -GyY;
    GyY = tempGyX;
    angle_state.calibrate_gyro2(GyX, GyY, GyZ);
    //Serial.print("-"); Serial.print('\t');
    if(i%10==0){
      Serial.print(angle_state.get_bias_gy_x(),4); Serial.print('\t');
      Serial.print(angle_state.get_bias_gy_y(),4); Serial.print('\t');
      Serial.println(angle_state.get_bias_gy_z(),4);
    }
    wait();
  }
  angle_state.set_bias_gy_x(angle_state.get_bias_gy_x()/(float(N)));
  angle_state.set_bias_gy_y(angle_state.get_bias_gy_y()/(float(N)));
  angle_state.set_bias_gy_z(angle_state.get_bias_gy_z()/(float(N)));

  Serial.print(angle_state.get_bias_gy_x(),4); Serial.print('\t');
  Serial.print(angle_state.get_bias_gy_y(),4); Serial.print('\t');
  Serial.println(angle_state.get_bias_gy_z(),4);
  wait();
}

void calibrate_acc_out(){
  uint32_t N = 100000;
  for(uint32_t i=0;i<N;i++){
    mpu6050.get_MPU6050_OUT(&AcX, &AcY, &AcZ, &Tmp, &GyX, &GyY, &GyZ);
    int16_t tempAcX = AcX;
    AcX = -AcY;
    AcY = tempAcX;
    angle_state.calibrate_acc(AcX, AcY, AcZ);
    //Serial.print(millis()); Serial.print('\t');
    if(i%10==0){
      Serial.print(angle_state.get_bias_ac_x(),6); Serial.print('\t');
      Serial.print(angle_state.get_bias_ac_y(),6); Serial.print('\t');
      Serial.println(angle_state.get_bias_ac_z(),6);
    }
    wait();
  }
}

void print_angles(){
  Serial.print(state.phi*180/PI); Serial.print('\t');
  Serial.print(state.th*180/PI); Serial.print('\t');
  Serial.print(state.psi*180/PI); Serial.print('\t');
}

void print_angular_velocity(){
  Serial.print(state.wx*180/PI); Serial.print('\t');
  Serial.print(state.wy*180/PI); Serial.print('\t');
  Serial.print(state.wz*180/PI); Serial.print('\t');
}

void print_rpm(){
  Serial.print(motor1.rpm*60); Serial.print('\t');
  Serial.print(motor2.rpm*60); Serial.print('\t');
  Serial.print(motor3.rpm*60); Serial.print('\t');
  Serial.print(motor4.rpm*60); Serial.print('\t');
}

void print_pwm(){
  Serial.print(motor1.pwm); Serial.print('\t');
  Serial.print(motor2.pwm); Serial.print('\t');
  Serial.print(motor3.pwm); Serial.print('\t');
  Serial.print(motor4.pwm); Serial.print('\t');
}

void drive_motors(){

  if(arm_status==0){
    motor1.pwm = 1000;
    motor2.pwm = 1000;
    motor3.pwm = 1000;
    motor4.pwm = 1000;
  }
  else if(arm_status==1){
    motor1.pwm = 1050;
    motor2.pwm = 1050;
    motor3.pwm = 1050;
    motor4.pwm = 1050;
  }
  else{
    motor1.pwm = math.saturate(motor1.pwm,PWM_MIN+50,PWM_MAX-50);
    motor2.pwm = math.saturate(motor2.pwm,PWM_MIN+50,PWM_MAX-50);
    motor3.pwm = math.saturate(motor3.pwm,PWM_MIN+50,PWM_MAX-50);
    motor4.pwm = math.saturate(motor4.pwm,PWM_MIN+50,PWM_MAX-50);
  }

  pwm.set_pwm4n(motor1.pwm,motor2.pwm,motor3.pwm,motor4.pwm);
}

void disable_GPGSV(){
  //Disable GPGSV megps_serialages
  uint8_t Disable_GPGSV[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 
                               0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39};
  Serial3.write(Disable_GPGSV,16);
  delay(200);
}

void set_to_5Hz(){
  uint8_t Set_to_5Hz[14] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};
  Serial3.write(Set_to_5Hz,14);
  delay(200);
}

void set_to_8Hz(){
  uint8_t Set_to_8Hz[14] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x7D, 0x00, 0x01, 0x00, 0x01, 0x00, 0x93, 0xA8};
  Serial3.write(Set_to_8Hz,14);
  delay(200);
}

void set_to_10Hz(){
  uint8_t Set_to_10Hz[14] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12};
  Serial3.write(Set_to_10Hz,14);
  delay(200);
}

void set_baud_115200(){
  uint8_t baud_115200[28] = {0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0xC2,0x01,0x00,0x07,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0xC0,0x7E};
  Serial3.write(baud_115200,28);
  delay(200);
}

void compensate_delay(){
  for(int i=0;i<NN-1;i++){
    ax_buff[i] = ax_buff[i+1];
    ay_buff[i] = ay_buff[i+1];
    vx_buff[i] = vx_buff[i+1];
    vy_buff[i] = vy_buff[i+1];
  }
  ax_buff[NN-1] = state.ax*981.0; // cm/sec^2
  ay_buff[NN-1] = state.ay*981.0; // cm/sec^2

  acc_index--;

  if((gps_vel_available==1) || (acc_index<0)){
    acc_index = NN/4;
  }

  float vx_gps = vx*0.1; // cm/sec
  float vy_gps = vy*0.1; // cm/sec
  float x_gps = x_nwu; // cm
  float y_gps = y_nwu; // cm

  float ax_sum = 0;
  float ay_sum = 0;

  for(int i=NN-1;i>=acc_index;i--){
    ax_sum += ax_buff[i];
    ay_sum += ay_buff[i];
  }

  state.vx = vx_gps + ax_sum*dt; //cm/sec
  state.vy = vy_gps + ay_sum*dt; //cm/sec

  vx_buff[NN-1] = state.vx; // cm
  vy_buff[NN-1] = state.vy; // cm

  float vx_sum = 0;
  float vy_sum = 0;

  for(int i=NN-1;i>=acc_index;i--){
    vx_sum += vx_buff[i];
    vy_sum += vy_buff[i];
  }

  state.x = x_gps + vx_sum*dt + 0.5*ax_sum*dt*dt; //cm
  state.y = y_gps + vy_sum*dt + 0.5*ay_sum*dt*dt; //cm
}

void tfmp_restore_factory_set(){
  // Save settings
  Serial2.write(0x5A);
  Serial2.write(0x04);
  Serial2.write(0x10);
  Serial2.write(0x6E);
}

void tfmp_set_frame_rate(uint16_t frame_rate){
  // Frame rate (5A 06 03 LL HH SU)
  uint8_t LL = (frame_rate & 0xff);
  uint8_t HH = ((frame_rate>>8) & 0xff);
  uint32_t su = 0x5A + 0x06 + 0x03 + LL + HH;
  uint8_t SU = (su & 0xff);
  
  Serial2.write(0x5A);
  Serial2.write(0x06);
  Serial2.write(0x03);
  Serial2.write(LL);
  Serial2.write(HH);
  Serial2.write(SU);
}

void tfmp_set_output_format(){
  // Output format (Standard 9 bytes(cm))
  Serial2.write(0x5A);
  Serial2.write(0x05);
  Serial2.write(0x05);
  Serial2.write(0x01);
  Serial2.write(0x65);
}

void tfmp_save_settings(){
  // Save settings
  Serial2.write(0x5A);
  Serial2.write(0x04);
  Serial2.write(0x11);
  Serial2.write(0x6F);
}

float get_lidar_data(){
  lidar_data = false;
  lidar_data_valid = false;
  new_data = false;
  
  while(Serial2.available()>0){
    new_data = true;
    for(int i=0;i<8;i++){
      uart[i] = uart[i+1];
    }
    uart[8] = Serial2.read();
  }

  /*for(int i=0;i<9;i++){
    Serial.print(uart[i],HEX); Serial.print('\t');
  }*/
  
  if((new_data == true) && ((uart[0]==HEADER) && (uart[1]==HEADER))){
    lidar_data = true;
    check = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];
    if(uart[8]==(check & 0xff)){
      lidar_data_valid = true;
    }
  }

  if(lidar_data_valid==true){
    //Serial.print(1000000.0/float(micros()-t_lidar)); Serial.print('\t');
    //t_lidar = micros();
    //Serial.println(dist);
    /*Serial.print(dist); Serial.print('\t');
    Serial.print(strength); Serial.print('\t');
    Serial.print(temprature); Serial.print('\t');
    Serial.println();*/
    dist = uart[2] + uart[3]*256;
    strength = uart[4] + uart[5]*256;
    temprature = uart[6] + uart[7]*256;
    temprature = temprature/8 - 256;
    state.z = dist*cos(state.phi)*cos(state.th);
  }

  return (float)dist;
  
  /*Serial.print(dist); Serial.print('\t');
  Serial.print(strength); Serial.print('\t');
  Serial.print(temprature); Serial.print('\t');
  Serial.println();*/
}
