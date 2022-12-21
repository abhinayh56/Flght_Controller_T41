#include "I2Cdev.h"
#include "HMC5883L_driver.h"

I2Cdev i2c_master;

HMC5883L_driver mag;

float SampleFrequency = 100, dt = 1/SampleFrequency, loop_timer = 1000000*dt, t;
float tn;

int16_t MagX, MagY, MagZ;

void setup(){
  Serial.begin(57600);
  
  i2c_master.initialize();

  Serial.println("---------------------------");
  Serial.println("Initializing test!");
  
  Serial.println(TWBR);

  mag.initialize();

  Serial.println("MD:");
  Serial.println(mag.get_MD());
  mag.set_MD(0); // 0 to 1
  Serial.println(mag.get_MD());
  
  Serial.println("MA:");
  Serial.println(mag.get_MA());
  //mag.set_MA(0); //0,1,2,3:1,2,4,8
  Serial.println(mag.get_MA());

  Serial.println("DO:");
  Serial.println(mag.get_DO());
  mag.set_DO(6); //0 to 6 75 Hz
  Serial.println(mag.get_DO());

  Serial.println("MS:");
  Serial.println(mag.get_MS());
  mag.set_MS(0); // 0 to 3
  Serial.println(mag.get_MS());

  Serial.println("GN:");
  Serial.println(mag.get_GN());
  //mag.set_GN(1); // 0 to 8
  Serial.println(mag.get_GN());

  t = micros();
}

void loop(){
  mag.get_MAG_OUT(&MagX, &MagY, &MagZ);
  MagX = MagX;
  MagY = -MagY;
  MagZ = -MagZ;

  //print_raw_mag_data();
  print_calib_mag_data();

  //tn = t;
  //Serial.print(1000000.0/(micros()-tn)); Serial.print("\t");
  wait();
  //Serial.print(1000000.0/(micros()-tn)); Serial.print("\t");
}

void print_raw_mag_data()
{
  Serial.print(MagX); Serial.print('\t'); //445,-342
  Serial.print(MagY); Serial.print('\t'); //200, -596
  Serial.println(MagZ);
}

void print_calib_mag_data()
{
  float magX = ((float)MagX - (445.0-342.0)/2.0)/(4.45+3.42);
  float magY = ((float)MagY - (200.0-596.0)/2.0)/(2.00+5.96);
  
  //Serial.print(magX); Serial.print('\t'); //445,-342
  //Serial.print(magY); Serial.print('\t'); //200, -596
  Serial.println(atan2(-magY,magX)*180.0/PI);
}

void wait(){
  while(micros()-t<loop_timer){}
  t = micros();
}
