#include "I2Cdev.h"
#include "MS5611_driver.h"

I2Cdev i2c_master;
MS5611_driver bar;

float SampleFrequency = 100, dt = 1/SampleFrequency, loop_timer = 1000000*dt, t;
float tn;

void setup(){
  Serial.begin(115200);
  
  i2c_master.initialize();

  //Serial.println("---------------------------");
  //Serial.println("0. Initializing test!");
  //Serial.println(TWBR);

  //Serial.println("1. bar initializing!");
  bar.initialize();
  delay(100);
  //Serial.println("2. bar initialized!");

  //Serial.println("3. reading calibration!");
  bar.read_calibration_data();
  delay(100);
  //Serial.println("4. reading calibration read successful!");

  /*Serial.println("5. Calibration data:");
  Serial.println(bar.get_C1());
  Serial.println(bar.get_C2());
  Serial.println(bar.get_C3());
  Serial.println(bar.get_C4());
  Serial.println(bar.get_C5());
  Serial.println(bar.get_C6());
  Serial.println(bar.get_OSR());*/
  bar.set_OSR(4096);
  //Serial.println(bar.get_OSR());

  t = micros();
}
double h = 0;

void loop(){
  bar.request_raw_pressure();
  delay(10);
  bar.get_raw_pressure();
  //Serial.println(bar.get_raw_pressure());
  //Serial.print('\t');
  
  bar.request_raw_temperature();
  delay(10);
  bar.get_raw_temperature();
  //Serial.println(bar.get_raw_temperature());

  bar.cal_temp();
  bar.cal_temp_compensated_pressure();

  double temp = bar.get_temp_C();
  double pressure = bar.get_pressure_mbar();
  
  //Serial.print(temp);
  //Serial.print('\t');
  //Serial.print(pressure*100);

  // height calculation
  double a = 0.65/100;
  double T0 = 20.42 + 273.15;
  double P0 = 98994.75;
  double P1 = pressure*100;
  double R = 8.314;
  double M = 0.02896;
  double g = 9.80620;
  h =0.75*h + 0.25*( (T0/a)*(1 - pow(P1/P0, (R*a)/(M*g) )));
  //Serial.print('\t');
  Serial.println(h*100);
  
  tn = t;
  //Serial.print(1000000.0/(micros()-tn)); Serial.print("\t");
  wait();
  //Serial.println(1000000.0/(micros()-tn));
}

void wait(){
  while(micros()-t<loop_timer){}
  t = micros();
}
