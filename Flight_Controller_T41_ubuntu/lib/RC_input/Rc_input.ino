#include "RC_input.h"
//#include "PPM_driver.h"

uint8_t mode;
uint8_t arm_status;
float throttle;
float roll;
float pitch;
float yaw;

RC_input rc;
//PPM_driver ppm;

void setup(){
  Serial.begin(57600); 
}

void loop(){
  rc.int_pilot_cmd(&mode, &arm_status, &throttle, &roll, &pitch, &yaw);
  //Serial.println(ppm.get_ppm_pw(3));
  Serial.print(throttle); Serial.print('\t');
  Serial.print(roll); Serial.print('\t');
  Serial.print(pitch); Serial.print('\t');
  Serial.print(yaw); Serial.print('\t');
  Serial.print(mode); Serial.print('\t');
  Serial.print(arm_status); Serial.print('\t');
  Serial.print(rc.get_aux1_cmd()); Serial.print('\t');
  Serial.print(rc.get_aux2_cmd()); Serial.print('\t');
  Serial.println(rc.get_aux3_cmd());
   
  delay(10);
}
