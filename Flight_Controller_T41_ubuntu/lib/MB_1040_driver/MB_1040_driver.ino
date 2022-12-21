#include "MB_1040_driver.h"
//#include "PPM_driver.h"

MB_1040_driver son;
//PPM_driver ppm;

void setup(){
  Serial.begin(57600); 
  //son.initialize_sonar();
  //ppm.initialize_ppm(); 
}

void loop(){
  Serial.print(((float)son.get_sonar_pw()/147.0)*2.54); Serial.print('\t');
  /*for(int i=1;i<9;i++){
    Serial.print(ppm.get_ppm_pw(i)); Serial.print('\t');
  }*/
  Serial.println();
  delay(10);
}
