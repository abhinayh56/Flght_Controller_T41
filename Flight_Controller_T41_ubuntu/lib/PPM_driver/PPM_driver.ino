#include "PPM_driver.h"

PPM_driver ppm;

void setup(){
  Serial.begin(115200);
  //ppm.initialize_ppm();
}

void loop(){
  for(int i=1;i<9;i++){
    Serial.print(ppm.get_ppm_pw(i)); Serial.print('\t');
  }
  Serial.println();
  delay(10);
}
