#include "MB_1040_driver.h"

unsigned long MB_1040_driver::t_pre_son = 0;
unsigned long MB_1040_driver::pw_son = 0;

MB_1040_driver::MB_1040_driver(){
}

void MB_1040_driver::initialize_sonar(){
  pinMode(SONAR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SONAR_PIN), isr_sonar, CHANGE);
}

long MB_1040_driver::get_sonar_pw(){
  return pw_son;
}

void MB_1040_driver::isr_sonar(){
  unsigned long t_now_son = micros();  
  if(digitalRead(SONAR_PIN)==1){
    MB_1040_driver::t_pre_son = t_now_son;
  }
  else{
    MB_1040_driver::pw_son = t_now_son - MB_1040_driver::t_pre_son;
  }
}
