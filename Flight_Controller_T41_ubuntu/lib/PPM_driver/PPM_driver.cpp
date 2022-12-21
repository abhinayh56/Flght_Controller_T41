#include "PPM_driver.h"

long PPM_driver::t_pre_ppm = 0;
long PPM_driver::pw_ppm = 0;

long PPM_driver::ch_ppm[8] = {0,0,0,0,0,0,0,0};
uint8_t PPM_driver::ch_no_ppm = 0;

PPM_driver::PPM_driver(){
}

void PPM_driver::initialize_ppm(){
  pinMode(PPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), isr_ppm, RISING);
}

long PPM_driver::get_ppm_pw(int ch_i){
  return ch_ppm[ch_i-1];
}

void PPM_driver::isr_ppm(){
  unsigned long t_now_ppm = micros();
  PPM_driver::pw_ppm = t_now_ppm - PPM_driver::t_pre_ppm;
  PPM_driver::t_pre_ppm = t_now_ppm;
  
  if((PPM_driver::pw_ppm>PPM_PW_MAX) || (PPM_driver::ch_no_ppm==NO_CH_IN_PPM)){
    PPM_driver::ch_no_ppm = 0;
  }
  else{
    PPM_driver::ch_ppm[PPM_driver::ch_no_ppm] = PPM_driver::pw_ppm;
    PPM_driver::ch_no_ppm++;
  }
}
