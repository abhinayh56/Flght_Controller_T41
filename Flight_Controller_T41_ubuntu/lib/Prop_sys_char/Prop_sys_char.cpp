#include "Prop_sys_char.h"

Prop_sys_char::Prop_sys_char(){
}

float Prop_sys_char::rotor_speed_to_pwm(float w, float v){ // (rps, volt) to us
  return a2*w*w + b2*v*v + a1b1*w*v + a1*w + b1*v + c0;
}
