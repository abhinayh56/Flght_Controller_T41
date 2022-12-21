#include "PWM_driver.h"

PWM_driver pwm;

void setup(){
  pwm.initialize_PWM(561,612,1578.2,1896.45);
}

void loop(){
  //pwm.set_pwm6n(1101,1201,1301,1401,1501,1601);
  /*pwm.set_pwm1(1100);
  pwm.set_pwm2(1200);
  pwm.set_pwm3(1300);
  pwm.set_pwm4(1400);
  pwm.set_pwm5(1500);
  pwm.set_pwm6(1600);*/
}
