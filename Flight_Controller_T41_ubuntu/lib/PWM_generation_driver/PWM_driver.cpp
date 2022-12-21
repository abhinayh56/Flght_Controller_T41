#include "PWM_driver.h"

PWM_driver::PWM_driver(){
}

void PWM_driver::initialize_PWM(float pwm1, float pwm2, float pwm3, float pwm4){
  pinMode(motor1,OUTPUT);
  pinMode(motor2,OUTPUT);
  pinMode(motor3,OUTPUT);
  pinMode(motor4,OUTPUT);

  analogWriteFrequency(motor1,PWM_frequency);
  analogWriteFrequency(motor2,PWM_frequency);
  analogWriteFrequency(motor3,PWM_frequency);
  analogWriteFrequency(motor4,PWM_frequency);

  analogWriteResolution(PWM_resolution);

  set_pwm4n(1000.0,1000.0,1000.0,1000.0);
}

void PWM_driver::set_pwm4n(float pwm1, float pwm2, float pwm3, float pwm4){
  analogWriteResolution(PWM_resolution);
  analogWrite(motor1,pwm_to_bit_val(pwm1));
  analogWrite(motor2,pwm_to_bit_val(pwm2));
  analogWrite(motor3,pwm_to_bit_val(pwm3));
  analogWrite(motor4,pwm_to_bit_val(pwm4));
  analogWriteResolution(PWM_resolution);
}

void PWM_driver::set_pwm1(float pwm1){
  analogWriteResolution(PWM_resolution);
  analogWrite(motor1,pwm_to_bit_val(pwm1));
  analogWriteResolution(PWM_resolution);
}

void PWM_driver::set_pwm2(float pwm2){
  analogWriteResolution(PWM_resolution);
  analogWrite(motor2,pwm_to_bit_val(pwm2));
  analogWriteResolution(PWM_resolution);
}

void PWM_driver::set_pwm3(float pwm3){
  analogWriteResolution(PWM_resolution);
  analogWrite(motor3,pwm_to_bit_val(pwm3));
  analogWriteResolution(PWM_resolution);
}

void PWM_driver::set_pwm4(float pwm4){
  analogWriteResolution(PWM_resolution);
  analogWrite(motor4,pwm_to_bit_val(pwm4));
  analogWriteResolution(PWM_resolution);
}

int PWM_driver::pwm_to_bit_val(float pwm){
  return 13.1072*pwm; //(32768.0/2500.0)*pwm; // ((2^PWM_resolution) / (1,000,000/PWM_frequency)) * pwm
}
