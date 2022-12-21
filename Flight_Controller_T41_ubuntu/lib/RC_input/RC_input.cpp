#include "RC_input.h"

RC_input::RC_input(){
      //ppm.initialize_ppm();
}

void RC_input::initialize_rc_ppm(){
  ppm.initialize_ppm();
  
  /*float fc = 2.5;
  float tau = 1.0/(2.0*PI*fc);
  float fs = 50.0;
  float DT = 1.0/fs;
  float alpha = 1 - DT/(DT+tau);*/
  float alpha = 0.7262211; //0.76094; // (3 Hz, 2.5 Hz)

  lpf_ch1.set_alpha_LPF(alpha);
  lpf_ch2.set_alpha_LPF(alpha);
  lpf_ch3.set_alpha_LPF(alpha);
  lpf_ch4.set_alpha_LPF(alpha);
}

void RC_input::int_pilot_cmd(uint8_t* mode_, uint8_t* arm_status_, float* throttle_, float* roll_, float* pitch_, float* yaw_){
  float pw1 = ppm.get_ppm_pw(1);
  float pw2 = ppm.get_ppm_pw(2);
  float pw3 = ppm.get_ppm_pw(3);
  float pw4 = ppm.get_ppm_pw(4);
  float pw5 = ppm.get_ppm_pw(5);
  //float pw6 = ppm.get_ppm_pw(6);
  //float pw7 = ppm.get_ppm_pw(7);
  //float pw8 = ppm.get_ppm_pw(8);

  pw1 = lpf_ch1.apply_LPF(pw1);
  pw2 = lpf_ch2.apply_LPF(pw2);
  pw3 = lpf_ch3.apply_LPF(pw3);
  pw4 = lpf_ch4.apply_LPF(pw4);

  reverse_pw_dirn(&pw1, &pw2, &pw3, &pw4);
  int_mode_cmd(pw5);
  int_arm_cmd(pw3, pw4);
  //Serial.print(mode); Serial.print('\t'); Serial.println(arm_status);
  
  *mode_       = get_mode_cmd();
  *arm_status_ = get_arm_cmd();
  *throttle_   = int_thr_cmd(pw3);
  *roll_       = int_roll_cmd(pw1);
  *pitch_      = int_pitch_cmd(pw2);
  *yaw_        = int_yaw_cmd(pw4);
  
  //Serial.print(*mode_); Serial.print('\t'); Serial.println(*arm_status_);
}

void RC_input::reverse_pw_dirn(float* pw1, float* pw2, float* pw3, float* pw4){
  if(reverse_ch1==1){
    *pw1 = math.linear_map(*pw1, ch1_max, ch1_min, ch1_min, ch1_max);
  }

  if(reverse_ch2==1){
    *pw2 = math.linear_map(*pw2, ch2_max, ch2_min, ch2_min, ch2_max);
  }
  
  if(reverse_ch3==1){
    *pw3 = math.linear_map(*pw3, ch3_max, ch3_min, ch3_min, ch3_max);
  }

  if(reverse_ch4==1){
    *pw4 = math.linear_map(*pw4, ch4_max, ch4_min, ch4_min, ch4_max);
  }
}

void RC_input::int_mode_cmd(float pw){
  if(pw<1244){
    mode = 0; // attitude
  }
  else if(pw>=1244 && pw<1795){
    mode = 1; // height / position hold
  }
  else{
    mode = 2; // autonomous (position hold and position tracking (i.e. waypoint also))
  }
  //Serial.print(pw); Serial.print('\t'); Serial.println(mode);
}

void RC_input::int_arm_cmd(float pw3, float pw4){
  if(mode==0){
    // attitude control
    if(arm_status==0){ //disarmed
      if(pw3<1140 && pw4<1158){ //1936, 1108
        arm_status = 1;
      }
    }
    else if(arm_status==1){ // armed
      if(pw3<1140 && pw4>1886){ // for motor start
        arm_status = 0;
      }
      if(pw3>1160){ // for motor start + setpoint calculation start
        arm_status = 2;
      }
    }
    else if(arm_status==2){
      if(pw3<1140){ // for stepwise transition of arm_status
        arm_status = 1;
      }
    }
  }
  else if(mode==1){
    // autonomous height control
  }
  else if(mode==2){
    // autonomous position control
  }
}

float RC_input::int_thr_cmd(float pw){
  if(pw<ch3_min1){
    return thrust_min;
  }
  else if(pw>=ch3_min1 && pw<ch3_min2){
    return thrust_min1;
  }
  else{
    return math.linear_map(pw, ch3_min2, ch3_max, thrust_min1, thrust_max);
  }
  /*
   else if(pw>=ch3_min2 && pw<ch3_mid){
    return math.linear_map(pw, ch3_min2, ch3_mid, thrust_min1, thrust_mid);
  }
  else{
    return math.linear_map(pw, ch3_mid, ch3_max, thrust_mid, thrust_max);
  }
   */
}

float RC_input::int_roll_cmd(float pw){
  // normalize pw input to [-1,1]
  float pw_norm;
  if(pw>(ch1_mid+ch1_deadband*0.5)){
    pw_norm = math.linear_map(pw, ch1_mid + ch1_deadband*0.5, ch1_max, 0, 1);
  }
  else if(pw<(ch1_mid-ch1_deadband*0.5)){
    pw_norm = math.linear_map(pw, ch1_mid - ch1_deadband*0.5, ch1_min, 0, -1);
  }
  else{
    pw_norm = 0;
  }
  
  // cubic shaping
  return 20*pw_norm*pw_norm*pw_norm + 10*pw_norm;
}

float RC_input::int_pitch_cmd(float pw){
  // normalize pw input to [-1,1]
  float pw_norm;
  if(pw>(ch2_mid+ch2_deadband*0.5)){
    pw_norm = math.linear_map(pw, ch2_mid + ch2_deadband*0.5, ch2_max, 0, 1);
  }
  else if(pw<(ch2_mid-ch2_deadband*0.5)){
    pw_norm = math.linear_map(pw, ch2_mid - ch2_deadband*0.5, ch2_min, 0, -1);
  }
  else{
    pw_norm = 0;
  }
  
  // cubic shaping
  return 20*pw_norm*pw_norm*pw_norm + 10*pw_norm;
}

float RC_input::int_yaw_cmd(float pw){
  // calculate yaw rate, i.e. from pw input to [yaw_rate_min,yaw_rate_max]
  float yaw_rate_des;
  if(pw>(ch4_mid+ch4_deadband*0.5)){
    yaw_rate_des = math.linear_map(pw, ch4_mid + ch4_deadband*0.5, ch4_max, 0, yaw_rate_max);
  }
  else if(pw<(ch4_mid-ch4_deadband*0.5)){
    yaw_rate_des = math.linear_map(pw, ch4_mid - ch4_deadband*0.5, ch4_min, 0, yaw_rate_min);
  }
  else{
    yaw_rate_des = 0;
  }
  
  // integrate yaw rate
  float dt = 0.0025;
  yaw_desired = yaw_desired + yaw_rate_des*dt;
  yaw_desired = math.wrap(yaw_desired, -180, 180);
  return yaw_desired;
}

void RC_input::set_yaw_cmd_angle(float _yaw_desired){
  yaw_desired = _yaw_desired;
}

uint8_t RC_input::get_arm_cmd(){
  return arm_status;
}

uint8_t RC_input::get_mode_cmd(){
  return mode;
}

float RC_input::get_aux1_cmd(){
  return ppm.get_ppm_pw(6);
}

float RC_input::get_aux2_cmd(){
  return ppm.get_ppm_pw(7);
}

float RC_input::get_aux3_cmd(){
  return ppm.get_ppm_pw(8);
}
