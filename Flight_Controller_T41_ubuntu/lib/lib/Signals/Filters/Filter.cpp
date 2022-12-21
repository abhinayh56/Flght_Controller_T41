#include "Filter.h"

LPF_filter::LPF_filter(){
  start = true;
}

void LPF_filter::set_param(float fc_, float dt_){
  fc = fc_;
  dt = dt_;

  tau = 1.0 / (2.0*math_pi*fc);
  alpha = dt / (dt + tau);
}

void LPF_filter::get_param(float* fc_, float* dt_){
  *fc_ = fc;
  *dt_ = dt;
}

void LPF_filter::set_fc(float fc_){
  fc = fc_;

  tau = 1.0 / (2.0*math_pi*fc);
  alpha = dt / (dt + tau);
}

void LPF_filter::set_tau(float tau_){
  tau = tau_;

  fc = 1.0 / (2.0*math_pi*tau);
  alpha = dt / (dt + tau);
}

void LPF_filter::set_dt(float dt_){
  dt = dt_;
  
  alpha = dt / (dt + tau);
}

void LPF_filter::set_alpha(float alpha_){
  alpha = alpha_;

  tau = ((1.0 - alpha)*dt) / alpha;
  fc = 1.0 / (2.0*math_pi*tau);
}

float LPF_filter::get_fc(){
  return fc;
}

float LPF_filter::get_tau(){
  return tau;
}

float LPF_filter::get_dt(){
  return dt;
}

float LPF_filter::get_alpha(){
  return alpha;
}

float LPF_filter::get_y(){
  return y;
}

void LPF_filter::cal_y(float x){
  if(start==true){
    start = false;
    y = x;
  }
  else{
    y = alpha*x + (1.0 - alpha)*y_pre;
  }
  y_pre = y;
}

void LPF_filter::reset(){
  y_pre = 0.0;
  start = true;
}

/////////////////////////////////////////////////////

HPF_filter::HPF_filter(){
  start = true;
}

void HPF_filter::set_param(float fc_, float dt_){
  fc = fc_;
  dt = dt_;

  tau = 1.0 / (2.0*math_pi*fc);
  alpha = tau / (dt + tau);
}

void HPF_filter::get_param(float* fc_, float* dt_){
  *fc_ = fc;
  *dt_ = dt;
}

void HPF_filter::set_fc(float fc_){
  fc = fc_;

  tau = 1.0 / (2.0*math_pi*fc);
  alpha = tau / (dt + tau);
}

void HPF_filter::set_tau(float tau_){
  tau = tau_;

  fc = 1.0 / (2.0*math_pi*tau);
  alpha = tau / (dt + tau);
}

void HPF_filter::set_dt(float dt_){
  dt = dt_;
  
  alpha = tau / (dt + tau);
}

void HPF_filter::set_alpha(float alpha_){
  alpha = alpha_;

  tau = (alpha/(1.0 - alpha)) * dt;
  fc = 1.0 / (2.0*math_pi*tau);
}

float HPF_filter::get_fc(){
  return fc;
}

float HPF_filter::get_tau(){
  return tau;
}

float HPF_filter::get_dt(){
  return dt;
}

float HPF_filter::get_alpha(){
  return alpha;
}

float HPF_filter::get_y(){
  return y;
}

void HPF_filter::cal_y(float x){
  if(start==true){
    start = false;
    y = x;
  }
  else{
    y = alpha*(y_pre + x - x_pre);
  }
  x_pre = x;
  y_pre = y;
}

void HPF_filter::reset(){
  x_pre = 0.0;
  y_pre = 0.0;
  start = true;
}
