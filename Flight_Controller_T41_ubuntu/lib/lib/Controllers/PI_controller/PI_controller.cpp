#include "PI_controller.h"

PI_controller::PI_controller(){
}

void PI_controller::set_param(float Kp_, float Ki_, float dt_, float I_max_, float u_max_){
    Kp = Kp_;
    Ki = Ki_;
    dt = dt_;
    I_max = I_max_;
    u_max = u_max_;
}

void PI_controller::get_param(float* Kp_, float* Ki_, float* dt_, float* I_max_, float* u_max_){
    *Kp_ = Kp;
    *Ki_ = Ki;
    *dt_ = dt;
    *I_max_ = I_max;
    *u_max_ = u_max;
}

void PI_controller::set_Kp(float Kp_){
    Kp = Kp_;
}

void PI_controller::set_Ki(float Ki_){
    Ki = Ki_;
}

void PI_controller::set_dt(float dt_){
    dt = dt_;
}

void PI_controller::set_I_max(float I_max_){
    I_max = I_max_;
}

void PI_controller::set_u_max(float u_max_){
    u_max = u_max_;
}

float PI_controller::get_Kp(){
    return Kp;
}

float PI_controller::get_Ki(){
    return Ki;
}

float PI_controller::get_dt(){
    return dt;
}

float PI_controller::get_I_max(){
    return I_max;
}

float PI_controller::get_u_max(){
    return u_max;
}

float PI_controller::get_P(){
    return P;
}

float PI_controller::get_I(){
    return I;
}

float PI_controller::get_u(){
    return u;
}

void PI_controller::cal_u(float x0, float x){
    float e = x0 - x;
    P = Kp*e;
    I = I + Ki*e*dt;
    I = math_fun.saturate(I,-I_max,I_max);
    u = P + I;
    u = math_fun.saturate(u,-u_max,u_max);
}

void PI_controller::reset(){
    P = 0.0;
    I = 0.0;
    u = 0.0;
}