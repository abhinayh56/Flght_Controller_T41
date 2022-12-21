#include "PID_controller.h"

PID_controller::PID_controller(){
}

void PID_controller::set_param(float Kp_, float Ki_, float Kd_, float dt_, float I_max_, float u_max_){
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
    dt = dt_;
    I_max = I_max_;
    u_max = u_max_;
    d_filter = false;
}

void PID_controller::set_param(float Kp_, float Ki_, float Kd_, float dt_, float I_max_, float u_max_, float fc_){
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
    dt = dt_;
    I_max = I_max_;
    u_max = u_max_;
    fc = fc_;
    d_filter = true;
    lpf.set_param(fc,dt);
}

void PID_controller::get_param(float* Kp_, float* Ki_, float* Kd_, float* dt_, float* I_max_, float* u_max_){
    *Kp_ = Kp;
    *Ki_ = Ki;
    *Kd_ = Kd;
    *dt_ = dt;
    *I_max_ = I_max;
    *u_max_ = u_max;
}

void PID_controller::get_param(float* Kp_, float* Ki_, float* Kd_, float* dt_, float* I_max_, float* u_max_, float* fc_){
    *Kp_ = Kp;
    *Ki_ = Ki;
    *Kd_ = Kd;
    *dt_ = dt;
    *I_max_ = I_max;
    *u_max_ = u_max;
    *fc_ = fc;
}

void PID_controller::set_Kp(float Kp_){
    Kp = Kp_;
}

void PID_controller::set_Ki(float Ki_){
    Ki = Ki_;
}

void PID_controller::set_Kd(float Kd_){
    Kd = Kd_;
}

void PID_controller::set_dt(float dt_){
    dt = dt_;
}

void PID_controller::set_I_max(float I_max_){
    I_max = I_max_;
}

void PID_controller::set_u_max(float u_max_){
    u_max = u_max_;
}

void PID_controller::set_fc(float fc_){
    fc = fc_;
}

void PID_controller::set_d_filter(bool d_filter_){
    d_filter = d_filter_;
}

float PID_controller::get_Kp(){
    return Kp;
}

float PID_controller::get_Ki(){
    return Ki;
}

float PID_controller::get_Kd(){
    return Kd;
}

float PID_controller::get_dt(){
    return dt;
}

float PID_controller::get_I_max(){
    return I_max;
}

float PID_controller::get_u_max(){
    return u_max;
}

float PID_controller::get_fc(){
    return fc;
}

bool PID_controller::get_d_filter(){
    return d_filter;
}

float PID_controller::get_P(){
    return P;
}

float PID_controller::get_I(){
    return I;
}

float PID_controller::get_D(){
    return D;
}

float PID_controller::get_u(){
    return u;
}

float PID_controller::cal_u(float x0, float x, bool d_filter_=false){
    float e = x0 - x;
    P = Kp*e;
    I = I + Ki*e*dt;
    I = math_fun.saturate(I,-I_max,I_max);
    D = Kd*(e - e_pre)/dt;
    if(d_filter_==true){
        lpf.cal_y(D);
        D = lpf.get_y();
    }
    e_pre = e;
    u = P + I + D;
    u = math_fun.saturate(u,-u_max,u_max);
}

void PID_controller::reset(){
    e_pre = 0.0;
    P = 0.0;
    I = 0.0;
    D = 0.0;
    u = 0.0;
}