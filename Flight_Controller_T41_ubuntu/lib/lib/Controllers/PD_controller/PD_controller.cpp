#include "PD_controller.h"

PD_controller::PD_controller(){
}

void PD_controller::set_param(float Kp_, float Kd_, float dt_, float u_max_){
    Kp = Kp_;
    Kd = Kd_;
    dt = dt_;
    u_max = u_max_;
    d_filter = false;
}

void PD_controller::set_param(float Kp_, float Kd_, float dt_, float u_max_, float fc_){
    Kp = Kp_;
    Kd = Kd_;
    dt = dt_;
    u_max = u_max_;
    fc = fc_;
    d_filter = true;
    lpf.set_param(fc,dt);
}

void PD_controller::get_param(float* Kp_, float* Kd_, float* dt_, float* u_max_){
    *Kp_ = Kp;
    *Kd_ = Kd;
    *dt_ = dt;
    *u_max_ = u_max;
}

void PD_controller::get_param(float* Kp_, float* Kd_, float* dt_, float* u_max_, float* fc_){
    *Kp_ = Kp;
    *Kd_ = Kd;
    *dt_ = dt;
    *u_max_ = u_max;
    *fc_ = fc;
}

void PD_controller::set_Kp(float Kp_){
    Kp = Kp_;
}

void PD_controller::set_Kd(float Kd_){
    Kd = Kd_;
}

void PD_controller::set_dt(float dt_){
    dt = dt_;
}

void PD_controller::set_u_max(float u_max_){
    u_max = u_max_;
}

void PD_controller::set_fc(float fc_){
    fc = fc_;
}

void PD_controller::set_d_filter(bool d_filter_){
    d_filter = d_filter_;
}

float PD_controller::get_Kp(){
    return Kp;
}

float PD_controller::get_Kd(){
    return Kd;
}

float PD_controller::get_dt(){
    return dt;
}

float PD_controller::get_u_max(){
    return u_max;
}

float PD_controller::get_fc(){
    return fc;
}

bool PD_controller::get_d_filter(){
    return d_filter;
}

float PD_controller::get_P(){
    return P;
}

float PD_controller::get_D(){
    return D;
}

float PD_controller::get_u(){
    return u;
}

float PD_controller::cal_u(float x0, float x, bool d_filter_=false){
    float e = x0 - x;
    P = Kp*e;
    D = Kd*(e - e_pre)/dt;
    if(d_filter_==true){
        lpf.cal_y(D);
        D = lpf.get_y();
    }
    e_pre = e;
    u = P + D;
    u = math_fun.saturate(u,-u_max,u_max);
}

void PD_controller::reset(){
    e_pre = 0.0;
    P = 0.0;
    D = 0.0;
    u = 0.0;
}