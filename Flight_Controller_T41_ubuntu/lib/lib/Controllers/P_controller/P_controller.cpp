#include "P_controller.h"

P_controller::P_controller(){
}

void P_controller::set_param(float Kp_, float u_max_){
    Kp = Kp_;
    u_max = u_max_;
}

void P_controller::get_param(float* Kp_, float* u_max_){
    *Kp_ = Kp;
    *u_max_ = u_max;
}

void P_controller::set_Kp(float Kp_){
    Kp = Kp_;
}

void P_controller::set_u_max(float u_max_){
    u_max = u_max_;
}

float P_controller::get_Kp(){
    return Kp;
}

float P_controller::get_u_max(){
    return u_max;
}

float P_controller::get_u(float x0, float x){
    float u = Kp*(x0-x);
    u = math_fun.saturate(u,-u_max,u_max);
    return u;
}