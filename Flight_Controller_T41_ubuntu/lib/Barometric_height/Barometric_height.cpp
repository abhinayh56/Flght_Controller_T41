#include "Barometric_height.h"

Barometric_height::Barometric_height(){}

void Barometric_height::set_T0(double T0_){
    T0 = T0_;
}

void Barometric_height::set_P0(double P0_){
    P0 = P0_;
}

void Barometric_height::set_z0(double z0_){
    z0 = z0_;
}

void Barometric_height::set_R(double R_){
    R = R_;
}

void Barometric_height::set_M(double M_){
    M = M_;
}

void Barometric_height::set_g(double g_){
    g = g_;
}

void Barometric_height::set_a(double a_){
    a = a_;
}

double Barometric_height::get_T0(){
    return T0;
}

double Barometric_height::get_P0(){
    return P0;
}

double Barometric_height::get_z0(){
    return z0;
}

double Barometric_height::get_R(){
    return R;
}

double Barometric_height::get_M(){
    return M;
}

double Barometric_height::get_g(){
    return g;
}

double Barometric_height::get_a(){
    return a;
}

/*
Note: Barometric height function used here is just the thickness of atmospleric layer between two points
having pressure P0, temperature T0 and pressure P1 and temperature following linear variation from T0 with
slope a;
*/
double Barometric_height::get_barometric_height1(double P1){
    double z_temp = z0 + (R*T0)/(M*g)*log(P0/P1);
    //if(z_temp==isinf) x_temp=0;
    return z_temp;
}

double Barometric_height::get_barometric_height2(double P1){
    double z_temp = z0 + (T0/a) * (1.0 - pow(P1/P0, (R*a)/(M*g)));
    return z_temp;
}
