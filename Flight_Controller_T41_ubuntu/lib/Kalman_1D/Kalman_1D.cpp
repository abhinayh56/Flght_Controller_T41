#include "Kalman_1D.h"

Kalman_1D::Kalman_1D(){
}

void Kalman_1D::set_dt(float dt_=1.0/400.0){
    dt = dt_;
}

void Kalman_1D::set_sen_res(float res_){
    res = res_;
}

void Kalman_1D::set_input_noise(float ek_=200){
    ek = ek_;
}

void Kalman_1D::init_states(float z_=0, float zd_=0, float zdd_b_=0){
    z = z_;
    zd = zd_;
    zdd_b = zdd_b_;
}

void Kalman_1D::init_states_cov(float p11_=9999, float p22_=9999, float p33_=9999){
    p11 = p11_;
    p22 = p22_;
    p33 = p33_;
}

void Kalman_1D::predict(float az){
    // predicted states
    float z_temp = z + dt*zd + (az*dt*dt)*0.5 - (dt*dt*zdd_b)*0.5;
    float zd_temp = zd + az*dt - dt*zdd_b;
    float zdd_b_temp = zdd_b;

    // first calculate then update for simultaneous update
    z = z_temp;
    zd = zd_temp;
    zdd_b = zdd_b_temp;

    // predicted error covariance
    float p11_temp = (pow(dt,4)*pow(ek,2))*0.25 + (p33*pow(dt,4))*0.25 + p22*pow(dt,2) + p11;
    float p12_temp = (pow(dt,3)*pow(ek,2))*0.5 + (p33*pow(dt,3))*0.5 + p22*dt;
    float p13_temp = -(pow(dt,2)*p33)*0.5;
    float p21_temp = (pow(dt,3)*pow(ek,2))*0.5 + (p33*pow(dt,3))*0.5 + p22*dt;
    float p22_temp =  pow(dt,2)*pow(ek,2) + p33*pow(dt,2) + p22;
    float p23_temp = -dt*p33;
    float p31_temp = -(pow(dt,2)*p33)*0.5;
    float p32_temp = -dt*p33;
    float p33_temp = p33;

    // first calculate then update for simultaneous update
    p11 = p11_temp;
    p12 = p12_temp;
    p13 = p13_temp;
    p21 = p21_temp;
    p22 = p22_temp;
    p23 = p23_temp;
    p31 = p31_temp;
    p32 = p32_temp;
    p33 = p33_temp;
}

void Kalman_1D::correct(float h){
    // innovation / measurement residue
    float Y = h - z;

    // kalman gain
    float K11 = ((pow(dt,4)*pow(ek,2))/4 + (p33*pow(dt,4))/4 + p22*pow(dt,2) + p11)/((pow(dt,4)*pow(ek,2))/4 + (p33*pow(dt,4))/4 + p22*pow(dt,2) + pow(res,2) + p11);
    float K21 = ((pow(dt,3)*pow(ek,2))/2 + (p33*pow(dt,3))/2 + p22*dt)/((pow(dt,4)*pow(ek,2))/4 + (p33*pow(dt,4))/4 + p22*pow(dt,2) + pow(res,2) + p11);
    float K31 = -(pow(dt,2)*p33)/(2*((pow(dt,4)*pow(ek,2))/4 + (p33*pow(dt,4))/4 + p22*pow(dt,2) + pow(res,2) + p11));

    // update state estimate
    float z_temp = z + K11*(h - z);
    float zd_temp = zd + K21*(h - z);
    float zdd_b_temp = zdd_b + K31*(h - z);

    z = z_temp;
    zd = zd_temp;
    zdd_b = zdd_b_temp;

    // update error covariance
    float p11_temp = -(K11 - 1)*((pow(dt,4)*(ek,2))/4 + (p33*pow(dt,4))/4 + p22*pow(dt,2) + p11);
    float p12_temp = -(K11 - 1)*((pow(dt,3)*pow(ek,2))/2 + (p33*pow(dt,3))/2 + p22*dt);
    float p13_temp = (pow(dt,2)*p33*(K11 - 1))/2;
    float p21_temp = dt*p22 - K21*((pow(dt,4)*pow(ek,2))/4 + (p33*pow(dt,4))/4 + p22*pow(dt,2) + p11) + (pow(dt,3)*p33)/2 + (pow(dt,3)*pow(ek,2))/2;
    float p22_temp =  p22 + pow(dt,2)*p33 - K21*((pow(dt,3)*pow(ek,2))/2 + (p33*pow(dt,3))/2 + p22*dt) + pow(dt,2)*pow(ek,2);
    float p23_temp = (K21*p33*pow(dt,2))/2 - p33*dt;
    float p31_temp = - K31*((pow(dt,4)*pow(ek,2))/4 + (p33*pow(dt,4))/4 + p22*pow(dt,2) + p11) - (pow(dt,2)*p33)/2;
    float p32_temp = - dt*p33 - K31*((pow(dt,3)*pow(ek,2))/2 + (p33*pow(dt,3))/2 + p22*dt);
    float p33_temp = (K31*p33*pow(dt,2))/2 + p33;

    // first calculate then update for simultaneous update
    p11 = p11_temp;
    p12 = p12_temp;
    p13 = p13_temp;
    p21 = p21_temp;
    p22 = p22_temp;
    p23 = p23_temp;
    p31 = p31_temp;
    p32 = p32_temp;
    p33 = p33_temp;
}

void Kalman_1D::get_estimates(float* z_, float* zd_, float* zdd_b_){
    *z_ = z;
    *zd_ = zd;
    *zdd_b_ = zdd_b;
}

float Kalman_1D::get_z(){
    return z;
}

float Kalman_1D::get_zd(){
    return zd;
}

float Kalman_1D::get_zdd_b(){
    return zdd_b;
}
