#include "Actuator_mixing.h"

Actuator_mixing::Actuator_mixing(){
}

void Actuator_mixing::calc_U_to_rpm(float u1, float u2, float u3, float u4){ // (gm , gm-cm, gm-cm, gm-cm) to rps
    rpm1 = a_m_inv*u1 - b_m_inv*u2 - b_m_inv*u3 - c_m_inv*u4; // rpm is in rps
    rpm2 = a_m_inv*u1 - b_m_inv*u2 + b_m_inv*u3 + c_m_inv*u4;
    rpm3 = a_m_inv*u1 + b_m_inv*u2 + b_m_inv*u3 - c_m_inv*u4;
    rpm4 = a_m_inv*u1 + b_m_inv*u2 - b_m_inv*u3 + c_m_inv*u4;

    if(rpm1<0) rpm1 = 0;
    if(rpm2<0) rpm2 = 0;
    if(rpm3<0) rpm3 = 0;
    if(rpm4<0) rpm4 = 0;

    rpm1 = sqrt(rpm1);
    rpm2 = sqrt(rpm2);
    rpm3 = sqrt(rpm3);
    rpm4 = sqrt(rpm4);
}

float Actuator_mixing::get_rpm1(){
    return rpm1;
}

float Actuator_mixing::get_rpm2(){
    return rpm2;
}

float Actuator_mixing::get_rpm3(){
    return rpm3;
}

float Actuator_mixing::get_rpm4(){
    return rpm4;
}

void Actuator_mixing::get_rpm1234(float* _rpm1, float* _rpm2, float* _rpm3, float* _rpm4){
    *_rpm1 = rpm1;
    *_rpm2 = rpm2;
    *_rpm3 = rpm3;
    *_rpm4 = rpm4;
}
