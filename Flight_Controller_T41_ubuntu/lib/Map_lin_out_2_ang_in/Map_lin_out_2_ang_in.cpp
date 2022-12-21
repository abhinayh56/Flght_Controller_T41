#include "Map_lin_out_2_ang_in.h"

Map_lin_out_2_ang_in::Map_lin_out_2_ang_in(){
}

float Map_lin_out_2_ang_in::acxy_2_angle_phi0(float ac_x, float ac_y, float psi){
    return g_inv*(ac_x*sin(psi) - ac_y*cos(psi));
}

float Map_lin_out_2_ang_in::acxy_2_angle_th0(float ac_x, float ac_y, float psi){
    return g_inv*(ac_x*cos(psi) + ac_y*sin(psi));
}
