#ifndef MAP_LIN_OUT_2_ANG_IN
#define MAP_LIN_OUT_2_ANG_IN

#include <math.h>

class Map_lin_out_2_ang_in{
    public:
        Map_lin_out_2_ang_in();
        float acxy_2_angle_phi0(float ac_x, float ac_y, float psi);
        float acxy_2_angle_th0(float ac_x, float ac_y, float psi);
    
    private:
        float g = 9.81;
        float g_inv = 0.10193679918450560652395514780836;
};

#endif
