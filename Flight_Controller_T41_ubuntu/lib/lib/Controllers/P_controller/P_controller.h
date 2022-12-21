#ifndef P_CONTROLLER
#define P_CONTROLLER

#include "Math_functions.h"

class P_controller{
    public:
        P_controller();
        
        void set_param(float Kp_, float u_max_);
        void get_param(float* Kp_, float* u_max_);

        void set_Kp(float Kp_);
        void set_u_max(float u_max_);

        float get_Kp();
        float get_u_max();

        float get_u(float x0, float x);

    private:
        Math_functions math_fun;
        float Kp = 0.0;
        float u_max = 0.0;
};

#endif