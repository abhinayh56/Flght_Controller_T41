#ifndef PI_CONTROLLER
#define PI_CONTROLLER

#include "Math_functions.h"

class PI_controller{
    public:
        PI_controller();

        void set_param(float Kp_, float Ki_, float dt_, float I_max_, float u_max_);
        void get_param(float* Kp_, float* Ki_, float* dt_, float* I_max_, float* u_max_);

        void set_Kp(float Kp_);
        void set_Ki(float Ki_);
        void set_dt(float dt_);
        void set_I_max(float I_max_);
        void set_u_max(float u_max_);

        float get_Kp();
        float get_Ki();
        float get_dt();
        float get_I_max();
        float get_u_max();

        float get_P();
        float get_I();
        float get_u();

        void cal_u(float x0, float x);
        void reset();

    private:
        Math_functions math_fun;
        
        float Kp = 0.0;
        float Ki = 0.0;
        float dt = 0.0;
        float I_max = 0.0;
        float u_max = 0.0;
        
        float P = 0.0;
        float I = 0.0;
        float u = 0.0;
};

#endif