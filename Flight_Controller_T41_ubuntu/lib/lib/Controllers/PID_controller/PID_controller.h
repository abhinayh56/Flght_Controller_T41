#ifndef PID_CONTROLLER
#define PID_CONTROLLER

#include "Math_functions.h"
#include "Filter.h"

class PID_controller{
    public:
        PID_controller();

        void set_param(float Kp_, float Ki_, float Kd_, float dt_, float I_max_, float u_max_);
        void set_param(float Kp_, float Ki_, float Kd_, float dt_, float I_max_, float u_max_, float fc_);

        void get_param(float* Kp_, float* Ki_, float* Kd_, float* dt_, float* I_max_, float* u_max_);
        void get_param(float* Kp_, float* Ki_, float* Kd_, float* dt_, float* I_max_, float* u_max_, float* fc_);

        void set_Kp(float Kp_);
        void set_Ki(float Ki_);
        void set_Kd(float Kd_);
        void set_dt(float dt_);
        void set_I_max(float I_max_);
        void set_u_max(float u_max_);
        void set_fc(float fc_);
        void set_d_filter(bool d_filter_);

        float get_Kp();
        float get_Ki();
        float get_Kd();
        float get_dt();
        float get_I_max();
        float get_u_max();
        float get_fc();
        bool  get_d_filter();

        float get_P();
        float get_I();
        float get_D();
        float get_u();

        float cal_u(float x0, float x, bool d_filter_=false);
        void reset();

    private:
        Math_functions math_fun;
        LPF_filter lpf;

        bool d_filter = false;

        float Kp = 0.0;
        float Ki = 0.0;
        float Kd = 0.0;
        float dt = 0.0;
        float I_max = 0.0;
        float u_max = 0.0;
        float fc = 0.0;
        
        float e_pre = 0.0;
        float P = 0.0;
        float I = 0.0;
        float D = 0.0;
        float u = 0.0;
};

#endif