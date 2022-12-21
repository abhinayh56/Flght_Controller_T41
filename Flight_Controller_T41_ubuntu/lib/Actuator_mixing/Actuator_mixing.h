#ifndef ACTUATOR_MIXING
#define ACTUATOR_MIXING

#include <math.h>

class Actuator_mixing{
    // for quadrotor using Teensy 4.1
    public:
        Actuator_mixing();
        void calc_U_to_rpm(float u1, float u2, float u3, float u4); // (gm , gm-cm, gm-cm, gm-cm) to rps
        float get_rpm1();
        float get_rpm2();
        float get_rpm3();
        float get_rpm4();
        void get_rpm1234(float* _rpm1, float* _rpm2, float* _rpm3, float* _rpm4);

    private:
        float a_m_inv = 4.582259535160579;
        float b_m_inv = 0.288013048041668;
        float c_m_inv = 1.198175065365971;

        float rpm1 = 0;
        float rpm2 = 0;
        float rpm3 = 0;
        float rpm4 = 0;
};

#endif
