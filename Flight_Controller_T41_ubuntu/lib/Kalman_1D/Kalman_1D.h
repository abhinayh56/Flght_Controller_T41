#ifndef KALMAN_1D
#define KALMAN_1D

#include <math.h>

class Kalman_1D{
    public:
        Kalman_1D();
        void set_dt(float dt_);
        void set_sen_res(float res_);
        void set_input_noise(float ek_=200);
        void init_states(float z_=0, float zd_=0, float zdd_b_=0);
        void init_states_cov(float p11_=9999, float p22_=9999, float p33_=9999);
        void predict(float az);
        void correct(float h);
        void get_estimates(float* z_, float* zd_, float* zdd_b_);
        float get_z();
        float get_zd();
        float get_zdd_b();
        
    private:
        // model parameters
        float dt = 1.0/400.0; // sample time
        float res = 0.25; // niose covariance of measurement for TFM-plus LIDAR (sample covariance from sensor measurement)
        float ek = 200.0; // noise covariance of accelerometer (not standard deviation) (tune it for required performance)

        // input
        float az = 0;

        // measurement
        float h = 0;

        // initialize states
        //X = [z;       --> position
        //    zd;       --> velocity
        //    zdd_b];   --> input / acceleration bias
        float z = 0;
        float zd = 0;
        float zdd_b = 0;

        // initialize state covariance
        // choose it a large value for quick convergence
        //P = [p11   0   0;
        //     0   p22   0;
        //     0     0 p33];
        float p11 = 9999,  p12 = 0,    p13 = 0;
        float p21 = 0,     p22 = 9999, p23 = 0;
        float p31 = 0,     p32 = 0,    p33 = 9999;

};

#endif
