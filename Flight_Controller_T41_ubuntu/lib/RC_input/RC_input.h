#ifndef RC_INPUT
#define RC_INPUT

#include "PPM_driver.h"
#include "Math_functions.h"
#include "Filter.h"

class RC_input{
  public:
    RC_input();
    void initialize_rc_ppm();
    void int_pilot_cmd(uint8_t* mode, uint8_t* arm_status, float* throttle, float* roll, float* pitch, float* yaw);
    float get_thr_cmd();
    float get_roll_cmd();
    float get_pitch_cmd();
    float get_yaw_cmd();
    void set_yaw_cmd_angle(float _yaw_desired);
    uint8_t get_mode_cmd();
    uint8_t get_arm_cmd();
    float get_aux1_cmd();
    float get_aux2_cmd();
    float get_aux3_cmd();

  private:
    PPM_driver ppm;
    Math_functions math;
    Filter_LP lpf_ch1;
    Filter_LP lpf_ch2;
    Filter_LP lpf_ch3;
    Filter_LP lpf_ch4;

    // roll
    // channel input
    uint16_t ch1_min = 1108;
    uint16_t ch1_mid = 1518;
    uint16_t ch1_max = 1936;
    // setpoint parameters condition
    uint16_t ch1_deadband = 40;
    float roll_angle_min = -30; // degree
    float roll_angle_max = 30;
        
    // pitch
    // channel input
    uint16_t ch2_min = 1108;
    uint16_t ch2_mid = 1518;
    uint16_t ch2_max = 1936;
    // setpoint parameters condition
    uint16_t ch2_deadband = 40;
    float pitch_angle_min = -30; // degree
    float pitch_angle_max = 30;

    // throttle
    // channel input
    uint16_t ch3_min  = 1108;
    uint16_t ch3_min1 = 1158;
    uint16_t ch3_min2 = 1198;
    uint16_t ch3_mid  = 1644;
    uint16_t ch3_max  = 1936;
    // setpoint parameters condition
    float thrust_min  = 0;
    float thrust_min1 = 0.49;
    float thrust_mid  = 11.772;
    float thrust_max  = 25; // Newton

    // yaw
    // channel input
    uint16_t ch4_min = 1108;
    uint16_t ch4_mid = 1518;
    uint16_t ch4_max = 1936;
    // setpoint parameters condition
    uint16_t ch4_deadband = 40;
    float yaw_rate_min = -200; // degree/sec
    float yaw_rate_max = 200;
    float yaw_desired = 0;

    // arming and mode
    uint8_t arm_status = 0;
    uint8_t mode = 0;

    void  reverse_pw_dirn(float* pw1, float* pw2, float* pw3, float* pw4);
    void  int_mode_cmd(float pw);
    void  int_arm_cmd(float ch3, float ch4);
    float int_thr_cmd(float pw);
    float int_roll_cmd(float pw);
    float int_pitch_cmd(float pw);
    float int_yaw_cmd(float pw);
    float int_aux1_cmd(float pw);
    float int_aux2_cmd(float pw);
    float int_aux3_cmd(float pw);

    bool reverse_ch1 = 0; // 1 for no reverse, 0 for reverse
    bool reverse_ch2 = 1;
    bool reverse_ch3 = 1;
    bool reverse_ch4 = 1;
    bool reverse_ch5 = 0;
    bool reverse_ch6 = 0;
    bool reverse_ch7 = 0;
    bool reverse_ch8 = 0;
};

#endif
