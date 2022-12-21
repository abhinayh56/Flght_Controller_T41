#ifndef PPM_DRIVER
#define PPM_DRIVER

#include <Arduino.h>

#define PPM_PIN 23

#define PPM_PW_MAX 4000 //10916
#define NO_CH_IN_PPM 8

class PPM_driver{
  public:
    PPM_driver();
    void initialize_ppm();
    long get_ppm_pw(int ch_i);
  
  private:
    static void isr_ppm();
    static long ch_ppm[8]; //10916, 1540, 1520, 1816, 1520, 1520, 1448, 1520, 1520
    static uint8_t ch_no_ppm;
    static long pw_ppm;
    static long t_pre_ppm;
};
#endif
