#ifndef PWM_DRIVER
#define PWM_DRIVER

#include <Arduino.h>

class PWM_driver{
  public:
    PWM_driver();
    void initialize_PWM(float pwm1=1000, float pwm2=1000, float pwm3=1000, float pwm4=1000);
    void set_pwm4n(float pwm1=1000, float pwm2=1000, float pwm3=1000, float pwm4=1000);
    void set_pwm1(float pwm1);
    void set_pwm2(float pwm2);
    void set_pwm3(float pwm3);
    void set_pwm4(float pwm4);

    private:
      uint8_t motor1 = 2;
      uint8_t motor2 = 3;
      uint8_t motor3 = 5;
      uint8_t motor4 = 6;

      float PWM_frequency = 400; // in Hz (i.e. T = 2500 us)
      uint32_t PWM_resolution = 15; // 32768

      int pwm_to_bit_val(float);
};

//analogWriteResolution(12);
//analogWrite(pin, duty);

#endif
