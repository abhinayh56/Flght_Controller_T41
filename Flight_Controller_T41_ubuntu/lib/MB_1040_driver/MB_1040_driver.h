#ifndef MB_1040_DRIVER
#define MB_1040_DRIVER

#include <Arduino.h>

#define SONAR_PIN 33

class MB_1040_driver{
  public:
    MB_1040_driver();
    void initialize_sonar();
    long get_sonar_pw();
      
  private:
    static void isr_sonar();
    static unsigned long t_pre_son;
    static unsigned long pw_son;
};
#endif
