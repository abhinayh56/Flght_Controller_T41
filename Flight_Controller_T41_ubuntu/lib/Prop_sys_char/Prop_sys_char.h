#ifndef PROP_SYS_CAHR
#define PROP_SYS_CHAR

class Prop_sys_char{
    public:
        Prop_sys_char();
        float rotor_speed_to_pwm(float w, float v); // (rps, volt) to us

    private:
        // Rotor rpm to pwm variables
        float a2   = 0.039050514422000;
        float b2   = 3.513433765980000;
        float a1b1 = -0.711422633380000;
        float a1   = 11.499308216548000;
        float b1   = -72.304474753421999;
        float c0   = 1450.850740639114;
};

#endif
