#ifndef BAROMETRIC_HEIGHT
#define BAROMETRIC_HEIGHT

#include <math.h>

class Barometric_height{
    public:
        Barometric_height();

        void set_T0(double T0_);
        void set_P0(double P0_);
        void set_z0(double h0_);
        void set_R(double R_);
        void set_M(double M_);
        void set_g(double g_);
        void set_a(double a_);

        double get_T0();
        double get_P0();
        double get_z0();
        double get_R();
        double get_M();
        double get_g();
        double get_a();

        double get_barometric_height1(double P1);
        double get_barometric_height2(double P1);

    private:
        double z0 = 0;
        double T0 = 32.19 + 273.15;
        double P0 = 974.47; // in mbar
        double R  = 8.31446261815324;
        double M  = 0.0289647;
        double g  = 9.80665;
        double a  = 0.00649; //0.0065616798 (2 deg C/1000 feet upto 36000feet i.e. 11 Km)
        double P1 = 0;
        double h  = 0;
};

#endif
