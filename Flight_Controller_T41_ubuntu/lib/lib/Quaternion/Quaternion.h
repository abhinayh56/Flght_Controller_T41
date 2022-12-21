#ifndef QUATERNION
#define QUATERNION

#include "math.h"

class Quat{
    public:
        Quat();
        Quat(double q0_, double q1_, double q2_, double q3_){
            q0 = q0_;
            q1 = q1_;
            q2 = q2_;
            q3 = q3_;
        }
        double q0;
        double q1;
        double q2;
        double q3;
    private:
};

class Quat_alg{
    public:
        Quat_alg();
        
        Quat add(Quat p, Quat q){
            Quat s;
            s.q0 = p.q0 + q.q0;
            s.q1 = p.q1 + q.q1;
            s.q2 = p.q2 + q.q2;
            s.q3 = p.q3 + q.q3;
            return s;
        }

        Quat sub(Quat p, Quat q){
            Quat s;
            s.q0 = p.q0 - q.q0;
            s.q1 = p.q1 - q.q1;
            s.q2 = p.q2 - q.q2;
            s.q3 = p.q3 - q.q3;
            return s;
        }

        Quat mul(Quat q, double k){
            Quat s;
            s.q0 = k * q.q0;
            s.q1 = k * q.q1;
            s.q2 = k * q.q2;
            s.q3 = k * q.q3;
            return s;
        }

        Quat div(Quat q, double k){
            Quat s;
            double d = 1.0/k;
            s.q0 = d * q.q0;
            s.q1 = d * q.q1;
            s.q2 = d * q.q2;
            s.q3 = d * q.q3;
            return s;
        }

        Quat mul(Quat p, Quat q){
            Quat s;
            s.q0 = 0;
            s.q1 = 0;
            s.q2 = 0;
            s.q3 = 0;
            return s;
        }

        double norm(Quat q){
            double s = sqrt(q.q0*q.q0 + q.q1*q.q1 + q.q2*q.q2 + q.q3*q.q3);
            return s;
        }

        Quat normalize(Quat q){
            Quat s;
            double norm_q = norm(q);
            s = div(q,norm_q);
            return s;
        }

        Quat conj(Quat q){
            Quat s;
            s.q0 = q.q0;
            s.q1 = -q.q1;
            s.q2 = -q.q2;
            s.q3 = -q.q3;
            return s;
        }

        Quat inv(Quat q){
            Quat s;
            s.q0 = q.q0;
            s.q1 = -q.q1;
            s.q2 = -q.q2;
            s.q3 = -q.q3;
            return s;
        }

    private:
};

#endif