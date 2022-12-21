#ifndef FILTER
#define FILTER

#include "Constants.h"

class LPF_filter{
  public:
    LPF_filter();
    
    void set_param(float fc_, float dt_);
    void get_param(float* fc_, float* dt_);
    
    void set_fc(float fc_);
    void set_tau(float tau_);
    void set_dt(float dt_);
    void set_alpha(float alpha_);

    float get_fc();
    float get_tau();
    float get_dt();
    float get_alpha();

    float get_y();

    void cal_y(float x_i);
    void reset();

  private:
    float fc = 0.0;
    float tau = 0.0;
    float dt = 0.0;
    float alpha = 0.0;

    float y_pre = 0.0;
    float y = 0.0;
    bool start = true;
};

///////////////////////////////////////////////////////

class HPF_filter{
  public:
    HPF_filter();
    
    void set_param(float fc_, float dt_);
    void get_param(float* fc_, float* dt_);
    
    void set_fc(float fc_);
    void set_tau(float tau_);
    void set_dt(float dt_);
    void set_alpha(float alpha_);

    float get_fc();
    float get_tau();
    float get_dt();
    float get_alpha();

    float get_y();

    void cal_y(float x_i);
    void reset();

  private:
    float fc = 0.0;
    float tau = 0.0;
    float dt = 0.0;
    float alpha = 0.0;

    float x_pre = 0.0;
    float y_pre = 0.0;
    float y = 0.0;
    bool start = true;
};

#endif
