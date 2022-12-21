#ifndef NAVIGATION_
#define NAVIGATION_

#include <math.h>
#include <Arduino.h>

#define num_points 2

class Navigation{
    public:
        Navigation();
        void set_u(float u_);
        void fill_d_arr();
        void fill_t_arr();
        void init(float u_);
        void get_next_wp_no(float t);
        void update(float t);
        float get_x();
        float get_y();

    private:
        float u = 0;
        //float x_arr[num_points] = {0,4,4,0,0};
        //float y_arr[num_points] = {0,0,4,4,0};
        float x_arr[num_points] = {0,0};
        float y_arr[num_points] = {0,10};
        float d_arr[num_points];
        float t_arr[num_points];
        float d_total;
        float t_total;

        int last_wp_no;
        int next_wp_no;

        bool goal_reached = false;

        float t_now;
        float x, y;
        float dt = 0.01;
};

#endif