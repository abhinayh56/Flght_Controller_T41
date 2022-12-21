#include "Navigation.h"

Navigation::Navigation(){
}

void Navigation::set_u(float u_){
    u = u_;
}

void Navigation::fill_d_arr(){
    d_arr[0] = 0;
    for(int i=1;i<num_points;i++){
        float dx = x_arr[i] - x_arr[i-1];
        float dy = y_arr[i] - y_arr[i-1];
        float dl = sqrt(dx*dx + dy*dy);
        d_arr[i] = d_arr[i-1] + dl;
    }
    d_total = d_arr[num_points-1];
}

void Navigation::fill_t_arr(){
    t_arr[0] = 0;
    for(int i=1;i<num_points;i++){
        t_arr[i] = d_arr[i] / u;
    }
    t_total = t_arr[num_points-1];
}

void Navigation::init(float u_){
    set_u(u_);
    fill_d_arr();
    fill_t_arr();
    last_wp_no = 0;
    goal_reached = false;

    /*
    for(int i=0;i<num_points;i++){
        Serial.print(x_arr[i]); Serial.print('\t');
    }
    Serial.println();

    for(int i=0;i<num_points;i++){
        Serial.print(y_arr[i]); Serial.print('\t');
    }
    Serial.println();

    for(int i=0;i<num_points;i++){
        Serial.print(d_arr[i]); Serial.print('\t');
    }
    Serial.println();

    for(int i=0;i<num_points;i++){
        Serial.print(t_arr[i]); Serial.print('\t');
    }
    Serial.println();
    */
}

void Navigation::get_next_wp_no(float t){
    if(t>t_total){
        goal_reached = true;
    }
    else{
        goal_reached = false;
    }

    if(goal_reached==true){
        next_wp_no = num_points-1;
        last_wp_no = num_points-2;
    }
    else{
        for(int i=0;i<num_points-1;i++){
            if(t>t_arr[i]){
                next_wp_no = i+1;
                last_wp_no = i;
            }
        }
    }
}

void Navigation::update(float t){
    if(t>t_total){
        goal_reached = true;
    }
    else{
        goal_reached = false;
    }

    Serial.print(goal_reached); Serial.print('\t');

    if(goal_reached==true){
        x = x_arr[num_points-1];
        y = y_arr[num_points-1];
    }
    else{
        get_next_wp_no(t);

        Serial.print(last_wp_no); Serial.print('\t');
        Serial.print(next_wp_no); Serial.print('\t');

        float dx = x_arr[next_wp_no] - x_arr[last_wp_no];
        float dy = y_arr[next_wp_no] - y_arr[last_wp_no];
        float dl = sqrt(dx*dx + dy*dy);
        float ux = u*(dx/dl);
        float uy = u*(dy/dl);
        x += ux*dt;
        y += uy*dt;

        x = x_arr[last_wp_no] + (x_arr[next_wp_no]-x_arr[last_wp_no]) * (t-t_arr[last_wp_no])/(t_arr[next_wp_no]-t_arr[last_wp_no]);
        y = y_arr[last_wp_no] + (y_arr[next_wp_no]-y_arr[last_wp_no]) * (t-t_arr[last_wp_no])/(t_arr[next_wp_no]-t_arr[last_wp_no]);

        Serial.print(x); Serial.print('\t');
        Serial.print(y); Serial.print('\t');
    }
    Serial.println();
}

float Navigation::get_x(){
    return x;
}

float Navigation::get_y(){
    return y;
}