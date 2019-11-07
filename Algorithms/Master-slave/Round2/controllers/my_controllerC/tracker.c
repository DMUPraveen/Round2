#include "tracker.h"
#include <math.h>
#include <webots/robot.h>
#include <webots/display.h>
#include <base.h>
#include <stdio.h>

void find_next(float position1[2], float position2[2], float velocity1[2], float velocity2[2], float v, float av)
{
    //calculates the position and velocity of the next time step(store them in position2 velocity2) according to the current position and velocity(velocity2,poition2)
    for (int i = 0; i < 2; i++)

    { //printf("%f \n",v);
        position2[i] = v * velocity1[i] * T_STEP + position1[i];
    }
    velocity2[0] = cos(av * T_STEP) * velocity1[0] - sin(av * T_STEP) * velocity1[1];
    velocity2[1] = sin(av * T_STEP) * velocity1[0] + cos(av * T_STEP) * velocity1[1];
}

void show_where_I_am(WbDeviceTag Display, float xpos, float ypos, int color, int d_width, int d_height)
{
    int w = d_width;
    int h = d_height;
    int x = ((xpos) / 10) * w + w / 2;
    int y = ((ypos) / 10) * h + h / 2;
    wb_display_set_color(Display, color);
    wb_display_draw_pixel(Display, x, y);
}

float get_angle(float vector1[2], float vector2[2])
{
    //range is [-pi,pi]
    float dot_product = vector1[0] * vector2[0] + vector1[1] * vector2[1];
    float mag_product = magnitude(vector1) * magnitude(vector2);
    float angle = acos(dot_product / mag_product);
    //printf("%f,%f,%f \n",dot_product,mag_product,angle);
    return angle;
}

float magnitude(float vector[2])
{
    return sqrt(vector[0] * vector[0] + vector[1] * vector[1]);
}
int turn_left_by(float d_angle, float initial_v_vector[2], float current_vector[2])

{
    float angle = get_angle(initial_v_vector, current_vector);
    printf("%f \n",angle);
    float r_angle = (d_angle / 180) * M_PI;
    if (angle <= r_angle)
    {
        base_turn_left();
        return 0;
    }
    else
    {
        base_reset();
        return 1;
    }
}

void rotate(float* vector1,float* vector2,float av){
    vector2[0] = cos(av * T_STEP) * vector1[0] - sin(av * T_STEP) * vector1[1];
    vector2[1] = sin(av * T_STEP) * vector1[0] + cos(av * T_STEP) * vector1[1];
}