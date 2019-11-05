#include "tracker.h"
#include <math.h>
#include <webots/robot.h>
#include <webots/display.h>

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

void show_where_I_am(WbDeviceTag Display, float xpos,float ypos,int color,int d_width,int d_height)
{   
    int w = d_width;
    int h = d_height;
    int x = ((xpos)/10)*w +w/2;
    int y = ((ypos)/10)*h +h/2;
    wb_display_set_color(Display,color);
    wb_display_draw_pixel(Display,x,y);
    
}
