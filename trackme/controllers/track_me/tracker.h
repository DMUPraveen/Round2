#ifndef TRACKME
#define TRACKME

#include <webots/robot.h>

#define T_STEP 0.032
#define LINEAR_VELOCITTY 0.2
#define ANGULAR_VELOCITY 0.38748
void find_next(float position1[2], float position2[2], float velocity1[2], float velocity2[2],float v,float av);
void show_where_I_am(WbDeviceTag Display, float xpos,float ypos, int color,int d_width,int d_height);
#endif