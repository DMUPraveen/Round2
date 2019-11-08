#ifndef TRACKME
#define TRACKME

#include <webots/robot.h>

#define T_STEP 0.016
#define LINEAR_VELOCITTY 0.2
#define ANGULAR_VELOCITY 0.38748
void find_next(float position1[2], float position2[2], float velocity1[2], float velocity2[2],float v,float av);
void show_where_I_am(WbDeviceTag Display, float xpos,float ypos, int color,int d_width,int d_height);
float get_angle(float vector1[2], float vector2[2]);
float magnitude(float vector[2]);
int turn_left_by(float d_angle,float initial_v_vector[2],float current_vector[2]);
void rotate(float* vector1,float* vector2,float av);
void track();
int which_way_to_turn(float *c_vector,float* t_vecot);
#endif