
#ifndef HELPER
#define HELPER
#include <webots/robot.h>
#include <arm.h>
#include "tracker.h"

struct state{
  float rec_velocity;
  float angular_velocity;
};
struct position{
  float position[2];
  float UV_velocity[2];
};
struct state current_state;
struct position current_Position;
struct position next_Position;

struct stuck{
int counter;
float s_x;
float s_y;
int flag;
};
struct stuck Stuck;

void am_i_stuck();
void stop();
void turn_right();
void turn_left();
void backwards();
void forwards();
void set_initial_position();


void step();
void wait(float t);

#endif