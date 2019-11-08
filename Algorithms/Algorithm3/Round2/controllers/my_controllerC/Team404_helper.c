#include <webots/robot.h>
#include <stdlib.h>
#include "Team404_helper.h"
#include "tracker.h"
#include <base.h>
#include <math.h>
#define TIME_STEP 32

void step()
{
  if (wb_robot_step(TIME_STEP) == -1)
  {
    exit(0);
  }
}

void wait(float t)
{
  float initial = wb_robot_get_time();

  while (wb_robot_get_time() - initial < t)
  {
    step();
  }
}

void forwards()
{
  base_forwards();
  current_state.rec_velocity = LINEAR_VELOCITTY;
  current_state.angular_velocity = 0;
}
void backwards()
{
  base_backwards();
  current_state.rec_velocity = -LINEAR_VELOCITTY;
  current_state.angular_velocity = 0;
}
void turn_left()
{
  base_turn_left();
  current_state.rec_velocity = 0;
  current_state.angular_velocity = ANGULAR_VELOCITY;
}
void turn_right()
{
  base_turn_right();
  current_state.rec_velocity = 0;
  current_state.angular_velocity = -ANGULAR_VELOCITY;
}
void stop()
{
  base_reset();
  current_state.rec_velocity = 0;
  current_state.angular_velocity = 0;
}

void track()
{
  //printf("help \n");
  find_next(current_Position.position, next_Position.position, current_Position.UV_velocity, next_Position.UV_velocity, current_state.rec_velocity, current_state.angular_velocity);
  //printf("%f \n",current_state.angular_velocity);
  //printf("%f \n",current_state.rec_velocity);
  //printf("%f,%f\n",current_Position.UV_velocity[0],current_Position.UV_velocity[1]);
  float init[2] = {0, 1};
  //printf("%f\n",(get_angle(current_Position.UV_velocity,init)/M_PI)*180);
  for (int i = 0; i < 2; i++)
  {
    current_Position.position[i] = next_Position.position[i];

    current_Position.UV_velocity[i] = next_Position.UV_velocity[i];
  }
}

void set_initial_position()
{
  current_Position.position[0] = 0;
  current_Position.position[1] = 0;
  next_Position.position[0] = 0;
  next_Position.position[1] = 0;
  current_Position.UV_velocity[0] = 0;
  current_Position.UV_velocity[1] = 1;
  next_Position.UV_velocity[0] = 0;
  next_Position.UV_velocity[1] = 0;
}

void am_i_stuck()
{
  if (Stuck.counter == 0)
  {
    Stuck.s_x = current_Position.position[0];
    Stuck.s_y = current_Position.position[1];
    Stuck.flag = 0;
  }
  else if (Stuck.counter > 2000)
  {
    Stuck.counter = 0;
    if (Stuck.s_x - current_Position.position[0] < 0.05 && Stuck.s_x - current_Position.position[1] < 0.05){
      printf("i am stuck giving control to slave0\n");
      Stuck.flag = 1;
    }
    else
    {
      Stuck.flag =0;
    }
    
  }
  else
  { Stuck.flag = 0;
    Stuck.counter++;
  }
}