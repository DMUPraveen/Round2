#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/display.h>
#include <webots/keyboard.h>

#include <base.h>

#include "Team404_helper.h"
#include "Team404_keyboard_control.h"
#include "tracker.h"

#include <stdio.h>
#include <math.h>

#define MESSAGE "this code allows you to track the youbots progress in the world by calculating its position\n show in red is the calculated position and shown in blue is the actual postion,\n this will not work if the robot hits the wall but continues to drive\n you can use the usual keyboard commands to drive the robot\n both tracks might not be visible if the calculated and true positions are very close together\n"
#define TIME_STEP 32

float velocity1[2] = {0, 1};
float velocity2[2] = {0, 0};
float position1[2] = {0, 0};
float position2[2] = {0, 0};
float velocity = 0;
float angular_velocity = 0;
moves currentmove = STATIONARY;


int main()
{
  printf(MESSAGE);
  wb_robot_init();
  base_init();
  WbNodeRef youbot = wb_supervisor_node_get_from_def("youbot");
  
  /*if (youbot != NULL)
  {
    printf("success \n");
    //step();
  }*/
  WbDeviceTag Display = wb_robot_get_device("display");
  int width = wb_display_get_width(Display);
  int height = wb_display_get_height(Display);
  wb_keyboard_enable(TIME_STEP);

  while (1)
  {
    keyboard_control(wb_keyboard_get_key());
    switch (currentmove)
    {
    case STATIONARY:
      velocity = 0;
      angular_velocity = 0;
      break;
    case FORWARD:
      velocity = LINEAR_VELOCITTY;
      angular_velocity = 0;
      break;
    case BACKWARD:
      velocity = -LINEAR_VELOCITTY;
      angular_velocity = 0;
      break;
    case TURNL:
      velocity = 0;
      angular_velocity = ANGULAR_VELOCITY;
      break;
    case TURNR:
      velocity = 0;
      angular_velocity = -ANGULAR_VELOCITY;
      break;
    }
    //printf("%f \n",velocity);
  find_next(position1,position2,velocity1,velocity2,velocity,angular_velocity);
  for (int i = 0; i < 2; i++)
  {
    position1[i] = position2[i];
    velocity1[i] = velocity2[i];
  }
  printf("%f,%f\n",velocity1[0],velocity1[1]);
    
    const double *r_position = wb_supervisor_node_get_position(youbot);
    //printf("i_think_position: (%f,%f) \n, true_position: (%f,%f)",position1[0],position1[1],r_position[0],r_position[2]);
    show_where_I_am(Display,position1[1],-position1[0],0xff0000,width,height);
    show_where_I_am(Display,r_position[0],r_position[2],0x0000ff,width,height);
    /*
    float i_t = wb_robot_get_time();
    float i_x = wb_supervisor_node_get_position(youbot)[2];
    float i_angle = acos(wb_supervisor_node_get_orientation(youbot)[0]);
    //printf("angle : %f \n",i_angle);
    //printf("[%f,%f,%f],[%f,%f,%f],[%f,%f,%f]",rotation[0],rotation[1],rotation[2],rotation[3],rotation[4],rotation[5],rotation[6],rotation[7],rotation[8]);
    step(); 
    float t = wb_robot_get_time();
    float x = wb_supervisor_node_get_position(youbot)[2];
    float angle = acos(wb_supervisor_node_get_orientation(youbot)[0]);
    float a_speed = (angle - i_angle)/(t - i_t); 
    float speed =  (x - i_x)/(t - i_t);
    printf("rec_speed: %f, ang_speed: %f\n",speed,a_speed);
    */
  }

  wb_robot_cleanup();
  return 0;
}
