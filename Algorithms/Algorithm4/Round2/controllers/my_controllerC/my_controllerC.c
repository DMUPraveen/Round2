#include <webots/keyboard.h>
#include <webots/robot.h>
#include <webots/nodes.h>
#include <webots/supervisor.h>
#include <webots/camera.h>
#include <webots/display.h>
#include <webots/range_finder.h>

#include "Team404_helper.h"
#include "Team404_keyboard_control.h"
#include "Team404_imagerecognition.h"
#include "Team404_basic_algorithm.h"
#include "Team404_range.h"
#include "map.h"
#include "tracker.h"
#include "master.h"
#include "slave.h"

#include <arm.h>
#include <base.h>
#include <gripper.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define TIME_STEP 16

enum BLOB_TYPE
{
  RED,
  GREEN,
  BLUE,
  NONE
};
typedef enum
{
  SEARCHING,
  MOVING
} states;
show_blobs = 0;
autonomous = 0;
int teleported_boxes = 0;

//////////////////////////////////
float polar_distance = 0;
float polar_angle = 0;
Map_blob blob_pos_i;
Map_blob blob_pos;
int main(int argc, char **argv)
{
  // initialization: robot
  wb_robot_init();
  int wall_flag = 0;
  float wall_vector[2];
  int pause_counter = 0;
  set_initial_position();

  // initialization: range finder
  int rf_width, rf_height;
  states state = SEARCHING;
  float distance;
  WbDeviceTag range_finder;
  /* get the device, enable it and store data*/
  range_finder = wb_robot_get_device("range-finder");
  wb_range_finder_enable(range_finder, TIME_STEP);
  rf_width = wb_range_finder_get_width(range_finder);
  rf_height = wb_range_finder_get_height(range_finder);

  // initialization: camera parameters
  int width, height;
  int red, blue, green;
  WbDeviceTag camera;
  WbDeviceTag display;
  /* Get the camera device, enable it, and store its width and height */
  camera = wb_robot_get_device("camera");
  display = wb_robot_get_device("display");
  int d_height = wb_display_get_height(display);
  int d_width = wb_display_get_width(display);
  wb_camera_enable(camera, TIME_STEP);
  width = wb_camera_get_width(camera);
  height = wb_camera_get_height(camera);
  enum BLOB_TYPE current_blob;
  int image_array[width * height][3];
  int filter_array[width * height];
  printf("width: %d,height: %d \n", width, height);
  //float angle = 0;
  // initialization: obstacle-boxes
  int i, j;
  int no_boxes = 50;
  WbNodeRef KKboxes[no_boxes];
  char Obs_names[no_boxes][6];
  char tmp[3];
  int teleported_boxes = 0;
  /* random position vectors-values- for obstacles defined as KKBx */
  //int pre_i=-1;
  srand(time(0));
  double values[no_boxes][3];
  for (i = 0; i < no_boxes; i++)
  {
    char kb[6] = {"KKB"};
    sprintf(tmp, "%d", i);
    strcpy(Obs_names[i], strcat(kb, tmp));
    for (j = 0; j < 3; j++)
    {
      values[i][j] = round((rand() % 100 - 49.5)) / 10.0;
      values[i][1] = 0.041;
    }
  }
  /* setting KKBx translations to above random values */
  for (i = 0; i < no_boxes; i++)
  {
    KKboxes[i] = wb_supervisor_node_get_from_def(Obs_names[i]);
    WbFieldRef tr = wb_supervisor_node_get_field(KKboxes[i], "translation");
    wb_supervisor_field_set_sf_vec3f(tr, values[i]);
  }

  // initialization: robot base, arm and the gripper
  base_init();
  arm_init();
  gripper_init();
  wb_keyboard_enable(TIME_STEP);
  base_reset();

  initialize_and_setup_slave(3);
  setup_slave_3();
  printf("%d\n", Slave0.tapped);
  //Slave0.teleported =0;
  Stuck.flag = 0;
  float true_x =  0;
  float true_y = 0;
  Memory map_memory;
  set_memory(&map_memory);
  // Main loop
  while (wb_robot_step(TIME_STEP) != -1)
  {
    ///printf("x:%f, y:%f ",current_Position.position[0],current_Position.position[1]);
    //wb_display_set_color(display,0x000000);
    //wb_display_fill_rectangle(display,0,0,d_width,d_height);
    //show_where_I_am(display, current_Position.position[0], current_Position.position[1], 0x000000, d_width, d_height);
    //show_where_I_am(display,true_x,true_y,0x000000,d_width,d_height);
    WbNodeRef robot = wb_supervisor_node_get_from_def("youbot");
    true_x = wb_supervisor_node_get_position(robot)[0];
    true_y = wb_supervisor_node_get_position(robot)[2];
    //printf("hello world\n");
    show_where_I_am(display,-true_y,true_x,0xff00ff,d_width,d_height);
    //turn_left();
    //printf("%f \n",angle*180/M_PI);
    //printf("teleported_boxes_no: %d \n",teleported_boxes);
    // collection of range-finder and camera images
    const float *rfimage = wb_range_finder_get_range_image(range_finder);
    const unsigned char *image = wb_camera_get_image(camera);

    /*code snippet to translate boxes into a different 
    place when contacted with any object after 2s from
    the start of the simulation*/
    for (i = 0; i < no_boxes && wb_robot_get_time() > 2.0; i++)
    {
      int ct = wb_supervisor_node_get_number_of_contact_points(KKboxes[i]);
      if (ct > 0)
      {
        WbFieldRef tr = wb_supervisor_node_get_field(KKboxes[i], "translation");
        //teleport boxes to underworld ;-)  ----disapper
        values[i][1] = -20 * values[i][1];
        wb_supervisor_field_set_sf_vec3f(tr, values[i]);
        //printf("%d\n",teleported_boxes);
      }
    }
    int key = wb_keyboard_get_key();
    keyboard_control(key);
    get_image(camera, width, height, image_array);
    filter_image(image_array, filter_array, width, height);
    Blob blob_array[50] = BLOB_ARRAY;
    Blob largest_blob = {0, 0, 0, 0, 0};
    findblobs(blob_array, filter_array, width, height, 2, 50);
    find_largest_blob(&largest_blob, blob_array, 50);
    track();
    //draw_display(display,image_array,width,height);
    //Draw_blobs(display,blob_array);
    //float velocity = (polar_distance - find_distance_to_blob(&largest_blob))/0.016;
    //float a_velocity = (polar_angle -find_angle_to_blob(&largest_blob))/0.016;
    if (largest_blob.height > 0 && largest_blob.corner_x > 10 && (largest_blob.corner_x + largest_blob.width) < 50)
    {

      float polar_distance = find_distance_to_blob(&largest_blob);
      float polar_angle = find_angle_to_blob(&largest_blob);
      //printf("distance:%f angle:%f height:%d cornerx:%d cornery:%d \n", polar_distance, (polar_angle * 180) / M_PI, largest_blob.height, largest_blob.corner_x, largest_blob.corner_y);
      map_blob(&largest_blob, &blob_pos, current_Position.UV_velocity, current_Position.position);
      float a = blob_pos.x - blob_pos_i.x;
      float b = blob_pos.y - blob_pos_i.y;
      //printf("a:%f,b:%f", a, b);
      
      if ((a > 0.08 || a < -0.08) && (a > 0.08 || b < -0.08))
      {

        show_where_I_am(display,blob_pos.x,blob_pos.y,0x00ff00,d_width,d_height);
        store_in_memory(&map_memory,blob_pos.x,blob_pos.y);
        
      }
      blob_pos_i.x = blob_pos.x;
      blob_pos_i.y = blob_pos.y;
    }
    //wb_display_set_color(display,0x000000);
    //wb_display_fill_rectangle(display,(current_Position.position[0]/10)*d_width+d_width/2,(current_Position.position[1]/10)*d_height+d_height/2,3,3);
    clear_in_memory(&map_memory,current_Position.position[0],current_Position.position[1]);
    display_memory(display,&map_memory,d_width,d_height);
    float vectora[2];
    float vectorb[2];
    rotate_vector(current_Position.UV_velocity, vectora, 0.15);
    rotate_vector(current_Position.UV_velocity, vectorb, -0.15);
    show_where_I_am(display,4*vectora[0],4*vectora[1],0x00ffff,d_width,d_height);
    show_where_I_am(display,4*vectorb[0],4*vectorb[1],0xffff00,d_width,d_height);
    show_where_I_am(display,4*current_Position.UV_velocity[0],4*current_Position.UV_velocity[1],0xff0000,d_width,d_height);
    ///show_where_I_am(display,4*vectora[0],4*vectora[1],0x000000,d_width,d_height);
    ///show_where_I_am(display,4*vectorb[0],4*vectorb[1],0x000000,d_width,d_height);
    ///show_where_I_am(display,4*current_Position.UV_velocity[0],4*current_Position.UV_velocity[1],0x000000,d_width,d_height);

    
    show_where_I_am(display, current_Position.position[0], current_Position.position[1], 0xffffff, d_width, d_height);

    float av_distance = get_range_finder_image(range_finder, rf_width, rf_height);
    if (av_distance < 0.3 && wall_flag == 0)
    {
      //printf("updating\n");
      wall_flag = 1;
      wall_vector[0] = current_Position.UV_velocity[0];
      wall_vector[1] = current_Position.UV_velocity[1];
    }
    //show_where_I_am(display,current_Position.position[1],-current_Position.position[0],0xffffff,d_width,d_height);
    //int count = get_blob_count(blob_array,50);
    //printf("%d\n",count);

    //printf("%f,%f\n",current_Position.UV_velocity[0],current_Position.UV_velocity[1]);
    am_i_stuck();
    if (Stuck.flag)
    {
      wall_flag = 0;
    }
    //printf("%d \n",Stuck.flag);
    if (autonomous == 1)
    { /////////////////////MASTER_CONTROLER//////////////////////////////////////
      //if a suitable blob is found relieve all slaves and initiate slave0 and relieve slave1
      if (wall_flag)
      {
        untap();
        relieve_all_slaves();
        int wall_status = turn_left_by(120, wall_vector, current_Position.UV_velocity);
        //printf("%f,%f",wall_vector[0],wall_vector[1]);
        if (wall_status)
        {
          printf("initializing slave0 after avoiding wall\n");
          wall_flag = 0;
          setup_slave_0();
          initialize_and_setup_slave(0);
        }
      }

      if ((Slave3.status == 2 || Stuck.flag == 1) && !wall_flag)
      {
        printf("initializing slave0\n");
        relieve_all_slaves();
        initialize_and_setup_slave(0);
        setup_slave_0();
        setup_slave_3();
      }
      if (Slave0.teleported == 1 && Stuck.flag != 1 && !wall_flag)
      {
        //printf("%d\n",Slave0.teleported);
        printf("initializing salve3 \n");
        relieve_all_slaves();
        initialize_and_setup_slave(3);
        Slave3.initial_uv_vector[0] = current_Position.UV_velocity[0];
        Slave3.initial_uv_vector[1] = current_Position.UV_velocity[1];
        setup_slave_0();
        teleported_boxes++;
        printf("boxes:%d\n", teleported_boxes);
      }

      run_slave0(&largest_blob, width, height);
      //printf("%d",slave_status[3]);
      run_slave_3(blob_array, 50, current_Position.UV_velocity, &largest_blob);
      //run_slave1(&largest_blob,blob_array,current_Position.UV_velocity);
      //run_slave2(current_Position.UV_velocity);
      //printf("sweeps:%d\n",Slave3.sweeps);
      //printf("status:%d\n",Slave3.status);
    }
    if (show_blobs == 1)
    {
      printf("show blobs \n");
      for (int i = 0; i < 50; i++)
      {
        if (blob_array[i].initialized == 0)
        {
          break;
        }
        printf("blob_no: %d, corner_x: %d,corner_y: %d,width: %d,height: %d \n", i + 1, blob_array[i].corner_x, blob_array[i].corner_y, blob_array[i].width, blob_array[i].height);
        wb_display_set_color(display, 0x000000);
        wb_display_fill_rectangle(display, 0, 0, d_width, d_height);
      }
      show_blobs = 0;
    }
  }
  wb_robot_cleanup();

  return 0;
}
