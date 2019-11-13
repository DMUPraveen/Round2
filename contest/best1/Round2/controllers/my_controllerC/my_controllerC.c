#include <webots/keyboard.h>
#include <webots/robot.h>
#include <webots/nodes.h>
#include <webots/supervisor.h>
#include <webots/camera.h>
#include <webots/range_finder.h>

#include "Team404_helper.h"
#include "Team404_keyboard_control.h"
#include "Team404_imagerecognition.h"
#include "Team404_basic_algorithm.h"
#include "Team404_range.h"

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
autonomous = 1;
int tapped = 0;
float h_limit = 10;
int main(int argc, char **argv)
{
  // initialization: robot
  wb_robot_init();
  int pause_counter = 0;
  int wall_flag = 0;
  current_angle = 0;

  // initialization: range finder
  int rf_width, rf_height;
  states state = SEARCHING;
  float distance;
  WbDeviceTag range_finder;
  //WbDeviceTag display;
  /* get the device, enable it and store data*/
  range_finder = wb_robot_get_device("range-finder");
  wb_range_finder_enable(range_finder, TIME_STEP);
  rf_width = wb_range_finder_get_width(range_finder);
  rf_height = wb_range_finder_get_height(range_finder);

  //display = wb_robot_get_device("display");

  // initialization: camera parameters
  int width, height;
  int red, blue, green;
  WbDeviceTag camera;
  /* Get the camera device, enable it, and store its width and height */
  camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);
  width = wb_camera_get_width(camera);
  height = wb_camera_get_height(camera);
  enum BLOB_TYPE current_blob;
  int image_array[width * height][3];
  int filter_array[width * height];
  int r_filter_array[rf_width * rf_height];
  printf("width: %d,height: %d \n", width, height);

  // initialization: obstacle-boxes
  int i, j;
  int no_boxes = 50;
  WbNodeRef KKboxes[no_boxes];
  char Obs_names[no_boxes][6];
  char tmp[3];
  /* random position vectors-values- for obstacles defined as KKBx */
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

  // Main loop
  while (wb_robot_step(TIME_STEP) != -1)
  {
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
      }
    }
    int key = wb_keyboard_get_key();
    keyboard_control(key);
    get_image(camera, width, height, image_array);
    filter_image(image_array, filter_array, width, height);
    filter_range_image(r_filter_array, range_finder, rf_width, rf_height, 0.5);
    Blob blob_array[50] = BLOB_ARRAY;
    Blob largest_blob = {0, 0, 0, 0, 0};
    Blob r_blob_array[50] = BLOB_ARRAY;
    findblobs(r_blob_array, r_filter_array, rf_width, rf_height, 2, 50);
    findblobs(blob_array, filter_array, width, height, 2, 50);
    float av_distance = get_range_finder_image(range_finder, rf_width, rf_height);

    //printf("%f\n",av_distance);
    if (av_distance < 0.4 && wall_flag == 0 && !tapped)
    {

      printf("detecting wall\n");
      wall_flag = 1;
      current_angle = 0;
    }
    //printf("%d\n",wall_flag);

    if (wall_flag == 1)
    {

      //printf("%d\n",wall_flag);

      printf("%f\n", current_angle);
      if (current_angle > 1.5)
      {
        wall_flag = 0;
        current_angle = 0;
      }
      else
      {
        current_angle = current_angle + ANGULAR_VELOCITY * T_STEP;
        base_turn_left();
        untap();
      }
    }
    ///float obstacle = singed_distance(r_filter_array, rf_width, rf_height);
    ///printf("%f\n", singed_distance(r_filter_array, rf_width, rf_height));
    if (autonomous == 1)
    {

      if (wall_flag == 0)
      {
        //float obstacle = singed_distance(range_finder, rf_width, rf_height);
        
    
        find_largest_blob(&largest_blob, blob_array, 50);
        int status = 0;
        
        if(av_distance >0.6){
        status = check_sides(r_blob_array,rf_width,rf_height);
        }
        
        printf("%d\n", status);

        if (status == 0)
        { 
          if(largest_blob.height > h_limit){
          go_to_largest_blob(&largest_blob, width / 6, width, height, 52, &tapped);
          if(tapped){
          h_limit = 10;
          }
        }else{
          untap();
          base_turn_left();
          h_limit = h_limit -0.01;
          printf("h_limit:%f\n",h_limit);
        }
        }
        
        
      }
      //printf("%d \n", show_blobs);
    }

    if (show_blobs == 1)
    {
      printf("show blobs \n");
      //display_show_range(r_filter_array,rf_width,rf_height,display);
      //Draw_blobs(display,r_blob_array);
      for (int i = 0; i < 50; i++)
      {
        if (blob_array[i].initialized == 0)
        {
          break;
        }
        printf("blob_no: %d, corner_x: %d,corner_y: %d,width: %d,height: %d \n", i + 1, blob_array[i].corner_x, blob_array[i].corner_y, blob_array[i].width, blob_array[i].height);
      }
      show_blobs = 0;
    }
  }
  wb_robot_cleanup();

  return 0;
}
