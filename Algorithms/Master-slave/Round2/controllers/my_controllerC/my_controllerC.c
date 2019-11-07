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
autonomous = 1;
int teleported_boxes = 0;

//////////////////////////////////

int main(int argc, char **argv)
{
  // initialization: robot
  wb_robot_init();
  int pause_counter = 0;

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
  /* Get the camera device, enable it, and store its width and height */
  camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);
  width = wb_camera_get_width(camera);
  height = wb_camera_get_height(camera);
  enum BLOB_TYPE current_blob;
  int image_array[width * height][3];
  int filter_array[width * height];
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

  initialize_and_setup_slave(1);

  // Main loop
  while (wb_robot_step(TIME_STEP) != -1)
  { 
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
      }
    }
    int key = wb_keyboard_get_key();
    keyboard_control(key);
    get_image(camera, width, height, image_array);
    filter_image(image_array, filter_array, width, height);
    Blob blob_array[50] = BLOB_ARRAY;
    Blob largest_blob = {0, 0, 0, 0, 0};
    findblobs(blob_array, filter_array, width, height, 2, 50);
    find_largest_blob(&largest_blob,blob_array,50);
    if (autonomous == 1)
    { /////////////////////MASTER_CONTROLER//////////////////////////////////////
      //if a suitable blob is found relieve all slaves and initiate slave0 and relieve slave1
      if(Slave1.status == 1){
        printf("initializing slave0\n");
        relieve_all_slaves();
        initialize_and_setup_slave(0);
        reset_slave_variables(1);//reset slave1 memory

      }
      if(Slave0.teleported == 1){
      printf("initializing slave1\n");
      //if slave0 has teleported a box slave1 is initiated while slave0 memory is reset 
      untap();
      relieve_all_slaves();
      initialize_and_setup_slave(1);
      reset_slave_variables(0);
      reset_slave_variables(1);
      teleported_boxes++;

      }


      //if slave1 has done a full rotation update slave1 memory changing height boundary
      if(Slave1.status == 2){
        printf("changing slave1 memory\n");
        int m_height = Slave1.max_height;
        //printf("%d\n",m_height);
        reset_slave_variables(1);
        reset_slave_variables(0);//erase memory
        Slave1.height_boundary = m_height;
        

      }

      run_slave0(&largest_blob,width,height);
      run_slave1(&largest_blob);











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
      }
      show_blobs = 0;
    }
  }
  wb_robot_cleanup();

  return 0;
}
