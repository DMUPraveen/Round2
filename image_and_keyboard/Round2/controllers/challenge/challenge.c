#include <webots/keyboard.h>
#include <webots/robot.h>
#include <webots/nodes.h>
#include <webots/supervisor.h>
#include <webots/camera.h>
#include <webots/range_finder.h>
#include <webots/display.h>

#include <arm.h>
#include <base.h>
#include <gripper.h>

#include "Team404_helper.h"
#include "Team404_imagerecognition.h"
#include "Team404_keyboard_control.h"
#include "Team404_range.h"

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
  WbDeviceTag display;
  /* Get the camera device, enable it, and store its width and height */
  camera = wb_robot_get_device("camera");
  display = wb_robot_get_device("display");
  wb_camera_enable(camera, TIME_STEP);
  width = wb_camera_get_width(camera);
  height = wb_camera_get_height(camera);
  enum BLOB_TYPE current_blob;

  int image_array[width * height][3];
  int filter_array[width * height];
  printf("width: %d,height: %d \n", width, height);
  wait(1.0);

  // initialization: obstacle-boxes
  int i, j;
  int no_boxes = 3;
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
      values[i][1] = 0.03;
    }
  }

  for (i = 0; i < no_boxes; i++)
  {
    /* setting KKBx translations to above random values */
    KKboxes[i] = wb_supervisor_node_get_from_def(Obs_names[i]);
    WbFieldRef tr = wb_supervisor_node_get_field(KKboxes[i], "translation");
    wb_supervisor_field_set_sf_vec3f(tr, values[i]);
  }

  // initialization: robot base, arm and the gripper
  base_init();
  arm_init();
  gripper_init();
  wb_keyboard_enable(TIME_STEP);

  while (wb_robot_step(TIME_STEP) != -1)
  {
    // collection of range-finder and camera images
    const float *rfimage = wb_range_finder_get_range_image(range_finder);
    const unsigned char *image = wb_camera_get_image(camera);

    for (i = 0; i < no_boxes && wb_robot_get_time() > 2.0; i++)
    {
      /*code snippet to translate boxes into a different 
      place when contacted with any object after 2s from
      the start of the simulation*/

      int ct = wb_supervisor_node_get_number_of_contact_points(KKboxes[i]);

      if (ct > 4)
      {
        printf("%d\n", ct);
        WbFieldRef tr = wb_supervisor_node_get_field(KKboxes[i], "translation");
        //teleport boxes to underworld ;-)  ----disapper
        values[i][1] = -20 * values[i][1];
        wb_supervisor_field_set_sf_vec3f(tr, values[i]);
      }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    int key = wb_keyboard_get_key();
    keyboard_control(key);
    get_image(camera, width, height, image_array);
    filter_image(image_array, filter_array, width, height);
    Blob blob_array[50] = BLOB_ARRAY;
    findblobs(blob_array, filter_array, width, height, 2, 50);

    if (show_display)
    {
      draw_display(display, image_array, width, height);
      Draw_blobs(display, blob_array);

      /*
      float r_distance = get_range_finder_image(range_finder, rf_width, rf_height);
      float limit = 0.5;
      printf("%f \n", r_distance);
      int blacks = get_close_pixel_no(range_finder, rf_width, rf_height);
      printf("%d\n", blacks);
      if (r_distance < limit)
      {
        base_turn_left();
        while (r_distance < limit)
        {
          r_distance = get_range_finder_image(range_finder, rf_width, rf_height);
          step();
        }
        base_forwards();
      }
      if (blacks > 300)
      { 
        wait(0.1);
        base_reset();
        tap();
        wait(1);
        untap();
        wait(1);
        base_backwards();
        wait(1);
      }
    }
    */
      //////////////////////////////////////////////////////////////////////////////////////////////////////
    }
  }
  wb_robot_cleanup();
  return 0;
}
