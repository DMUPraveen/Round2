#include <webots/keyboard.h>
#include <webots/robot.h>
#include <webots/nodes.h>
#include <webots/supervisor.h>
#include <webots/camera.h>
#include <webots/range_finder.h>

#include <arm.h>
#include <base.h>
#include <gripper.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>


#define TIME_STEP 16

enum BLOB_TYPE { RED, GREEN, BLUE, NONE };
typedef enum { SEARCHING, MOVING } states;


int main(int argc, char **argv) {
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
  
  // initialization: obstacle-boxes
  int i, j;
  int no_boxes = 3;
  WbNodeRef KKboxes[no_boxes];
  char Obs_names[no_boxes][6];
  char tmp[3];
  /* random position vectors-values- for obstacles defined as KKBx */
  srand(time(0)); 
  double values[no_boxes][3];  
  for(i = 0; i<no_boxes; i++){
    char kb[6]={"KKB"};
    sprintf(tmp, "%d", i); 
    strcpy(Obs_names[i], strcat(kb, tmp));   
    for(j = 0; j<3; j++){ 
      values[i][j]= round((rand()%100-49.5))/10.0;
      values[i][1]= 0.03;
    }
  }
  /* setting KKBx translations to above random values */
  for (i = 0; i < no_boxes ; i++) {
    KKboxes[i] = wb_supervisor_node_get_from_def(Obs_names[i]);
    WbFieldRef tr = wb_supervisor_node_get_field(KKboxes[i], "translation");
    wb_supervisor_field_set_sf_vec3f(tr, values[i]);
  }
  
  // initialization: robot base, arm and the gripper
  base_init();
  arm_init();
  gripper_init();
  
  // Main loop
  while (wb_robot_step(TIME_STEP) != -1) {
    // collection of range-finder and camera images
    const float *rfimage = wb_range_finder_get_range_image(range_finder);
    const unsigned char *image = wb_camera_get_image(camera);
    
    /*code snippet to translate boxes into a different 
    place when contacted with any object after 2s from
    the start of the simulation*/
    for (i = 0; i < no_boxes && wb_robot_get_time()>2.0; i++) {
      int ct = wb_supervisor_node_get_number_of_contact_points(KKboxes[i]);
      if (ct >0){
        WbFieldRef tr = wb_supervisor_node_get_field(KKboxes[i], "translation");
        //teleport boxes to underworld ;-)  ----disapper
        values[i][1]= -20*values[i][1];
        wb_supervisor_field_set_sf_vec3f(tr, values[i]);
      }
    }
    
    /*==================================================
      example code for color detection using camera and
      simple control algorithm for stoping and steering away*/
    if (pause_counter > 0)
      pause_counter--;
    /*
     * Case 1
     * A blob was found recently
     * The robot waits in front of it until pause_counter
     * is decremented enough
     */
    if (pause_counter > 50) {
      base_reset();
    }
    /*
     * Case 2
     * A blob was found quite recently
     * The robot begins to turn but don't analyse the image for a while,
     * otherwise the same blob would be found again
     */
    else if (pause_counter > 0) {
      base_turn_left();
    }
    /*
     * Case 3
     * The robot turns and analyse the camera image in order
     * to find a new blob
     */
    else {  // pause_counter == 0  
      red = 0;
      green = 0;
      blue = 0;
    
      for (i = width / 3; i < 2 * width / 3; i++) {
        for (j = height / 2; j < 3 * height / 4; j++) {
          red += wb_camera_image_get_red(image, width, i, j);
          blue += wb_camera_image_get_blue(image, width, i, j);
          green += wb_camera_image_get_green(image, width, i, j);
        }
      }
    
      if ((red > 3 * green) && (red > 3 * blue))
        current_blob = RED;
      else if ((green > 3 * red) && (green > 3 * blue))
        current_blob = GREEN;
      else if ((blue > 3 * red) && (blue > 3 * green))
        current_blob = BLUE;
      else
        current_blob = NONE;

    
      if (current_blob == NONE) {
        base_turn_left();
      }
      /*
       * Case 3b
       * A blob is detected
       * the robot stops, stores the image, and changes its state
       */
      else {
        base_reset();// stoping robot
        pause_counter = 100;
      }
    }     
  }
  wb_robot_cleanup();
  return 0;
}
