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


#define TIME_STEP 32

enum BLOB_TYPE { RED, GREEN, BLUE, NONE };
typedef enum { SEARCHING, MOVING } states;

enum MOVES{
FORWARD,
BACKWARD,
TURNR,
TURNL,
TAP,
UNTAP,
};


void step(){
  if(wb_robot_step(TIME_STEP) == -1) {
  exit(0);
  }
}




void wait(float t){
  float initial = wb_robot_get_time();

  while(wb_robot_get_time() - initial < t){
    step();
  }
  }


void print_instructions(){
char *message = "You can the Drive and control the robot using the following keys(Remember to cick the 3D view first):\nA : Turn Left\nD : Turn Right\nW : Go Forward\nS : Go Backward\nQ : Strafe Left\nE : Strafe Right\nT : Tap\nU : Untap\n";
printf(message);
}  
void tap(){
arm_ik(0.2,0.10,0);
//wait(2);
}
void untap(){
arm_ik(0.2,0.13,0);
//wwait(2);
}

void keyboard_control(int key_control){

  switch(key_control){
  case 68:
    //base_reset();
   // step();
    base_turn_left();
    printf("turn left \n");
    break;
   case 87:
     base_forwards();
     printf("go forward \n");
     break;
   case 65:
     //base_reset();
     //step();
     base_turn_right();
     printf("turn right \n");
     break;
   case 83:
     base_backwards();
     printf("go backward \n");
     break;
   case 84:
     //base_reset();
     tap();
     printf("tap \n");
     break;
   case 85:
     //base_reset();
     untap();
     printf("untap \n");
     break;
   case 66:
     base_reset();
     printf("stop");
     break;
   case 81:
     base_strafe_left();
     printf("strafe left \n");
     break;
   case 69:
     base_strafe_right();
     printf("strafe right \n");    
     break;
  }
  
}

int main(int argc, char **argv) {
  // initialization: robot
  wb_robot_init();
  wb_keyboard_enable(TIME_STEP);
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
  print_instructions();
  while (wb_robot_step(TIME_STEP) != -1) {
    // collection of range-finder and camera images
    const float *rfimage = wb_range_finder_get_range_image(range_finder);
    const unsigned char *image = wb_camera_get_image(camera);
    
    /*code snippet to translate boxes into a different 
    place when contacted with any object after 2s from
    the start of the simulation*/
    for (i = 0; i < no_boxes && wb_robot_get_time()>2.0; i++) {
      int ct = wb_supervisor_node_get_number_of_contact_points(KKboxes[i]);
      //printf("%d %d \n",ct, count);
      if (ct >0){
        WbFieldRef tr = wb_supervisor_node_get_field(KKboxes[i], "translation");
        //teleport boxes to underworld ;-)  ----disapper
        values[i][1]= -20*values[i][1];
        wb_supervisor_field_set_sf_vec3f(tr, values[i]);
      }
    }
   int key = wb_keyboard_get_key();
   if( key != -1){
   keyboard_control(key);   
   }
   
  }
  
  wb_robot_cleanup();
  return 0;
}
