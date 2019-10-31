
#include <webots/robot.h>
#include <stdio.h>
#include <stdlib.h>


#include <gripper.h>
#include <arm.h>
#include <base.h>

#define TIME_STEP 32




void step(){
  if(wb_robot_step(TIME_STEP) == -1) {
  exit(0);
  }
}


void wait(float time){
  float initial = wb_robot_get_time();

  while(wb_robot_get_time() - initial < time){
    step();
  }



}
int main(int argc, char **argv) {
  wb_robot_init();
  
  arm_init();
  //wait(3.0);
  while (1) {
  step();
  
  
 


  }
  wb_robot_cleanup();

  return 0;
}
