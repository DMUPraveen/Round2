
#include <webots/robot.h>
#include <webots/range_finder.h>
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
  WbDeviceTag range_finder;
  range_finder = wb_robot_get_device("range-finder"); 
  wb_range_finder_enable(range_finder,32);
  
  arm_init();
  //wait(3.0);
  arm_ik(0.26,0.12,0);
  wait(3);
  for(float i = 0.26;i <0.28; i+=0.001){
  arm_ik(i,0.12,0);
  printf("%f \n",i);
  step();
  wait(1);
  
  }
  while (1) {
  step();
  
  
 


  }
  wb_robot_cleanup();

  return 0;
}
