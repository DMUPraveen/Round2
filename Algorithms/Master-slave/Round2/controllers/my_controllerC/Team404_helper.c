#include <webots/robot.h>
#include <stdlib.h>
#include "Team404_helper.h" 

#define TIME_STEP 32

void step(){
    if(wb_robot_step(TIME_STEP)==-1){
        exit(0);
    }
}

void wait(float t){
  float initial = wb_robot_get_time();

  while(wb_robot_get_time() - initial < t){
    step();
  }
  }