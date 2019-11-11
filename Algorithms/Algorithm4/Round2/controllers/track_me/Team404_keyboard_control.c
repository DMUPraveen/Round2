#include <webots/keyboard.h>
#include <webots/robot.h>

#include <arm.h>
#include <base.h>
#include <gripper.h>

#include "Team404_keyboard_control.h"
//#include "Team404_imagerecognition.h"

#include <stdio.h>

#define TIME_STEP 32


void print_instructions()
{ // prints the instructions for keyboard control
    char *message = "You can the Drive and control the robot using the following keys(Remember to cick the 3D view first):\nA : Turn Left\nD : Turn Right\nW : Go Forward\nS : Go Backward\nQ : Strafe Left\nE : Strafe Right\nT : Tap\nU : Untap\n";
    printf(message);
}

void tap()
{ //taps the box
    arm_ik(0.2, 0.10, 0);
}

void untap()
{ //untaps the boxes
    arm_ik(0.2, 0.23, 0);
}

void keyboard_control(int key_control)
{ //uses the returned value gicen by the keyboard capture to control the robot
    if (key_control != -1)
    {

        switch (key_control)
        {
        case 68:
            //base_reset();
            // step();
            base_turn_left();
            printf("turn left \n");
            currentmove = TURNL;
            break;
        case 87:
            base_forwards();
            printf("go forward \n");
            currentmove = FORWARD;
            break;
        case 65:
            //base_reset();
            //step();
            base_turn_right();
            printf("turn right \n");
            currentmove = TURNR;
            break;
        case 83:
            base_backwards();
            printf("go backward \n");
            currentmove = BACKWARD;
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
            currentmove = STATIONARY;
            break;
        case 81:
            base_strafe_left();
            printf("strafe left \n");
            break;
        case 69:
            base_strafe_right();
            printf("strafe right \n");
            break;
        case 80:
            show_blobs = 1;
            printf("display enabled \n");
            break;
        case 76:
            autonomous = 1;
            printf("autonomous enabled \n");
            break;
        case 75:
            autonomous = 0;
            printf("autonomous disabled \n");
            break;

        }
    }
}