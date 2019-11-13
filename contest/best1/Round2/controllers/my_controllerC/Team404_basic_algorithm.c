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

#include <arm.h>
#include <base.h>
#include <gripper.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
void find_largest_blob(Blob *largest_blob, Blob *blob_array, int blob_count)
{
    int largest_height = 0;
    for (int i = 0; i < blob_count; i++)
    {
        if (largest_height < blob_array[i].height)
        {
            largest_height = blob_array[i].height;
            largest_blob->width = blob_array[i].width;
            largest_blob->height = blob_array[i].height;
            largest_blob->corner_x = blob_array[i].corner_x;
            largest_blob->corner_y = blob_array[i].corner_y;
        }
    }
}
void go_to_largest_blob(Blob *largest_blob, float error_boundary, int camera_width, int camera_height, int close_height, int* tapped)
{
    float x = (largest_blob->corner_x) + (largest_blob->width) / 2;

    float y = (largest_blob->corner_y) + (largest_blob->height) / 2;

    float c_x = camera_width / 2;
    float c_y = camera_width / 2;
    int height = largest_blob->height;
    //printf("%d \n",height);

    //if the blob is to the left turn right
    if (x < c_x - error_boundary && x != 0 && height < close_height)
    {
        base_turn_right();
        //printf("turn right \n");
    }
    //if the blob is to the right turn left
    if (x > c_x + error_boundary && x != 0 && height < close_height)
    {
        base_turn_left();
        //printf("turn left \n");
    }
    if (x > c_x - error_boundary && x < c_x + error_boundary && x != 0 && height < close_height)
    {
        base_forwards();
        //printf("forward \n");
    }
    if (x == 0)

    {
        base_turn_left();
        untap();
        *tapped = 0;
        //printf("seraching \n");
    }
    if (height > close_height)
    {
        base_reset();
        tap();
        *tapped = 1;
        //printf("tapping \n");
    }
    if (height < close_height)
    {
        untap();
        *tapped = 0;
    }
}
