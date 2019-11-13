#include <webots/robot.h>
#include <webots/range_finder.h>
#include <webots/display.h>
#include <base.h>
#include "Team404_imagerecognition.h"
#include "Team404_basic_algorithm.h"
#include <stdio.h>

#include "Team404_range.h"
float get_range_finder_image(WbDeviceTag rangefinder, int width, int height)
{
    const float *rimage = wb_range_finder_get_range_image(rangefinder);
    float distance = 0;
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            float d_pixel = wb_range_finder_image_get_depth(rimage, width, x, y);
            distance += d_pixel;
        }
    }
    float av_distance = distance / (height * width);
    return av_distance;
}

int get_close_pixel_no(WbDeviceTag rangefinder, int width, int height)
{
    const float *rimage = wb_range_finder_get_range_image(rangefinder);
    int no_black = 0;
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            float d_pixel = wb_range_finder_image_get_depth(rimage, width, x, y);
            if (d_pixel < 0.1)
            {
                no_black += 1;
            }
        }
    }
    return no_black;
}

void filter_range_image(int *filter_array, WbDeviceTag rangefinder, int width, int height, float filter_depth)
{
    const float *rimage = wb_range_finder_get_range_image(rangefinder);
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            filter_array[y * height + x] = (wb_range_finder_image_get_depth(rimage, width, x, y) < filter_depth) ? 1 : 0;
            if(y > height*5/6){
                filter_array[y*height+x] = 0;
            }
        }
    }
}

void display_show_range(int *filter_array, int width, int height, WbDeviceTag display)
{
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            if (filter_array[y * height + x] == 1)
            {
                wb_display_set_color(display, 0xffffff);
            }
            else
            {
                wb_display_set_color(display, 0x000000);
            }
            wb_display_draw_pixel(display, x, y);
        }
    }
}


int check_sides(Blob *r_blob_array,int rf_width,int rf_height)
{   
    Blob r_largest_blob;
    for (int i = 0; i < 50; i++)
    {
        if (r_blob_array[i].initialized == 0)
            break;
        find_largest_blob(&r_largest_blob,r_blob_array,50);
        printf("height:%d,corner_y:%d\n",r_largest_blob.height,r_largest_blob.corner_y);

        if(r_largest_blob.height<70 && r_largest_blob.height > 50 && r_largest_blob.corner_y > 100){
            if(r_largest_blob.width/2+r_largest_blob.corner_x >2*rf_width/3){
                base_turn_left();
                return 1;
            }
            if(r_largest_blob.width/2+r_largest_blob.corner_x < 1*rf_width/3){
                base_turn_right();
                return 1;
            }
        }

    }
    return 0;
}

float singed_distance(int* filter_array,int width,int height){
    //const float *rimage = wb_range_finder_get_range_image(rangefinder);
    
    float result = 0;
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            result += filter_array[y*height+x]*(x - width/2); 
        }
    }
return result;
}
