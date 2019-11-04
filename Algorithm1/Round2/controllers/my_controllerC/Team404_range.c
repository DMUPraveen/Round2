#include <webots/robot.h>
#include <webots/range_finder.h>
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
            if(d_pixel < 0.1){
                no_black +=1;
            }
        }
    }
    return no_black;
}
