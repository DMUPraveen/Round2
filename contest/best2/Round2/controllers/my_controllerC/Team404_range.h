#ifndef RANGE_HELPER
#define RANGE_HELPER
#include <webots/robot.h>
float get_range_finder_image(WbDeviceTag rangefinder, int width, int height);
int get_close_pixel_no(WbDeviceTag rangefinder, int width, int height);
void filter_range_image(int* filter_array,WbDeviceTag rangefinder,int width,int height,float filter_depth);
void display_show_range(int* filter_array,int width,int height,WbDeviceTag display);
int check_sides(Blob *r_blob_array,int rf_width,int rf_height);
float singed_distance(int* filter_array,int rf_width,int rf_height);

#endif