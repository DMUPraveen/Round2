#ifndef RANGE_HELPER
#define RANGE_HELPER
#include <webots/robot.h>
float get_range_finder_image(WbDeviceTag rangefinder, int width, int height);
int get_close_pixel_no(WbDeviceTag rangefinder, int width, int height);
#endif