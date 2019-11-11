#ifndef MAP
#define MAP
#include <webots/robot.h>
#define floor_width 10
#define floor_length 10
#define resolution 2  // number of devisions per meter


struct memory{
int memory[floor_width*resolution][floor_width*resolution];
int width;
int length;
};
typedef struct memory Memory;
 

void store_in_memory(Memory* mem,float x,float y);
void set_memory(Memory *mem);

void store_in_memory(Memory *mem, float x, float y);

void clear_in_memory(Memory *mem, float x, float y);

void display_memory(WbDeviceTag display, Memory *mem, int d_width, int d_height);

void get_memory_cell(float x, float y, int *x_pos, int *y_pos);
    
void center_of_cell(int x_pos, int y_pos, float *x, float *y);


#endif