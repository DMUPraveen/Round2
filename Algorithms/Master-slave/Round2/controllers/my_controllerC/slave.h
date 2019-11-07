#ifndef SLAVE
#define SLAVE 

#include "Team404_basic_algorithm.h"


struct slave0{
    /////master_values
    int teleported; //1 if a box was succesfully teleported else 0

    ////slave_values
    int tapped; //1 if already tapped else 0;
};

struct slave0 Slave0;

struct slave1{
    int status;//0-still working, 1-found a suitably large object,2-done a full rotation
    int height_boundary;//the box height which is close enough
    float rot_angle;//the angle that it has curently rotated
    int max_height;//the maximux height it has observed uptil now
};

struct slave1 Slave1;

void run_slave1(Blob *largest_blob);
void run_slave0(Blob *largest_blob, int width, int height);
void setup_slave_1();
void setup_slave_0();
void reset_slave_variables(int slave_id);

#endif