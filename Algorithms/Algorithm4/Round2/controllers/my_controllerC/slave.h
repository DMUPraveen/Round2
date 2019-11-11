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
    int blob_count;
    int max_blob_count;
    float max_blob_count_UV_velocity[2];
};

struct slave1 Slave1;

struct slave2{
    int status;//0-running 1-done
    float target_uv_velocity[2];
    float turn_direction;

};
struct slave2 Slave2;

struct slave3{
    float initial_uv_vector[2];
    float search_angle;
    float best_uv_vector[2];
    int sweeps;
    int status; //0-doing, 1-done_sweeping,2-setted to best position
    int best;
};
struct slave3 Slave3;


struct slave4{
    int map;
    int status;
    float target_vector[2];
    float shortest;
    //float current_vector[2];
};
struct slave4 Slave4;

void run_slave1(Blob *largest_blob,Blob* blob_array,float* uv_velocity);
void run_slave0(Blob *largest_blob, int width, int height);
void setup_slave_1();
void setup_slave_0();
void reset_slave_variables(int slave_id);
void run_slave2(float* current_uv_vector);
void setup_slave_2();
void run_slave_3(Blob *blob_array, int blob_count, float *current_position,Blob* largest_blob);
void setup_slave_3();
#endif