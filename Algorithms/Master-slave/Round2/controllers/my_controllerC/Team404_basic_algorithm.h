#ifndef ALGORITHM
#define ALGORITHM

#include "Team404_imagerecognition.h"

void find_largest_blob(Blob *largest_blob, Blob *blob_array, int blob_count);
int go_to_largest_blob(Blob *largest_blob, float error_boundary, int camera_width, int camera_height, int close_height,int* tapped);
void initialize_slave(int slave_id,int slave[2],int slave_setup[2]);

void relieve_slave(int slave_id,int slave[2],int slave_setup[2]);

void relieve_all(int slave[2],int slave_setup[2]);
#endif