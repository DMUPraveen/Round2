#ifndef ALGORITHM
#define ALGORITHM

#include "Team404_imagerecognition.h"

void find_largest_blob(Blob *largest_blob, Blob *blob_array, int blob_count);
void go_to_largest_blob(Blob *largest_blob, float error_boundary, int camera_width, int camera_height, int close_height,int* tapped);

#endif