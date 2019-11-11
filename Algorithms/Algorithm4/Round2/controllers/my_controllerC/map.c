#include "map.h"
#include <webots/robot.h>
#include <webots/display.h>
#include <math.h>

void set_memory(Memory *mem)
{
    mem->width = floor_width * resolution;
    mem->length = floor_length * resolution;
    for (int i = 0; i < mem->length; i++)
    {
        for (int j = 0; j < mem->width; j++)
        {
            mem->memory[i][j] = 0;
        }
    }
}

void store_in_memory(Memory *mem, float x, float y)
{
    int x_pos = roundf((floor_width/2)*resolution + x * resolution);
    int y_pos = roundf((floor_length/2)*resolution + y * resolution);
    printf("%d,%d \n", x_pos, y_pos);
    if (x_pos > 0 && x_pos < floor_width * resolution && y_pos > 0 && y_pos < floor_length * resolution)
    {
        mem->memory[y_pos][x_pos] = 1;
    }
}

void clear_in_memory(Memory *mem, float x, float y)
{
    int x_pos = roundf((floor_width/2)*resolution + x * resolution);
    int y_pos = roundf((floor_length/2)*resolution + y * resolution);
    if (x_pos > 0 && x_pos < floor_width * resolution && y_pos > 0 && y_pos < floor_length * resolution)
    {
        mem->memory[y_pos][x_pos] = 0;
    }
}

void display_memory(WbDeviceTag display, Memory *mem, int d_width, int d_height)
{
    for (int i = 0; i < mem->length; i++)
    {
        for (int j = 0; j < mem->width; j++)
        {
            if (mem->memory[i][j] == 1)
            {
                wb_display_set_color(display, 0xffffff);
                wb_display_fill_rectangle(display, j * (d_width / (floor_width*resolution)), i * (d_height / (floor_length*resolution)), d_width / (floor_width*resolution), d_height / (floor_length*resolution));
            }
            else
            {
                wb_display_set_color(display, 0x000000);
                wb_display_fill_rectangle(display, j * (d_width / (floor_width*resolution)), i * (d_height / (floor_length*resolution)), d_width / (floor_width*resolution), d_height / (floor_length*resolution));
            }
            wb_display_set_color(display, 0xff00ff);
            wb_display_draw_rectangle(display, j * (d_width / (floor_width*resolution)), i * (d_height / (floor_length*resolution)), d_width / (floor_width*resolution), d_height / (floor_length*resolution));
        }
    }
}

void get_memory_cell(float x, float y, int *x_pos, int *y_pos)
{
    *x_pos = roundf(floor_width + x * resolution);
    *y_pos = roundf(floor_length + y * resolution);
}

void center_of_cell(int x_pos, int y_pos, float *x, float *y)
{
    *x = (x_pos - floor_width)/resolution;
    *y = (y_pos - floor_length)/resolution;

}