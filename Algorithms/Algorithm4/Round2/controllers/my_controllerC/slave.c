#include "slave.h"
#include "Master.h"
#include "Team404_basic_algorithm.h"
#include "Tracker.h"
#include "math.h"
#include "Team404_helper.h"
#include "Map.h"

#include <stdio.h>
void reset_slave_variables(int slave_id)
{
    switch (slave_id)
    {
    case 0:
        setup_slave_0();
        break;
    case 1:
        setup_slave_1();
        break;
    case 2:
        setup_slave_2();
        break;
    case 3:
        setup_slave_3();
        break;
    case 4:
        setup_slave_4();
        break;
    }
}

void setup_slave_0()
{
    Slave0.teleported = 0;
    Slave0.tapped = 0;
}

void setup_slave_1()
{
    Slave1.status = 0;
    Slave1.height_boundary = 19;
    Slave1.rot_angle = 0;
    Slave1.max_height = 0;
    Slave1.max_blob_count = 0;
    Slave1.max_blob_count_UV_velocity[0] = 0;
    Slave1.max_blob_count_UV_velocity[1] = 1;
}

void setup_slave_2()
{
    Slave2.status = 0;
    Slave2.target_uv_velocity[0] = 0;
    Slave2.target_uv_velocity[0] = 1;
    Slave2.turn_direction = 1;
}

void setup_slave_3()
{
    Slave3.status = 0;
    Slave3.initial_uv_vector[0] = 0;
    Slave3.initial_uv_vector[1] = 0;
    Slave3.best_uv_vector[0] = 0;
    Slave3.best_uv_vector[1] = 1;
    Slave3.search_angle = 0.3;
    Slave3.sweeps = 0;
    Slave3.best = 0;
}
void setup_slave_4()
{
    Slave4.status = 0;
    Slave4.map = 0;
    Slave4.shortest = 100;
    Slave4.target_vector[0] = 0;
    Slave4.target_vector[1] = 0;
}

void run_slave0(Blob *largest_blob, int width, int height)
{
    if (slave_status[0])
    { //printf("%d \n",largest_blob -> height);
        Slave0.teleported = go_to_largest_blob(largest_blob, width / 6, width, height, 52, &Slave0.tapped);
        //printf("teleported: %d\n",Slave0.teleported);
        //printf("%d\n",Slave0.teleported);
        //printf("%d \n",Slave0.tapped);
    }
}

void run_slave1(Blob *largest_blob, Blob *blob_array, float *uv_velocity)
{
    if (slave_status[1])
    { //printf("running slave1 \n");
        Slave1.blob_count = get_blob_count(blob_array, 50);
        if (largest_blob->height >= Slave1.height_boundary)
        {
            printf("large blob detected \n");
            Slave1.status = 1; //means a large box was found
        }
        else
        {
            turn_left();
            Slave1.rot_angle += ANGULAR_VELOCITY * T_STEP;
            //printf("%f\n",Slave1.rot_angle);
            if (Slave1.max_blob_count < Slave1.blob_count)
            {
                Slave1.max_blob_count = Slave1.blob_count;
                Slave1.max_blob_count_UV_velocity[0] = uv_velocity[0];
                Slave1.max_blob_count_UV_velocity[1] = uv_velocity[1];
            }
            if (Slave1.rot_angle > (2 * M_PI + M_PI / 5))
            {
                Slave1.status = 2;
            }
        }
    }
}

void run_slave2(float *current_uv_vector)
{
    if (slave_status[2] == 1)
    {
        if (Slave2.turn_direction == 1)
        {
            turn_right();
        }
        else
        {
            turn_left();
        }
        if (get_angle(current_uv_vector, Slave2.target_uv_velocity) < 0.01)
        {
            Slave2.status = 1;
        }
        else
        {
            Slave2.status = 0;
        }
    }
}

void run_slave_3(Blob *blob_array, int blob_count, float *current_position, Blob *largest_blob)
{
    //printf("hello\n");
    if (slave_status[3] == 1)
    {
        untap();

        //printf("%f\n", get_angle(current_position, Slave3.initial_uv_vector));

        if (Slave3.status == 0)

        {
            if (!get_angle(current_position, Slave3.best_uv_vector))
            {
                turn_left();
            }
            if (get_angle(current_position, Slave3.initial_uv_vector) < Slave3.search_angle)
            {
                if (Slave3.sweeps == 0)
                {
                    //printf("turning right\n");
                    turn_left();
                }
                if (Slave3.sweeps == 1)
                {
                    //printf("turning left\n");
                    turn_right();
                }
            }

            if (Slave3.sweeps == 0 && get_angle(current_position, Slave3.initial_uv_vector) >= Slave3.search_angle)
            {
                Slave3.sweeps = 1;
                turn_right();
            }
            if (Slave3.sweeps == 1 && get_angle(current_position, Slave3.initial_uv_vector) >= Slave3.search_angle + 0.01)
            {
                Slave3.status = 1;
            }

            int parameter = 0;
            for (int i = 0; i < blob_count; i++)
            {
                if (blob_array[i].initialized == 0)
                {
                    break;
                }
                parameter += blob_array[i].height;
            }
            if (parameter > Slave3.best)
            {
                Slave3.best = parameter;
                Slave3.best_uv_vector[0] = current_position[0];
                Slave3.best_uv_vector[1] = current_position[1];
            }
        }
        if (Slave3.status == 1)
        {
            int turn_side = which_way_to_turn(current_position, Slave3.best_uv_vector);
            if (turn_side == -1)
            {
                turn_right();
            }
            else
            {
                turn_left();
            }
            if (get_angle(current_position, Slave3.best_uv_vector) < 0.01)
            {
                Slave3.status = 2;
            }
        }
        if (largest_blob->height > 20)
        {
            Slave3.status = 2;
        }
    }
}

void run_slave_4(Memory *mem, float current_Pos[2])
{
    if (Slave4.map == 0)
    {
        if (Slave4.target_vector[0] == 0 && Slave4.target_vector[1] == 0)
        {
            for (int i = 0; i < mem->length; i++)
            {
                for (int j = 0; j < mem->width; j++)
                {
                    float vector[2];
                    center_of_cell(i, j, &vector[0], &vector[1]);
                    float vec_differnce[2];
                    for (int n = 0; n < 2; n++)
                    {
                        vec_differnce[n] = vector[n] - current_Pos[n];
                    }
                    if (magnitude(vec_differnce) < Slave4.shortest)
                    {
                        Slave4.target_vector[0] = vector[0];
                        Slave4.target_vector[1] = vector[1];
                        Slave4.shortest = magnitude(vec_differnce);
                    }
                }
            }
        }

        if (which_way_to_turn(current_Pos, Slave4.target_vector) == 1)
        {
            turn_right();
        }
        else
        {
            turn_left();
        }
        if (get_angle(Slave4.target_vector, current_Pos) < 0.01)
        {
            Slave4.status = 1;
        }
        else
        {
            Slave4.status = 0;
        }
    }
}