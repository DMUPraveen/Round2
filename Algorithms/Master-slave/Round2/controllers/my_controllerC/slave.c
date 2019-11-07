#include "slave.h"
#include "Master.h"
#include "Team404_basic_algorithm.h"
#include "Tracker.h"
#include "math.h"
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
    Slave1.height_boundary = 20;
    Slave1.rot_angle = 0;
    Slave1.max_height = 0;
}

void run_slave0(Blob *largest_blob, int width, int height)
{
    if (slave_status[0])
    {   //printf("%d \n",largest_blob -> height);

        Slave0.teleported = go_to_largest_blob(largest_blob, width / 6, width, height, 52, &Slave0.tapped);
        //printf("%d\n",Slave0.teleported);
        //printf("%d \n",Slave0.tapped);
    }
}

void run_slave1(Blob *largest_blob)
{
    if (slave_status[1])
    {   //printf("running slave1 \n");
        if (largest_blob->height >= Slave1.height_boundary){
            printf("large blob detected \n");
            Slave1.status = 1; //means a large box was found
        }
        else
        {
            base_turn_left();
            Slave1.rot_angle += ANGULAR_VELOCITY * T_STEP;
            //printf("%f\n",Slave1.rot_angle);
            if (Slave1.max_height < largest_blob->height)
                Slave1.max_height = largest_blob->height;
            if(Slave1.rot_angle > (2 * M_PI+M_PI/5)){
                Slave1.status = 2;
            }
        }

    }
}