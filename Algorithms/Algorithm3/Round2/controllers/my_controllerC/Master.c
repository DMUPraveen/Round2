////////////////////////////Master_Functions//////////////////////
#include "Master.h"
#include "slave.h"

void relieve_all_slaves()
{
    for (int i = 0; i < no_slaves; i++)
    {
        slave_status[i] = 0;
    }
}

void initialize_and_setup_slave(int slave_id)
{ ///slave_id geos from 0 to no_slaves
    slave_status[slave_id] = 1;
    reset_slave_variables(slave_id);
}

void revoke_slave(int slave_id){
    slave_status[slave_id] = 0;
}