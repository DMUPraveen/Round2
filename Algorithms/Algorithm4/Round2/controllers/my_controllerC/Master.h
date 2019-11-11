#ifndef MASTER
#define MASTER
#define no_slaves 5


//array which stores whether a slave is active or not
int slave_status[no_slaves];
void revoke_slave(int slave_id);
void initialize_and_setup_slave(int slave_id);
void relieve_all_slaves();
///0-inactive 1-active

#endif