#ifndef KEYBOARD_CONTROL
#define KEYBOARD_CONTROL

enum MOVES
{   
    FORWARD,
    BACKWARD,
    TURNR,
    TURNL,
    STATIONARY
};
typedef enum MOVES moves;
moves currentmove;
int show_blobs;
int autonomous;
void print_instructions();
void tap();
void untap();
void keyboard_control(int key_control);

#endif 