#ifndef KEYBOARD_CONTROL
#define KEYBOARD_CONTROL

enum MOVES;
int show_blobs;
int autonomous;
void print_instructions();
void tap();
void untap();
void keyboard_control(int key_control);

#endif 