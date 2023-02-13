#pragma once
#include <LiquidCrystal.h> // includes the LiquidCrystal Library 

int joystick_x = 0;
int joystick_y = 1;
int joystick_button = 9;
int button_left = 10;
int button_right = 11;
int button_gtz = 13; // go to zero
int button_sz = 12; // set zero

int rs = 2;
int enable = 3;
int d4 = 4;
int d5 = 5;
int d6 = 6;
int d7 = 7;
int contrast = 8;

LiquidCrystal lcd(rs, enable, d4, d5, d6, d7);

int max_step;
int step_num = 0;

bool screen_update = true;

unsigned long button_gtz_last_time;
int button_gtz_last_read;

unsigned long button_sz_last_time;
int button_sz_last_read;

unsigned long button_left_last_time;
int button_left_last_read;

unsigned long button_right_last_time;
int button_right_last_read;

unsigned long joystick_last_time;
unsigned long joystick_xy_last_time;
int joystick_button_last_read;

int ymin = 300;int ymax = 700; // joystick detection limits
int xmin = 300;int xmax = 700;

int debounce_delay = 250;

int mode = 0; // 0 : step_by_step;  1 : Auto;  2 : auto_interpolated;  3: manual
char *modes[] = {"Step by step", "Auto", "Auto interpolated","Manual"};
int num_modes = sizeof(modes)/sizeof(modes[0]);
bool mode_selected = false;

const int msg_length = 20;
char serial_msg[msg_length];
char msg_ch;
bool read = false;
int counter = 0;

int32_t data[8]; // steps + currents
