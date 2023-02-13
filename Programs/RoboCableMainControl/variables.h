#pragma once
#include "interp_traj2.h"
#include "traj2.h"

//See limitations of Arduino SoftwareSerial
SoftwareSerial serial(10, 11);   //Ixpin=10, Txpin=11
RoboClaw roboclaw(&serial, 10000);

#define addressM1 0x80
#define addressM2 0x81
#define addressM3 0x82
#define addressM4 0x83

#define anglesteps (6533/360)*10
#define speedsteps 1000
#define positionsteps (6533/360)
#define speeddefault 2000
const int8_t compte_timeout = 100;
//const int32_t zerospeed = 20;

//uint32_t speedmax = 20000;
uint32_t stepsmax = 6533;

uint32_t RCaccel = 0;
uint32_t RCspeed = speeddefault;
uint32_t RCsteps = anglesteps;
uint32_t RCdecel = 0;
int32_t RCpos1 = 0;
int32_t RCpos2 = 0;
int32_t RCpos3 = 0;
int32_t RCpos4 = 0;
uint32_t temp = 0;
bool ismoving = true;

int16_t current0;
int16_t current1;
int16_t current2;
int16_t current3;
int16_t current4;

int IndexTraj2 = 0;

int buffer_flag = 1;
int mydelay = 250;

uint8_t status1, status2, status3, status4;
bool valid1, valid2, valid3, valid4;
uint8_t depth1, depth2;
bool valid;
uint8_t statustemp;
bool validtemp;
uint8_t compte;

char msg_ch;
const int msg_length = 20;
char serial_msg[msg_length];
bool read = false;
int counter = 0;

float x_pos; float y_pos;
float theta;

long last_manual_exec_time;
