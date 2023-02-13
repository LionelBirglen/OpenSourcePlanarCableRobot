/*
functions used to calculate the inverse geometric model
of the robot for the manual mode

*/
#pragma once

float L = 1;
float H = 1;
float diag = 52e-3; // radius of the ring
float D_poulie = 65e-3; // fixed radius assumption

float E1_v[2] = {-diag*cos(PI/4),-diag*sin(PI/4)};
float E2_v[2] = { diag*cos(PI/4),-diag*sin(PI/4)};
float E3_v[2] = {-diag*cos(PI/4), diag*sin(PI/4)};
float E4_v[2] = { diag*cos(PI/4), diag*sin(PI/4)};

// positions of the cable ends on the base
float B1_v[2] = {-L/2,-H/2};
float B2_v[2] = { L/2,-H/2};
float B3_v[2] = {-L/2, H/2};
float B4_v[2] = { L/2, H/2};

float n[2]; 
float L10; float L20; 
float L30; float L40;

// returns norm of an array considered as a vector
float norm(float vec[]){
  return sqrt(pow(vec[0],2)+pow(vec[1],2));
}

// multiplies vec by scalar and returns it in vec2
void scalar_mult(float scalar, float vec[],float vec2[]){
  vec2[0] = scalar*vec[0];
  vec2[1] = scalar*vec[1];
}

// c = a - b
void substract(float a[],float b[],float c[]){
  c[0] = a[0]-b[0];
  c[1] = a[1]-b[1];
}

// c = a + b
void sum(float a[],float b[],float c[]){
  c[0] = a[0]+b[0];
  c[1] = a[1]+b[1];
}

// defining the lengths of the cables at the origin point (0,0)
void def_L0() {
  substract(E1_v, B1_v, n);  // Li0 = norm(pi0-Bi)
  L10 = norm(n);

  substract(E2_v, B2_v, n);
  L20 = norm(n);

  substract(E3_v, B3_v, n);
  L30 = norm(n);

  substract(E4_v, B4_v, n);
  L40 = norm(n);
}

// optimized version of the igm python code
void get_steps_from_pos_anneau(float x,float y, int32_t steps[]){
  float pos[2] = {x,y};

  //float n[2];
  /////// 1
  sum(pos,E1_v,n); // Pi is pos + Ei
  substract(n,B1_v,n); // p1-B1
  steps[0] = (6533/(2*PI))*2*(norm(n) - L10)/D_poulie;

  /////// 2
  sum(pos,E2_v,n);
  substract(n,B2_v,n);
  steps[1] = (6533/(2*PI))*2*(norm(n) - L20)/D_poulie;

  /////// 3
  sum(pos,E3_v,n);
  substract(n,B3_v,n);
  steps[2] = (6533/(2*PI))*2*(norm(n) - L30)/D_poulie;

  /////// 4
  sum(pos,E4_v,n);
  substract(n,B4_v,n);
  steps[3] = (6533/(2*PI))*2*(norm(n) - L40)/D_poulie;
}