/*
functions used to calculate the inverse geometric model
of the robot for the manual mode

*/
#pragma once

float L = 1;
float H = 1;
float Ra = 10e-3; // radius of the ring
float D_poulie = 65e-3; // fixed radius assumption

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
  scalar_mult(Ra / norm(B1_v), B1_v, n);  //pi0 = 0 + Ra*ni0; ni0 = Bi/norm(Bi)
  substract(n, B1_v, n);                  // DL = norm(pi0-Bi)
  L10 = norm(n);

  scalar_mult(Ra / norm(B2_v), B2_v, n);
  substract(n, B2_v, n);
  L20 = norm(n);

  scalar_mult(Ra / norm(B3_v), B3_v, n);
  substract(n, B3_v, n);
  L30 = norm(n);

  scalar_mult(Ra / norm(B4_v), B4_v, n);
  substract(n, B4_v, n);
  L40 = norm(n);
}

// optimized version of the igm python code
void get_steps_from_pos_anneau(float x,float y, int32_t steps[]){
  float pos[2] = {x,y};

  //float n[2];
  /////// 1
  substract(B1_v,pos,n); // getting n
  scalar_mult(Ra/norm(n),n,n); // norming and multiplying by Ra
  sum(pos,n,n); // position is pos + RA*n
  substract(n,B1_v,n); // p1-B1
  steps[0] = (6533/(2*PI))*2*(norm(n) - L10)/D_poulie;

  /////// 2
  substract(B2_v,pos,n);
  scalar_mult(Ra/norm(n),n,n);
  sum(pos,n,n);
  substract(n,B2_v,n);
  steps[1] = (6533/(2*PI))*2*(norm(n) - L20)/D_poulie;

  /////// 3
  substract(B3_v,pos,n);
  scalar_mult(Ra/norm(n),n,n);
  sum(pos,n,n);
  substract(n,B3_v,n);
  steps[2] = (6533/(2*PI))*2*(norm(n) - L30)/D_poulie;

  /////// 4
  substract(B4_v,pos,n);
  scalar_mult(Ra/norm(n),n,n);
  sum(pos,n,n);
  substract(n,B4_v,n);
  steps[3] = (6533/(2*PI))*2*(norm(n) - L40)/D_poulie;
}