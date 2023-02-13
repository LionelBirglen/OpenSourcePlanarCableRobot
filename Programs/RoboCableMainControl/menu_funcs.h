#pragma once
#include "variables.h"
#include "InterpolationLib.h"

// read and store current values from the controllers into current1,current2,... variables
void read_currents(){
  valid = roboclaw.ReadCurrents(addressM1, current1, current0);
  compte = 0;
  while(!valid && compte < compte_timeout){
    valid = roboclaw.ReadCurrents(addressM1, current1, current0);
    compte++;
  }

  valid = roboclaw.ReadCurrents(addressM2, current2, current0);
  compte = 0;
  while(!valid && compte < compte_timeout){
    valid = roboclaw.ReadCurrents(addressM2, current2, current0);
    compte++;
  }

  valid = roboclaw.ReadCurrents(addressM3, current3, current0);
  compte = 0;
  while(!valid && compte < compte_timeout){
    valid = roboclaw.ReadCurrents(addressM3, current3, current0);
    compte++;
  }

  valid = roboclaw.ReadCurrents(addressM4, current4, current0);
  compte = 0;
  while(!valid && compte < compte_timeout){
    valid = roboclaw.ReadCurrents(addressM4, current4, current0);
    compte++;
  }
}

// read position values from controllers
void get_pos(int32_t steps[4]) {
  int32_t R_M1_step = roboclaw.ReadEncM1(addressM1, &status1, &valid1);
  compte = 0;
  while ((valid1 == 0) && (compte < compte_timeout)) {
    R_M1_step = roboclaw.ReadEncM1(addressM1, &status1, &valid1);
    compte = compte + 1;
  }

  int32_t R_M2_step = roboclaw.ReadEncM1(addressM2, &status1, &valid1);
  compte = 0;
  while ((valid1 == 0) && (compte < compte_timeout)) {
    R_M2_step = roboclaw.ReadEncM1(addressM2, &status1, &valid1);
    compte = compte + 1;
  }

  int32_t R_M3_step = roboclaw.ReadEncM1(addressM3, &status1, &valid1);
  compte = 0;
  while ((valid1 == 0) && (compte < compte_timeout)) {
    R_M3_step = roboclaw.ReadEncM1(addressM3, &status1, &valid1);
    compte = compte + 1;
  }

  int32_t R_M4_step = roboclaw.ReadEncM1(addressM4, &status1, &valid1);
  compte = 0;
  while ((valid1 == 0) && (compte < compte_timeout)) {
    R_M4_step = roboclaw.ReadEncM1(addressM4, &status1, &valid1);
    compte = compte + 1;
  }

  steps[0] = R_M1_step;
  steps[1] = R_M2_step;
  steps[2] = R_M3_step;
  steps[3] = R_M4_step;
}

// read speed values from controllers
void get_speed(int32_t speeds[4]) {
  int32_t R_M1_V = roboclaw.ReadSpeedM1(addressM1, &status1, &valid1);
  compte = 0;
  while ((valid1 == 0) && (compte < compte_timeout)) {
    R_M1_V = roboclaw.ReadSpeedM1(addressM1, &status1, &valid1);
    compte = compte + 1;
  }

  int32_t R_M2_V = roboclaw.ReadSpeedM1(addressM2, &status1, &valid1);
  compte = 0;
  while ((valid1 == 0) && (compte < compte_timeout)) {
    R_M2_V = roboclaw.ReadSpeedM1(addressM2, &status1, &valid1);
    compte = compte + 1;
  }

  int32_t R_M3_V = roboclaw.ReadSpeedM1(addressM3, &status1, &valid1);
  compte = 0;
  while ((valid1 == 0) && (compte < compte_timeout)) {
    R_M3_V = roboclaw.ReadSpeedM1(addressM3, &status1, &valid1);
    compte = compte + 1;
  }

  int32_t R_M4_V = roboclaw.ReadSpeedM1(addressM4, &status1, &valid1);
  compte = 0;
  while ((valid1 == 0) && (compte < compte_timeout)) {
    R_M4_V = roboclaw.ReadSpeedM1(addressM4, &status1, &valid1);
    compte = compte + 1;
  }

  speeds[0] = R_M1_V;
  speeds[1] = R_M2_V;
  speeds[2] = R_M3_V;
  speeds[3] = R_M4_V;
}

// print the interpolated position and speed values
void print_Ivalues(float tc, int32_t M_step[], int32_t M_V[]){
    // print interpolated steps and velocity
    Serial.print("[");Serial.print(tc);Serial.print(", ");
    Serial.print(M_step[0]);Serial.print(", ");Serial.print(M_step[1]);Serial.print(", ");
    Serial.print(M_step[2]);Serial.print(", ");Serial.print(M_step[3]);Serial.print(", ");
    Serial.print(M_V[0]);Serial.print(", ");Serial.print(M_V[1]);Serial.print(", ");
    Serial.print(M_V[2]);Serial.print(", ");Serial.print(M_V[3]);Serial.println("],"); 
}

// print the real (measured) position and speed values
void print_Rvalues(float tc){
    int32_t steps[4], speeds[4];
    get_pos(steps);
    get_speed(speeds);
    // print real steps and velocity
    Serial.print("["); Serial.print(tc); Serial.print(", ");
    Serial.print(steps[0]); Serial.print(", "); Serial.print(steps[1]); Serial.print(", ");
    Serial.print(steps[2]); Serial.print(", "); Serial.print(steps[3]); Serial.print(", ");
    Serial.print(speeds[0]); Serial.print(", "); Serial.print(speeds[1]); Serial.print(", ");
    Serial.print(speeds[2]); Serial.print(", "); Serial.print(speeds[3]); Serial.println("],");  
}

// go through the trajectory step by step automatically
void run_trajectory() {
  Serial.println("Run Trajectory");

  for (int i = 0; i < Traj2Fin; i++) {
    ismoving = true;
    roboclaw.SpeedAccelDeccelPositionM1(addressM1, RCaccel, Vit2[i][0], RCdecel, Traj2[i][0], buffer_flag);
    roboclaw.SpeedAccelDeccelPositionM1(addressM2, RCaccel, Vit2[i][1], RCdecel, Traj2[i][1], buffer_flag);
    roboclaw.SpeedAccelDeccelPositionM1(addressM3, RCaccel, Vit2[i][2], RCdecel, Traj2[i][2], buffer_flag);
    roboclaw.SpeedAccelDeccelPositionM1(addressM4, RCaccel, Vit2[i][3], RCdecel, Traj2[i][3], buffer_flag);
    
    read_currents();

    RCpos1 = Traj2[i][0]; RCpos2 = Traj2[i][1]; RCpos3 = Traj2[i][2]; RCpos4 = Traj2[i][3];

    delay(1.5*mydelay); // delay so that the motors have time to reach position
    Serial.print("Waypoint Done: " ); Serial.println(i);
    Serial.print("current 1 : " ); Serial.print(0.01*current1);
    Serial.print("    current 2 : " ); Serial.print(0.01*current2);
    Serial.print("    current 3 : " ); Serial.print(0.01*current3);
    Serial.print("    current 4 : " ); Serial.println(0.01*current4);
    Serial.println("\n");

  }

  delay(1000);
  Serial.println("Done"); Serial.println("");
}

// run the input step of the trajectory
void run_step(int step) {
  
  ismoving = true;
  Serial.print("Run Trajectory step "); Serial.println(step);
  roboclaw.SpeedAccelDeccelPositionM1(addressM1, RCaccel, Vit2[step][0], RCdecel, Traj2[step][0], buffer_flag);
  roboclaw.SpeedAccelDeccelPositionM1(addressM2, RCaccel, Vit2[step][1], RCdecel, Traj2[step][1], buffer_flag);
  roboclaw.SpeedAccelDeccelPositionM1(addressM3, RCaccel, Vit2[step][2], RCdecel, Traj2[step][2], buffer_flag);
  roboclaw.SpeedAccelDeccelPositionM1(addressM4, RCaccel, Vit2[step][3], RCdecel, Traj2[step][3], buffer_flag);
  RCpos1 = Traj2[step][0];
  RCpos2 = Traj2[step][1];
  RCpos3 = Traj2[step][2];
  RCpos4 = Traj2[step][3];
  delay(mydelay);
}

// function to move motors to the desired position 
// takes in an array of the 4 desired step values of the motors
void go_to_step(int32_t steps[]) {
  int cspeed = 8000;
  ismoving = true;
  roboclaw.SpeedAccelDeccelPositionM1(addressM1, RCaccel, cspeed, RCdecel, steps[0], buffer_flag);
  roboclaw.SpeedAccelDeccelPositionM1(addressM2, RCaccel, cspeed, RCdecel, steps[1], buffer_flag);
  roboclaw.SpeedAccelDeccelPositionM1(addressM3, RCaccel, cspeed, RCdecel, steps[2], buffer_flag);
  roboclaw.SpeedAccelDeccelPositionM1(addressM4, RCaccel, cspeed, RCdecel, steps[3], buffer_flag);
  RCpos1 = steps[0];
  RCpos2 = steps[1];
  RCpos3 = steps[2];
  RCpos4 = steps[3];
}

// prints out the position values of the motors
void angle_measure() {
  Serial.println("Motor Angles in software:");
  Serial.print("M1: "); Serial.print(RCpos1, DEC); Serial.print(" angle : "); Serial.println(RCpos1*360/6533, DEC);
  Serial.print("M2: "); Serial.print(RCpos2, DEC); Serial.print(" angle : "); Serial.println(RCpos2*360/6533, DEC);
  Serial.print("M3: "); Serial.print(RCpos3, DEC); Serial.print(" angle : "); Serial.println(RCpos3*360/6533, DEC);
  Serial.print("M4: "); Serial.print(RCpos4, DEC); Serial.print(" angle : "); Serial.println(RCpos4*360/6533, DEC);
  Serial.println("Motor Angles in hardware:");
  temp = roboclaw.ReadEncM1(addressM1, &status1, &valid1);
  compte = 0;
  while ((valid1 == 0) && (compte < compte_timeout)) {
    temp = roboclaw.ReadEncM1(addressM1, &status1, &valid1);
    compte = compte + 1;
  }
  Serial.print("M1: "); Serial.print((int)temp, DEC); Serial.print("   Status: "); Serial.print(status1, DEC); Serial.print("   Valid: "); Serial.print(valid1, DEC); Serial.println("");
  temp = roboclaw.ReadEncM1(addressM2, &status2, &valid2);
  compte = 0;
  while ((valid2 == 0) && (compte < compte_timeout)) {
    temp = roboclaw.ReadEncM1(addressM2, &status2, &valid2);
    compte = compte + 1;
  }
  Serial.print("M2: "); Serial.print((int)temp, DEC); Serial.print("   Status: "); Serial.print(status2, DEC); Serial.print("   Valid: "); Serial.print(valid2, DEC); Serial.println("");
  temp = roboclaw.ReadEncM1(addressM3, &status3, &valid3);
  compte = 0;
  while ((valid3 == 0) && (compte < compte_timeout)) {
    temp = roboclaw.ReadEncM1(addressM3, &status3, &valid3);
    compte = compte + 1;
  }
  Serial.print("M3: "); Serial.print((int)temp, DEC); Serial.print("   Status: "); Serial.print(status3, DEC); Serial.print("   Valid: "); Serial.print(valid3, DEC); Serial.println("");
  temp = roboclaw.ReadEncM1(addressM4, &status4, &valid4);
  compte = 0;
  while ((valid4 == 0) && (compte < compte_timeout)) {
    temp = roboclaw.ReadEncM1(addressM4, &status4, &valid4);
    compte = compte + 1;
  }
  Serial.print("M4: "); Serial.print((int)temp, DEC); Serial.print("   Status: "); Serial.print(status4, DEC); Serial.print("   Valid: "); Serial.print(valid4, DEC); Serial.println("");
  Serial.println();
  Serial.print("Position step (software): "); Serial.println(RCsteps, DEC);
  Serial.print("Speed (software): "); Serial.println(RCspeed, DEC);
  //Serial.print("Speed step (software): ");Serial.println(speedsteps,DEC);
  Serial.println("Done"); Serial.println("");
}

void zero_software() {
  Serial.println("Set zero position in software");
  RCpos1 = 0;RCpos2 = 0;
  RCpos3 = 0;RCpos4 = 0;
  Serial.println("Done"); Serial.println("");
}

void zero_hardware() {
  Serial.println("Set zero position in hardware");
  roboclaw.ResetEncoders(addressM1);
  roboclaw.ResetEncoders(addressM2);
  roboclaw.ResetEncoders(addressM3);
  roboclaw.ResetEncoders(addressM4);
  Serial.println("Done"); Serial.println("");
}

void go_to_zero() {
  Serial.println("Go to zero position");
  roboclaw.SpeedAccelDeccelPositionM1(addressM1, RCaccel, 0, RCdecel, 0, buffer_flag);
  roboclaw.SpeedAccelDeccelPositionM1(addressM2, RCaccel, 0, RCdecel, 0, buffer_flag);
  roboclaw.SpeedAccelDeccelPositionM1(addressM3, RCaccel, 0, RCdecel, 0, buffer_flag);
  roboclaw.SpeedAccelDeccelPositionM1(addressM4, RCaccel, 0, RCdecel, 0, buffer_flag);
  Serial.println("Done"); Serial.println("");
  IndexTraj2 = 0;
  delay(2000);
}

// run interpolated trajectory
void interp_trajectory() {
  Serial.println("Run Interpolated Trajectory");
  float Tf = Step_times[N_steps - 1];
  int32_t M_step[4];
  int32_t M_V[4];

  float t0 = millis();
  float tc = 0.001 * (millis() - t0);

  while (tc < Tf) {
    tc = 0.001 * (millis() - t0);

    M_step[0] = Interpolation::Linear(Step_times, M1_steps, N_steps, tc, true);
    M_step[1] = Interpolation::Linear(Step_times, M2_steps, N_steps, tc, true);
    M_step[2] = Interpolation::Linear(Step_times, M3_steps, N_steps, tc, true);
    M_step[3] = Interpolation::Linear(Step_times, M4_steps, N_steps, tc, true);

    M_V[0] = Interpolation::Linear(Step_times, M1_V_steps, N_steps, tc, true);
    M_V[1] = Interpolation::Linear(Step_times, M2_V_steps, N_steps, tc, true);
    M_V[2] = Interpolation::Linear(Step_times, M3_V_steps, N_steps, tc, true);
    M_V[3] = Interpolation::Linear(Step_times, M4_V_steps, N_steps, tc, true);

    ismoving = true;
    
    roboclaw.SpeedAccelDeccelPositionM1(addressM1, RCaccel, M_V[0], RCdecel, M_step[0], buffer_flag);
    roboclaw.SpeedAccelDeccelPositionM1(addressM2, RCaccel, M_V[1], RCdecel, M_step[1], buffer_flag);
    roboclaw.SpeedAccelDeccelPositionM1(addressM3, RCaccel, M_V[2], RCdecel, M_step[2], buffer_flag);
    roboclaw.SpeedAccelDeccelPositionM1(addressM4, RCaccel, M_V[3], RCdecel, M_step[3], buffer_flag);
    
    RCpos1 = M_step[0]; RCpos2 = M_step[1]; RCpos3 = M_step[2]; RCpos4 = M_step[3];
    
    // print_Rvalues(tc);
    // print_Ivalues(tc,M_step,M_V);

  }
  delay(1000);
  Serial.println("Done\n");
}
