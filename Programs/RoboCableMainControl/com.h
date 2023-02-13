#pragma once
#include "variables.h"
#include "menu_funcs.h"
#include "mgi_led.h"

// checking the controllers
void test_controllers(){
  char version[32];

  while (roboclaw.ReadVersion(addressM1, version) != true) {
    roboclaw.ReadVersion(addressM1, version);
  }
  Serial.print("Motor 1: "); Serial.println(version);
  while (roboclaw.ReadVersion(addressM2, version) != true) {
    roboclaw.ReadVersion(addressM2, version);
  }
  Serial.print("Motor 2: "); Serial.println(version);
  while (roboclaw.ReadVersion(addressM3, version) != true) {
    roboclaw.ReadVersion(addressM3, version);
  }
  Serial.print("Motor 3: "); Serial.println(version);
  while (roboclaw.ReadVersion(addressM4, version) != true) {
    roboclaw.ReadVersion(addressM4, version);
  }
  Serial.print("Motor 4: "); Serial.println(version);
  Serial.print("Done"); Serial.println("");
}

// reading the serial connection between the mega and the pro mini
// each command message starts with # and ends with %
bool read_serial() {

  if (Serial1.available()) { // if there is a char

    msg_ch = Serial1.read(); // read

    if (read) { // if we are currently reading a message, add the char to the message
      serial_msg[counter] = msg_ch;
      counter++; // move to the next letter of the message
    }

    if (msg_ch == '#') { // if the char read is '#' it means it is the start of a message
      read = true; // we set the state as currently reading
      //for (int k = 0; k < msg_length-1; k++)
      
      serial_msg[0] = '\0'; // we 'delete' the previous message by inserting an end of line char at the beginning
    }

    if (msg_ch == '%') { // if char = '%' then we reached the end of the message
      serial_msg[counter - 1] = '\0'; // we set the end of the message with end of line char
      counter = 0; // we reset the position
      read = false; // we are no longer reading
      return true; // return true to say that a message has been received
    }
  }

  return false; // no message has been received
}

// initialisation protocole so that the pro mini can get the number of steps in the trajectory
void initialise_com() {

  long tinit = millis();

  Serial.println("sending traj num");
  Serial1.println("#init%"); // every time the pro mini reads this message, it starts the initialisation protocole as well

  while (millis()-tinit < 3000) { // sending the message for 3 seconds to make sure the pro mini received it
    Serial1.print("#{");Serial1.print(Traj2Fin);Serial1.print("%"); // the '{' is for the pro mini to know taht this message contains the number of steps
    Serial1.print("#step_num_end%"); // indicating the end of transmision
  }


  while (strcmp(serial_msg, "maxstep_ok") != 0) { // waiting for a confirmation message from the pro mini
    read_serial();
  }

  Serial.println("ok !"); // end of initialisation
}

// reading the inputs in manual mode
void read_manual_input() {
  String msg = serial_msg; // converting to string because it's easier to work with
  if(msg.length() == 7){ // checking the length of the message which should be exactly 7

    int left = msg[4] == '1'; // left button state
    int right = msg[6] == '1'; // right button state
    int x = (msg[0] == '1') - (msg[0] == '2'); // if '1' then x = 1 else if '2' then x = -1
    int y = (msg[2] == '1') - (msg[2] == '2'); // same here

    // movement of 1cm between -0.3 and + 0.3 m
    x_pos = max(-0.3,min(0.3,x_pos + 0.01*x)); // incrementing the x and y position of 1 cm every time
    y_pos = max(-0.3,min(0.3,y_pos + 0.01*y));
    theta = min(50*PI/180,theta + right*10*PI/180); // add 10° if right pressed
    theta = max(-50*PI/180,theta - left*10*PI/180); // substract 10° if left pressed

    int32_t steps[4];
    get_steps_from_pos_anneau(x_pos,y_pos,steps); // getting the motor positions

    read_currents(); // reading the currents

    // sending steps and currents to pro mini for  display
    Serial1.print("#");
    Serial1.print(steps[0]);Serial1.print(";");
    Serial1.print(steps[1]);Serial1.print("%");
    Serial1.print("#");
    Serial1.print(steps[2]);Serial1.print(";");
    Serial1.print(steps[3]);Serial1.print("%");

    Serial1.print("#");
    Serial1.print(current1);Serial1.print(";");
    Serial1.print(current2);Serial1.print("%");
    Serial1.print("#");
    Serial1.print(current3);Serial1.print(";");
    Serial1.print(current4);Serial1.print("%");

    Serial1.print("#end%"); // end of message

    // printing information on computer
    Serial.print(x_pos);Serial.print(" ");Serial.println(y_pos);
    Serial.print(steps[0]);Serial.print(" ");
    Serial.print(steps[1]);Serial.print(" ");
    Serial.print(steps[2]);Serial.print(" ");
    Serial.println(steps[3]);
    Serial.println(" ");

    go_to_step(steps); // sending the position command to controllers
  }
}

// reading the messages coming from the pro mini
void decode_msg(){
  String msg = serial_msg;

  if(msg.substring(0,3)=="gtz"){ // go to zero command
    go_to_zero();
    x_pos = 0; y_pos = 0; theta = 0;
    Serial1.print("#end_gtz%");
  }

  else if(msg.substring(0,2)=="sz"){ // set zero command
    zero_software();
    zero_hardware();
    x_pos = 0; y_pos = 0; theta = 0;
  }

  else if(msg.substring(0,4)=="step"){ // step command indicates which step of the trajectory to run the message if #stepi% with i the number of the step
    int step = msg.substring(4,msg.length()).toInt();
    Serial.print("received step ");Serial.println(step); // confirming acquisition of the command
    

    run_step(step); // running the step
    read_currents(); // reading currents

    // sending the data to pro mini
    Serial1.print("#");
    Serial1.print(Traj2[step][0]);Serial1.print(";");
    Serial1.print(Traj2[step][1]);Serial1.print("%");
    Serial1.print("#");
    Serial1.print(Traj2[step][2]);Serial1.print(";");
    Serial1.print(Traj2[step][3]);Serial1.print("%");

    Serial1.print("#");
    Serial1.print(current1);Serial1.print(";");
    Serial1.print(current2);Serial1.print("%");
    Serial1.print("#");
    Serial1.print(current3);Serial1.print(";");
    Serial1.print(current4);Serial1.print("%");

    Serial1.print("#end%"); // end of transmision
  }

  else if(msg.substring(0,6)=="interp"){ // run interpolated trajectory
    Serial.println("received interp");
    go_to_zero(); // going to zero position because currently the trajectory starts at the origin
    interp_trajectory();
    Serial1.println("#interp_end%"); // sending end of mode command 
  }

  else if(msg.substring(0,4)=="auto"){ // run normal trajectory
    Serial.println("received auto");
    go_to_zero();
    run_trajectory();
    Serial1.println("#auto_end%");
  }

  else if(msg.substring(0,6)=="manual"){ // manual mode
    Serial.println("received manual");

    serial_msg[0] = '\0';
    while (strcmp(serial_msg, "end_manual") != 0){ // while not received and of mode message, we read the inputs and execute
      if(read_serial())
        read_manual_input();
    }

    Serial.println("end manual");
  }

}