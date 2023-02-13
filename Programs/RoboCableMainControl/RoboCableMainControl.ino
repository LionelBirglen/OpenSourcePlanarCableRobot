//Includes required to use Roboclaw library
#include <SoftwareSerial.h>
#include "RoboClaw.h"
#include "menu_funcs.h"
#include "variables.h"
#include "com.h"

/*
int step_counter = 0;
int char_step_counter;
char step_msg[50];
int32_t step_mot[4] = {0,0,0,0};
bool read_step = false;

bool decode_step_msg() {

  if (Serial.available()) { // if there is a char

    char step_ch = Serial.read(); // read

    if(step_ch == 'g'){
      Serial.println("zero");
      go_to_zero();
    } 

    if (read_step) { // if we are currently reading a message, add the char to the message
      step_msg[char_step_counter] = step_ch;
      char_step_counter++; // move to the next letter of the message
    }

    if (step_ch == '[') { // if the char read is '#' it means it is the start of a message
      read_step = true; // we set the state as currently reading
      //for (int k = 0; k < msg_length-1; k++)
      
      step_msg[0] = '\0'; // we 'delete' the previous message by inserting an end of line char at the beginning
    }


    else if (step_ch == ']') { // if char = '%' then we reached the end of the message
      step_msg[char_step_counter - 1] = '\0'; // we set the end of the message with end of line char
      char_step_counter = 0; // we reset the position
      read_step = false; // we are no longer reading
      return true; // return true to say that a message has been received
    }
  }

  return false; // no message has been received
}

void mot_step(){
  if(decode_step_msg()){
    String msg = step_msg;

    int current_step = 0;
    int last_i = 0;
    for(int i = 0; i < msg.length(); i++){
      if(msg[i] == ';'){
        step_mot[current_step] = msg.substring(last_i,i).toInt();
        Serial.print(step_mot[current_step]);Serial.print(" ");
        last_i = i+1;
        current_step++;
      }
    }
    Serial.println("\n");

    int cspeed = 5000;
    ismoving = true;
    roboclaw.SpeedAccelDeccelPositionM1(addressM1, RCaccel, cspeed, RCdecel, step_mot[0], buffer_flag);
    roboclaw.SpeedAccelDeccelPositionM1(addressM2, RCaccel, cspeed, RCdecel, step_mot[1], buffer_flag);
    roboclaw.SpeedAccelDeccelPositionM1(addressM3, RCaccel, cspeed, RCdecel, step_mot[2], buffer_flag);
    roboclaw.SpeedAccelDeccelPositionM1(addressM4, RCaccel, cspeed, RCdecel, step_mot[3], buffer_flag);
    RCpos1 = step_mot[0];
    RCpos2 = step_mot[1];
    RCpos3 = step_mot[2];
    RCpos4 = step_mot[3];
  }
}
*/

//This is the first function arduino runs on reset/power up
void setup() {

  Serial.begin(57600); // serial with computer
  Serial1.begin(9600); // communication with slave arduino pro mini
  roboclaw.begin(38400); //Open Serial and roboclaw at 38400bps

  Serial.println("Starting...");
  delay(100);

  test_controllers();

  initialise_com();
  
  zero_software();
  zero_hardware();
  def_L0(); 

}

void loop() {

  //mot_step();

  if(read_serial())
    decode_msg(); // listening to the pro mini
}
