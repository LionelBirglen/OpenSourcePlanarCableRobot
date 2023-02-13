#pragma once
#include "funcs.h"

// function to decode incoming messages
// same principale as the one on the arduino mega
bool read_serial() {

  if (Serial.available()) {

    msg_ch = Serial.read();

    if (read) {
      serial_msg[counter] = msg_ch;
      counter++;
    }

    if (msg_ch == '#') {
      read = true;
      for (int k = 0; k < msg_length-1; k++)
        serial_msg[k] = '\0';
    }

    if (msg_ch == '%') {
      serial_msg[counter - 1] = '\0';
      counter = 0;
      read = false;
      return true;
    }
  }

  return false;
}

// initialisation 
void initialise_com() {
  bool ok = false;
  char num_msg[5];


  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("initialisation ...");
  
  while (!ok) { // while not received...
    serial_msg[0] = '\0'; // message cleared

    while (strcmp(serial_msg, "step_num_end")!=0) { // waiting for the step_num_end message
      if(read_serial()){ // if there is a message

        if(serial_msg[0] == '{'){ // if the first caracter is '{' it means that this message contains the number of steps
          String msg = serial_msg;
          max_step = msg.substring(1,msg.length()).toInt();
        }

      }
    }

    ok = true; // received the end message
  }

  delay(3000); // the mega keeps sending for 3s so we wait to be sure that he receives
  Serial.println("#maxstep_ok%"); // sending confirmation message 
  delay(1000);
}

