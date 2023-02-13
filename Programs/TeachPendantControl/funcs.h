#pragma once
#include "inputs.h"
#include "com.h"

bool read_serial();

// setting the displayed data back to zero and also the step number
void clear_data(){
  data[0] = 0; data[1] = 0;
  data[2] = 0; data[3] = 0;
  data[4] = 0; data[5] = 0;
  data[6] = 0; data[7] = 0;
  step_num = 0;
}

// printing the infos on the step by step mode screen
void step_by_step_screen(){
  // printing the step number on line 1
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("step : ");
  lcd.setCursor(7, 0);
  lcd.print(step_num);
  
  // printing the motor steps on line 2 and 3
  lcd.setCursor(0, 1);
  lcd.print(data[0]);
  lcd.setCursor(10, 1);
  lcd.print(data[1]);
  lcd.setCursor(0, 2);
  lcd.print(data[2]);
  lcd.setCursor(10, 2);
  lcd.print(data[3]);

  // printing the motor currents on line 4
  // value is devided by 100 to obtain value in amps
  // only absolute value because not enough space
  lcd.setCursor(0, 3);
  lcd.print(abs(0.01*data[4]));
  lcd.setCursor(5, 3);
  lcd.print(abs(0.01*data[5]));
  lcd.setCursor(10, 3);
  lcd.print(abs(0.01*data[6]));
  lcd.setCursor(15, 3);
  lcd.print(abs(0.01*data[7]));
}

// manual mode screen display
void manual_screen(){
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("mode manual");
  
  // printing motor steps and currents like in the step by step mode
  lcd.setCursor(0, 1);
  lcd.print(data[0]);
  lcd.setCursor(10, 1);
  lcd.print(data[1]);
  lcd.setCursor(0, 2);
  lcd.print(data[2]);
  lcd.setCursor(10, 2);
  lcd.print(data[3]);

  lcd.setCursor(0, 3);
  lcd.print(abs(0.01*data[4]));
  lcd.setCursor(5, 3);
  lcd.print(abs(0.01*data[5]));
  lcd.setCursor(10, 3);
  lcd.print(abs(0.01*data[6]));
  lcd.setCursor(15, 3);
  lcd.print(abs(0.01*data[7]));
}

// functions that listens to the mega for the motor step and current messages
void get_step_current(){
  long tdeb = millis();
  serial_msg[0] = '\0';
  int msg_c = 0; // counter for the number of info received

  while (strcmp(serial_msg, "end")!=0 && millis()-tdeb < 500) { // waiting to receive the end message with a time limit 
    if(read_serial()){
      String msg = serial_msg;
      if(msg.length()!=0){
        //Serial.print("received : ");Serial.println(msg);
        int i = 0;
        while(msg[i] !=';' && i < msg.length()) // coutuing until we reach the separator "xxx;xxx"
          i++;
        
        data[0+msg_c*2] = msg.substring(0,i).toInt(); // first value of message
        data[1+msg_c*2] = msg.substring(i+1,msg.length()).toInt(); // second value of message
        msg_c++;
      }
    }        
  }
}


// step by step mode updates the position of the trajectory accodingto the user input
void step_by_step_mode() {

  
  if(read_joystick_button()){ // if joystick pressed we leave the mode
    mode_selected = false;
    screen_update = true;
    //clear_data();
  }

  else if(read_y_up()){ // y up means we add 1 to the current step
    step_num = (step_num+1)*(step_num+1<=max_step) + (step_num+1>max_step); // min = 1 ; max = max_step
    screen_update = true;
    Serial.print("#step");
    Serial.print(step_num-1); // sending step_num-1 to start a the first element of the trajectory
    Serial.println("%");

    get_step_current();// getting steps and currents from mega
  }

  else if(read_y_down()){ // same with y down
    step_num = (step_num-1)*(step_num-1>=1) + (max_step)*(step_num-1<1); // min = 1 ; max = max_step
    screen_update = true;
    Serial.print("#step");
    Serial.print(step_num-1);
    Serial.println("%");

    get_step_current();// getting steps and currents from mega
  }


}

// launching automatic trajectory running
void auto_mode(){
  Serial.println("#gtz%"); // going to zero first

  serial_msg[0] = '\0';
  while (strcmp(serial_msg, "end_gtz") != 0) // waiting for confirmation
    read_serial();
  
  Serial.println("#auto%"); // launching the auto mode
  serial_msg[0] = '\0';
  while (strcmp(serial_msg, "auto_end") != 0) // waiting for the end
    read_serial();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("auto end"); // end message for 3s
  delay(3000);

  Serial.println("#gtz%"); // going back to zero in case the trajectory does not end at the origin
  serial_msg[0] = '\0';
  while (strcmp(serial_msg, "end_gtz") != 0) // waiting for confirmation
    read_serial();

  mode_selected = false; // we leave the mode
  screen_update = true; // update the screen
  clear_data();
}

// luanching interpolated trajectory
void auto_interp_mode() { // same procedure as the auto mode
  Serial.println("#gtz%");
  serial_msg[0] = '\0';
  while (strcmp(serial_msg, "end_gtz") != 0)
    read_serial();

  Serial.println("#interp%");
  serial_msg[0] = '\0';
  while (strcmp(serial_msg, "interp_end") != 0)
    read_serial();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("auto interp end");
  delay(3000);

  Serial.println("#gtz%");
  serial_msg[0] = '\0';
  while (strcmp(serial_msg, "end_gtz") != 0)
    read_serial();

  mode_selected = false;
  screen_update = true;
  clear_data();
}

// manual control of the robot
void manual_mode(){
  clear_data();
  Serial.println("#gtz%");
  serial_msg[0] = '\0';
  while (strcmp(serial_msg, "end_gtz") != 0)
    read_serial();
  
  Serial.println("#manual%");
  
  manual_screen(); // update the screen

  while(!read_joystick_button()){ // while joystick not pressed we stay in this mode
    int x = 0; int y = 0;
    bool left = false; bool right = false;

    if(millis()-joystick_xy_last_time > 100){ // updating the inputs every 100 ms
      y = read_y_up() + 2*read_y_down(); // 1 if y up; 2 if y down to avoid a negative sign that would change the message length
      x = read_x_up() + 2*read_x_down();
      left = read_button_left();
      right = read_button_right();
      joystick_xy_last_time = millis();
    }

    int change = left+right+x+y; // if any changes occur, change will be non zero

    if(change){ // sending the inputs if there is a change
      Serial.print("#");
      Serial.print(x);Serial.print(";");
      Serial.print(y);Serial.print(";");
      Serial.print(left);Serial.print(";");
      Serial.print(right);Serial.println("%");

      get_step_current(); // getting steps and currents from mega

      manual_screen(); // update screen
    }
    
  }
  
  Serial.println("#end_manual%"); // sending the end of mode message

  Serial.println("#gtz%");
  serial_msg[0] = '\0';
  while (strcmp(serial_msg, "end_gtz") != 0)
    read_serial();
  
  mode_selected = false;
  screen_update = true;
  clear_data();
  
}



void select_mode() { // mode selection function 

  if(read_joystick_button()){ // joystick press means a mode has been selected
    mode_selected = true;
    screen_update = true;
  }

  else if(read_x_up()){
    mode = (mode+1)*(mode+1<=num_modes-1); // x up or down we change the mode to display
    screen_update = true;
  }

  else if(read_x_down()){
    mode = (mode-1)*(mode-1>=0) + (num_modes-1)*(mode-1<0);
    screen_update = true;
  }

}

void mode_select_screen() { // screen displayed when no mode is selected
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("    Select mode :   ");
  lcd.setCursor(0, 2);
  lcd.print(modes[mode]);
}

void mode_screen() { // updating the screen according to the selected mode
  switch (mode) {
    case 0:
      step_by_step_screen();
      break;
    
    case 1:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("mode auto");
      lcd.setCursor(0, 2);
      lcd.print("running...");
      break;

    case 2:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("mode auto interp");
      lcd.setCursor(0, 2);
      lcd.print("running...");
      break;

    case 3:
      manual_screen();
      break;

    default:
      break;
  }
}

// updating the screen if the screen_update variable is true and then set it to false
// this avoids the flickering screen
void update_screen() { 
                         
  if (screen_update) {

    if (!mode_selected)
      mode_select_screen();

    else
      mode_screen();
    
    screen_update = false;
  }
}