#pragma once
#include "vars.h"

// functions that return true if a button is pressed or the joysticks are moved

bool read_button_sz() {
  int reading = digitalRead(button_sz);

  if(reading != button_sz_last_read && reading == 1 && millis()-button_sz_last_time > debounce_delay){ // rising edge detection + time interval check
    button_sz_last_time = millis();
    button_sz_last_read = reading;
    return true;
  }
  
  button_sz_last_read = reading;
  return false;
}

bool read_button_gtz() {
  int reading = digitalRead(button_gtz);

  if(reading != button_gtz_last_read && reading == 1 && millis()-button_gtz_last_time > debounce_delay){ // rising edge detection + time interval check
    button_gtz_last_time = millis();
    button_gtz_last_read = reading;
    return true;
  }
  
  button_gtz_last_read = reading;
  return false;
}

bool read_button_left() {
  int reading = digitalRead(button_left);

  if(reading != button_left_last_read && reading == 1 && millis()-button_left_last_time > debounce_delay){ // rising edge detection + time interval check
    button_left_last_time = millis();
    button_left_last_read = reading;
    return true;
  }
  
  button_left_last_read = reading;
  return false;
}

bool read_button_right() {
  int reading = digitalRead(button_right);

  if(reading != button_right_last_read && reading == 1 && millis()-button_right_last_time > debounce_delay){ // rising edge detection + time interval check
    button_right_last_time = millis();
    button_right_last_read = reading;
    return true;
  }
  
  button_right_last_read = reading;
  return false;
}

bool read_joystick_button() {
  int reading = digitalRead(joystick_button);

  if(reading != joystick_button_last_read && reading == 1 && millis()-joystick_last_time > debounce_delay){ // rising edge detection + time interval check
    joystick_last_time = millis();
    joystick_button_last_read = reading;
    return true;
  }
  
  joystick_button_last_read = reading;
  return false;
}

bool read_y_up() {
  int reading = analogRead(joystick_y);
  if (millis() - joystick_last_time > debounce_delay) {

    if (reading < ymin) {  // move up detection + time interval check
      joystick_last_time = millis();
      return true;
    }
  }
  return false;
}

bool read_y_down() {
  int reading = analogRead(joystick_y);

  if (millis() - joystick_last_time > debounce_delay) {

    if (reading > ymax) {  // move up detection + time interval check
      joystick_last_time = millis();
      return true;
    }
  }
  return false;
}

bool read_x_up() {
  int reading = analogRead(joystick_x);
  if (millis() - joystick_last_time > debounce_delay) {

    if (reading < xmin) {  // move up detection + time interval check
      joystick_last_time = millis();
      return true;
    }
  }
  return false;
}

bool read_x_down() {
  int reading = analogRead(joystick_x);

  if (millis() - joystick_last_time > debounce_delay) {

    if (reading > xmax) {  // move up detection + time interval check
      joystick_last_time = millis();
      return true;
    }
  }
  return false;
}