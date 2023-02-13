#include "com.h"

long t0 = millis();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(button_left, INPUT_PULLUP);
  pinMode(button_right, INPUT_PULLUP);
  pinMode(button_gtz, INPUT_PULLUP);
  pinMode(button_sz, INPUT_PULLUP);
  
  lcd.begin(20,4); // Initializes the interface to the LCD screen, and specifies the dimensions (width and height) of the display }
  analogWrite(contrast,20); // Generate PWM signal at pin D11, value of 100 (out of 255)

  initialise_com();  // initialisation
}


void loop() {

  if(read_serial()){ // if we receive the init message we execute the protocole again
    if(strcmp(serial_msg, "init") == 0){
      initialise_com();
      screen_update = true;
      clear_data();
    }
  }

  if(read_button_gtz()){ // if the go to zero button is pressed 
    Serial.println("#gtz%"); // sending the command to the mega
    
    lcd.clear(); // displaying a message on screen
    lcd.setCursor(0, 0);
    lcd.print("going to position");
    lcd.setCursor(0, 1);
    lcd.print("zero");
    //delay(3000);

    serial_msg[0] = '\0';
    while (strcmp(serial_msg, "end_gtz") != 0) // waiting for the end confirmation
      read_serial();

    screen_update = true; // if true, thescreen will be updated prevents flickering
    clear_data();
  }

  else if(read_button_sz()){ // if set zero button pressed
    Serial.println("#sz%"); // sending command
    lcd.clear(); // displaying
    lcd.setCursor(0, 0);
    lcd.print("setting position");
    lcd.setCursor(0, 1);
    lcd.print("zero");
    delay(3000);
    screen_update = true;
    clear_data();
  }

  update_screen(); // updating the screen if necessary

  if(!mode_selected) // if the mode is not selected
    select_mode();
  
  else{ // else execute the selected mode

    switch(mode){
      case 0:
        step_by_step_mode();
        break;
      case 1:
        auto_mode();
        break;
      case 2:
        auto_interp_mode();
        break;
      case 3:
        manual_mode();
        break;
      default:
        mode_selected = false;
        break;
    }

  }

  
}

