#include "joystick.h"
#include <Arduino.h>

void setupJoystick() {
    pinMode(XPIN, INPUT); 
    pinMode(YPIN, INPUT); 
};

JoystickReading readJoystick() {
    JoystickReading output;
    
    output.x = analogRead(XPIN)/2048.0 - 1.0; 
    output.y = analogRead(YPIN)/2048.0 - 1.0; 

    return output;
};



