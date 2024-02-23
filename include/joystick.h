#ifndef JOYSTICK_H
#define JOYSTICK_H

#include "pinout.h"

struct JoystickReading {
    double x;
    double y;
};

void setupJoystick();
JoystickReading readJoystick();

#endif