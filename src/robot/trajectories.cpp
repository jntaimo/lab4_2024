#include "trajectories.h"


TaskSpace horizontalLine(TaskSpace nominalPosition , double frequency, double amplitude, unsigned long time){
    nominalPosition.x = nominalPosition.x + amplitude*sin(2*M_PI*frequency*time/1000.0);
    return nominalPosition;
}

TaskSpace verticalLine(TaskSpace nominalPosition , double frequency, double amplitude, unsigned long time){
    nominalPosition.y = nominalPosition.y + amplitude*sin(2*M_PI*frequency*time/1000.0);
    return nominalPosition;
}

TaskSpace circle(TaskSpace nominalPosition , double frequency, double radius, unsigned long time){
    nominalPosition.x = nominalPosition.x + radius*cos(2*M_PI*frequency*time/1000.0);
    nominalPosition.y = nominalPosition.y + radius*sin(2*M_PI*frequency*time/1000.0);
    return nominalPosition;
}

TaskSpace spiral(TaskSpace nominalPosition , double frequency, double radius, uint revolutions, unsigned long time){
    nominalPosition.x = nominalPosition.x + radius*cos(2*M_PI*frequency*time/1000.0)*sin(2*M_PI*frequency*time/1000.0);
    nominalPosition.y = nominalPosition.y + radius*sin(2*M_PI*frequency*time/1000.0)*sin(2*M_PI*frequency*time/1000.0);
    return nominalPosition;
}

TaskSpace joystickControl(TaskSpace nominalPosition , JoystickReading joystickReading, unsigned long time){
    nominalPosition.x = nominalPosition.x + 5*joystickReading.x;
    nominalPosition.y = nominalPosition.y + 5*joystickReading.y;
    return nominalPosition;
}

TaskSpace updateSetpoint(TaskSpace nominalPosition , TrajectoryType trajectoryType, unsigned long time){
    switch (trajectoryType)
    {
    case HORIZONTAL_LINE:
        return horizontalLine(nominalPosition, 0.5, 20, time);
        break;
    case VERTICAL_LINE:
        return verticalLine(nominalPosition, 0.5, 10, time);
        break;
    case CIRCLE:
        return circle(nominalPosition, 0.5, 5, time);
        break;
    case SPIRAL:
        return spiral(nominalPosition, 0.5, 5, 5, time);
        break;
    case JOYSTICK:
        return joystickControl(nominalPosition, joystickReading, time);
        break;
    default:
        return nominalPosition;
        break;
    }
}