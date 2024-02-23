#include <Arduino.h>
#include "PID.h"
#include "EncoderVelocity.h"
#include "pinout.h"
#include "util.h"
#include "MotorDriver.h"

// #define SerialMonitor
#define MatlabPlot

//PID Parameters
PID motorPID(10.0, 0.0, 0.0, 0.0, 0.1, false);

double setpoint = 0; 
double position = 0; //radians
double controlEffort = 0; //duty cycle
double desiredPosition = -M_PI/4.0;

int squarePeriod = 4000;  // square wave period in milli-seconds
int squareOn = 2000; 

MotorDriver motor(DIR1, PWM1, 0);
EncoderVelocity encoder(ENCODER1_A_PIN, ENCODER1_B_PIN, CPR_60_RPM, 0.2);

void setup() {
    motor.setup();
    Serial.begin(); 
}

void loop() {
    // Update PID at 1kHz
    EVERY_N_MICROS(1000) {
        if (millis() % squarePeriod < squareOn) {
            setpoint = desiredPosition;
        } else {
            setpoint = 0.0;
        }
        position = encoder.getPosition();
        controlEffort = motorPID.calculateParallel(position, setpoint);
        motor.drive(controlEffort);
    }

    // Print values at 50Hz
    EVERY_N_MILLIS(25) {
            // Print values to serial monitor
            #ifdef SerialMonitor
            // Print values to serial monitor
                Serial.printf("Setpoint (rad): %.2f, Position (rad): %.2f, Control Effort: %.2f",
                          setpoint, position, controlEffort);
            #endif

            #ifdef MatlabPlot
                Serial.printf("%.3f\t%.3f\t%.3f\n", millis()/1000.0, setpoint, position);
            #endif
        }
}
