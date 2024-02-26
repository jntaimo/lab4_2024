#include <Arduino.h>
#include "PID.h"
#include "SimpleFilters.h"
#include "EncoderVelocity.h"
#include "pinout.h"
#include "util.h"
#include "MotorDriver.h"

//#define SerialMonitor
#define MatlabPlot

#define Ti 0.0183
#define Td 0.0021
#define Kp 9.36
#define leadAlpha 10

//PID Parameters
LeadFilter lead = LeadFilter(leadAlpha, Td);
LagFilter lag = LagFilter(Ti);

double setpoint = 0; 
double position = 0; //radians
double controlEffort = 0; //duty cycle
double desiredPosition = -M_PI/4.0;

int squarePeriod = 4000;  // square wave period in milli-seconds
int squareOn = 2000; 

MotorDriver motor(DIR1, PWM1, 0);
EncoderVelocity encoder(ENCODER1_A_PIN, ENCODER1_B_PIN, CPR_60_RPM, 0.2);

double calculateControlEffort(double position, double setpoint);

void setup() {
    motor.setup();
    Serial.begin(); 
}

void loop() {
    // Update PID at 2kHz
    EVERY_N_MICROS(500) {
        // if (millis() % squarePeriod < squareOn) {
        //     setpoint = desiredPosition;
        // } else {
        //     setpoint = 0.0;
        // }
        setpoint = M_PI/20*sin(20*millis()/1000.0);
        position = encoder.getPosition();
        controlEffort = calculateControlEffort(position, setpoint);
        motor.drive(controlEffort);
    }

    // Print values at 50Hz
    EVERY_N_MILLIS(25) {
            // Print values to serial monitor
            #ifdef SerialMonitor
            // Print values to serial monitor
                Serial.printf("Setpoint (rad): %.2f, Position (rad): %.2f, Control Effort: %.2f\n",
                          setpoint, position, controlEffort);
            #endif

            #ifdef MatlabPlot
                Serial.printf("%.3f\t%.3f\t%.3f\n", millis()/1000.0, setpoint, position);
            #endif
        }
}

// Calculate the control effort using a lead lag contoller
double calculateControlEffort(double position, double setpoint) {
    double error = Kp*(setpoint - position);
    double lagOutput = lag.calculate(error);
    return lead.calculate(error);
    
    // return lead.calculate(lagOutput);

}