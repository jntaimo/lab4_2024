#include <Arduino.h>
#include <math.h>
#include "PID.h"
#include "EncoderVelocity.h"
#include "util.h"
#include "MotorDriver.h"
#include "kinematics.h"
#include "joystick.h"

// #define SerialMonitor
#define MatlabPlot

enum TrajectoryType {
    HORIZONTAL_LINE,
    VERTICAL_LINE,
    CIRCLE,
    JOYSTICK
};

TrajectoryType trajectoryType = HORIZONTAL_LINE;

unsigned long startTime;
unsigned long elapsedTime;
unsigned long setupTime = 2500;
unsigned long currentMillis;
double t;

//PID Parameters
double tau = 0.1; //seconds
PID motorPID1(2.0, 250.0, 0.0, 0.0, tau, false);
PID motorPID2(3.4, 350.0, 0.05, 0.0, tau, false);

JoystickReading joystickReading;

JointSpace state;
JointSpace setpoint = {THETA1_OFFSET, 0.0};
JointSpace newSetpoint;
JointSpace endEffectorState;

TaskSpace endEffectorInitial = {0, 25};
TaskSpace endEffectorTarget;
TaskSpace endEffectorActual;

double theta1 = 0; //radians
double theta2 = 0; //radians
double controlEffort1 = 0; //duty cycle
double controlEffort2 = 0; //duty cycle

MotorDriver motor1(DIR1, PWM1, 0);
MotorDriver motor2(DIR2, PWM2, 0);
EncoderVelocity encoder1(ENCODER1_A_PIN, ENCODER1_B_PIN, CPR_60_RPM, 0.2);
EncoderVelocity encoder2(ENCODER2_A_PIN, ENCODER2_B_PIN, CPR_60_RPM, 0.2);

// Checks if provided JointSpace state is within safety limits
// Returns true if it is and false otherwise
bool safetyLimit(JointSpace state) {
    return abs(state.theta1 - M_PI/2.0) <= M_PI/2.0 &&
           abs(state.theta2) <= M_PI*0.9;
};

void setup() {
    motor1.setup();
    motor2.setup();

    Serial.begin(); 
    setupJoystick();

    startTime = millis();
}

void loop() {
    // Update setpoint at 2kHz
    EVERY_N_MICROS(500) {
        currentMillis = millis();
        elapsedTime = currentMillis - startTime;
        // Takes setupTime milliseconds to go to initial position 
        if (elapsedTime < setupTime) {
            endEffectorTarget = endEffectorInitial;
        } else {
            t = currentMillis/1000.0;
            switch (trajectoryType)
            {
            case HORIZONTAL_LINE:
                endEffectorTarget.x = endEffectorInitial.x + 10*sin(M_PI/4.0*t);
                break;
            case VERTICAL_LINE:
                endEffectorTarget.y = endEffectorInitial.y + 10*sin(M_PI/4.0*t);
                break;
            case CIRCLE:
                endEffectorTarget.x = endEffectorInitial.x + 5*cos(t);
                endEffectorTarget.y = endEffectorInitial.y + 5*sin(t);
                break;
            case JOYSTICK:
                joystickReading = readJoystick(); 
                // TODO 3: Convert joystickReading to a reasonable target end effector position
                // Make sure your endEffectorTarget coordinates are bounded within a reasonable range
                // x in [-25, 25], y in [20, 35]
                // endEffectorTarget.x = 
                // endEffectorTarget.y = 
                break;
            }

        }

        // TODO 1: Set newSetpoint using inverseKinematics() 
        //newSetpoint = 
        
        // If new setpoint is within safety limits, use new setpoint
        // Otherwise, keeps old setpoint so robot does nothing
        if (safetyLimit(newSetpoint)) {
            setpoint = newSetpoint;
        }

        endEffectorState = {theta1, theta2};
        endEffectorActual = forwardKinematics(endEffectorState);
    }

    // Update PID at 10kHz
    EVERY_N_MICROS(100) {        
        theta1 = encoder1.getPosition() + THETA1_OFFSET;
        theta2 = -encoder2.getPosition();
        controlEffort1 = motorPID1.calculateParallel(theta1, setpoint.theta1);
        controlEffort2 = motorPID2.calculateParallel(theta2, setpoint.theta2);

        motor1.drive(controlEffort1);
        motor2.drive(controlEffort2);
    }

    // Print values at 50Hz
    EVERY_N_MILLIS(25) {
            #ifdef SerialMonitor
            // Print values to serial monitor
                Serial.printf("Target X: %.3f, Y: %.3f   "
                          "Actual X: %.3f, Y: %.3f\n",
                          endEffectorTarget.x, endEffectorTarget.y,
                          endEffectorActual.x, endEffectorActual.y);
            #endif

            #ifdef MatlabPlot
                Serial.printf("%.2f\t%.2f\t%.2f\t%.2f\n", 
                        endEffectorTarget.x, endEffectorTarget.y, 
                        endEffectorActual.x, endEffectorActual.y);
            #endif
    }
}
