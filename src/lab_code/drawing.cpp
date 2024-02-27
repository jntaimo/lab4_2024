#include "drawing.h"


// #define SERIAL_MONITOR
#define MATLAB_PLOT

TrajectoryType trajectoryType = HORIZONTAL_LINE;

JoystickReading joystickReading;


//PID Parameters
double tau = 0.1; //seconds
PID motorPID1(2.0, 250.0, 0.0, 0.0, tau, false);
PID motorPID2(3.4, 350.0, 0.05, 0.0, tau, false);


JointSpace targetPose = {THETA1_OFFSET, 0.0};//initial setpoint
JointSpace pose = {THETA1_OFFSET, 0}; //current angles on the robot

TaskSpace initialXY = {0, L1 + L2}; //initial position of end effector
TaskSpace nominalXY = {0, (L1 + L2)*0.6}; //nominal position of end effector
TaskSpace targetXY = nominalXY; //where we want the end effector to go
TaskSpace actualXY; //where the end effector actually is

double controlEffort1 = 0; //duty cycle
double controlEffort2 = 0; //duty cycle

//set up motor drivers
MotorDriver motor1(DIR1, PWM1, 0);
MotorDriver motor2(DIR2, PWM2, 1);
//set up encoders
EncoderVelocity encoder1(ENCODER1_A_PIN, ENCODER1_B_PIN, CPR_60_RPM, 0.2);
EncoderVelocity encoder2(ENCODER2_A_PIN, ENCODER2_B_PIN, CPR_60_RPM, 0.2);


void setup() {
    motor1.setup();
    motor2.setup();
    Serial.begin(); 
    setupJoystick();

}

void loop() {
    // Update setpoint and kinematics at 200hz
    EVERY_N_MILLIS(5) {
        //get new cartesian setpoint based on trajectory type
        targetXY = updateSetpoint(nominalXY, trajectoryType, millis());
        //update current XY based on current pose
        actualXY = forwardKinematics(pose);
        //Get new joint setpoint based on cartesian setpoint
        JointSpace newTargetPose = inverseKinematics(targetXY);
        //limit pose to be within safe limits
        targetPose = poseLimit(newTargetPose);
    }

    // Update PID at 2kHz
    EVERY_N_MICROS(500) {  
        //get the current position of the end effector based on the encoders
        pose.theta1 = encoder1.getPosition() + THETA1_OFFSET;
        pose.theta2 = -encoder2.getPosition();
        
        //calculate the control effort using PID
        controlEffort1 = motorPID1.calculateParallel(pose.theta1, targetPose.theta1);
        controlEffort2 = motorPID2.calculateParallel(pose.theta2, targetPose.theta2);

        //drive the motors
        motor1.drive(controlEffort1);
        motor2.drive(controlEffort2);
    }

    // Print values at 50Hz
    EVERY_N_MILLIS(20) {
            #ifdef SERIAL_MONITOR
            // Print values to serial monitor
                Serial.printf("Target X: %.3f, Y: %.3f   "
                          "Actual X: %.3f, Y: %.3f\n",
                          endEffectorTarget.x, endEffectorTarget.y,
                          endEffectorActual.x, endEffectorActual.y);
            #endif

            #ifdef MATLAB_PLOT
                Serial.printf("%.2f\t%.2f\t%.2f\t%.2f\n", 
                        targetXY.x, targetXY.y, 
                        actualXY.x, actualXY.y);
            #endif
    }
}

JointSpace poseLimit(JointSpace pose){
    pose.theta1 = constrain(pose.theta1, -THETA1_MAX + THETA1_OFFSET, THETA1_MAX + THETA1_OFFSET);
    pose.theta2 = constrain(pose.theta2, -THETA2_MAX, THETA2_MAX);
    pose.theta2 = constrain(pose.theta2, -THETA2_MAX/2-pose.theta1 + THETA1_OFFSET, THETA2_MAX/2+pose.theta1 + THETA1_OFFSET);
    return pose;
}