#include <Arduino.h>
#include "pinout.h"
#include "MotorDriver.h"

#define DELAY 200 // Delay between motor movements in milliseconds


// Create an instance of the MotorDriver class
MotorDriver motor1(DIR1, PWM1, 0);
MotorDriver motor2(DIR2, PWM2, 0);

void setup() {
    // Initialize serial communication
    Serial.begin();

    // Setup the motor driver
    motor1.setup();
    motor2.setup();
}

void loop() {
    // Move the motor forward at full speed
    Serial.println("Moving Forward at full speed");
    motor1.drive(0.3); // 30% duty cycle
    motor2.drive(0.3); // 30% duty cycle
    delay(DELAY);

    // Stop the motor
    Serial.println("Stopping");
    motor1.drive(0.0); // 0% duty cycle
    motor2.drive(0.0); // 0% duty cycle
    delay(DELAY);

    // Move the motor forward at half speed
    Serial.println("Moving Forward at half speed");
    motor1.drive(0.15); // 15% duty cycle
    motor2.drive(0.15); // 15% duty cycle
    delay(DELAY);

    // Stop the motor
    Serial.println("Stopping");
    motor1.drive(0.0); // 0% duty cycle
    motor2.drive(0.0); // 0% duty cycle
    delay(DELAY);

    // Move the motor backward at full speed
    Serial.println("Moving Backward at full speed");
    motor1.drive(-0.3); // -30% duty cycle (backward)
    motor2.drive(-0.3); // -30% duty cycle (backward)
    delay(DELAY);

    // Stop the motor
    Serial.println("Stopping");
    motor1.drive(0.0); // 0% duty cycle
    motor2.drive(0.0); // 0% duty cycle
    delay(DELAY);

    // Move the motor backward at half speed
    Serial.println("Moving Backward at half speed");
    motor1.drive(-0.15); // -15% duty cycle (backward)
    motor2.drive(-0.15); // -15% duty cycle (backward)
    delay(DELAY);

    // Stop the motor
    Serial.println("Stopping");
    motor1.drive(0.0); // 0% duty cycle
    motor2.drive(0.0); // 0% duty cycle
    delay(DELAY);
}
