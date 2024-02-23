#include <Arduino.h>
#include <ESP32Encoder.h>
#include "pinout.h"

#define PRINT_DELAY 100

ESP32Encoder encoder1;
ESP32Encoder encoder2;

void setup(){
	
	Serial.begin();

	// Enable the weak pull down resistors
	ESP32Encoder::useInternalWeakPullResistors=UP;

	encoder1.attachFullQuad(ENCODER1_A_PIN, ENCODER1_B_PIN);
	encoder2.attachFullQuad(ENCODER2_A_PIN, ENCODER2_B_PIN);
}

void loop(){
	// Loop and read the count
	Serial.println("Encoder 1 count = " + String((int32_t)encoder1.getCount()) + 
				   ",   Encoder 2 count = " + String((int32_t)encoder2.getCount()));
	delay(PRINT_DELAY);
}