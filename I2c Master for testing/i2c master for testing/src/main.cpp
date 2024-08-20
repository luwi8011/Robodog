#include <Arduino.h>
#include <Wire.h>

#define I2C_SLAVE_ADDRESS 0x08  // Replace with the actual address of your I2C slave device
#define MESSAGE_INTERVAL 1500   // Time interval in milliseconds

unsigned long previousMillis = 0;  // Variable to store the last time the message was sent

void setup() {
  Serial.begin(9600); // Initialize serial communication at 9600 baud rate
  Wire.begin();       // Join the I2C bus as a master
  Serial.println("I2C Master ready.");
}

void sendMessageOverI2C(String message) {
  // Start communication with the I2C slave
  Wire.beginTransmission(I2C_SLAVE_ADDRESS);

  // Convert the String message to a C-string and send it over I2C
  Wire.write(message.c_str());

  // End the I2C transmission
  Wire.endTransmission();

  // Print the message to the serial monitor for confirmation
  Serial.print("Sent over I2C: ");
  Serial.println(message);
}

void loop() {
  unsigned long currentMillis = millis();  // Get the current time

  // Check if 1500 milliseconds have passed
  if (currentMillis - previousMillis >= MESSAGE_INTERVAL) {
    previousMillis = currentMillis;  // Save the current time

    // Send the "HRT" message over I2C
    sendMessageOverI2C("HRT");
  }

  // If there's serial input available, process it
  if (Serial.available() > 0) {
    String message = Serial.readStringUntil('\n'); // Read the incoming serial data until newline character
    sendMessageOverI2C(message);
  }
}

