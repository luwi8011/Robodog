#include <Arduino.h>
#include <Wire.h>
#include <PID_v1.h>
#include <EEPROM.h>
#include <avr/wdt.h>

// Variables to store the minimum and maximum potentiometer values (as voltate not angle)
double minSmoothedValue = 1023; // Start with the maximum possible value for min
double maxSmoothedValue = 0;    // Start with the minimum possible value for max

// EEPROM Addresses for storing min and max smoothed values
const int addrMinSmoothedValue = sizeof(float) * 4; // Offset addresses for min and max values
const int addrMaxSmoothedValue = sizeof(float) * 5;

unsigned long previousMillis = 0; // stores the last time smoothe() was called
const long interval = 10;         // interval of 10 milliseconds
int callCount = 0;                // counter for smoothe() calls
bool smootheComplete = false;     // flag to indicate if smoothe() has been called 10 times





// Define PID variables
double currentAngle; // Current angle from the potentiometer
double targetAngle;  // Desired target angle
double motorOutput;  // Output value for the motor controller (0-255 for PWM)

// Define PID tuning parameters
double Kp = 1.75, Ki = 0.2, Kd = 0.75;

// Create PID controller object
PID myPID(&currentAngle, &motorOutput, &targetAngle, Kp, Ki, Kd, DIRECT);

// If the PWM value sent to the motors is less than +- this value then hold the motors locked
const int addrDeadZone = sizeof(float) * 6; // EEPROM address for deadZone
int defaultDeadZone = 5; // Default deadZone value
int deadZone = 0;

// Flag to indicate new data received
volatile bool newDataReceived = false;

// moving average smoothing factor and smoothed POT value
double alpha = 0.1;       // Smoothing factor (0 < alpha <= 1)
double smoothedValue = 0; // Initial smoothed value

// checks if EEPROM has data in it, if it does not have data then load generic PID values. If it does have data then load that data up

// EEPROM Addresses for storing PID values
const int addrKp = 0;
const int addrKi = sizeof(float);
const int addrKd = sizeof(float) * 2;
const int addrMarker = sizeof(float) * 3; // Marker to check if EEPROM is initialized
const byte markerValue = 0x55;            // Arbitrary value to mark EEPROM initialization

const byte SLAVE_ADDRESS = 0x08;
void receiveEvent(int numBytes);


// used for safemode timing
unsigned long lastMessageTime = 0;  // Tracks the time when the last message was received
bool safeModeActive = false;  // Flag to indicate if safe mode is active


#define LED_R_PIN 10
#define LED_G_PIN 11
#define LED_B_PIN 3

#define MotEnable 9 // Motor Enamble pin Runs on PWM signal
#define MotFwd 5    // Motor Forward pin
#define MotRev 6    // Motor Reverse pin

int rawValue = A6;        // Read potentiometer value (0 to 1023)
const int maxAngle = 180; // Maximum angle (e.g., 0 to 180 degrees)



/*
A0 reads the multisense pin
Digital pin 7 is an output for the select pin on the H bridge, this selects what information is sent to A0
*/

void setupLEDs() {
    pinMode(LED_R_PIN, OUTPUT);
    pinMode(LED_G_PIN, OUTPUT);
    pinMode(LED_B_PIN, OUTPUT);
}

void setLEDColor(int red, int green, int blue) {
    analogWrite(LED_R_PIN, red);
    analogWrite(LED_G_PIN, green);
    analogWrite(LED_B_PIN, blue);
}



void initializeEEPROM()
{
  // Check if EEPROM is initialized
  byte storedMarker;
  EEPROM.get(addrMarker, storedMarker);

  if (storedMarker != markerValue)
  {
    // EEPROM is not initialized, write default values
    float defaultKp = 1.75;
    float defaultKi = .2;
    float defaultKd = .75;

    EEPROM.put(addrKp, defaultKp);
    EEPROM.put(addrKi, defaultKi);
    EEPROM.put(addrKd, defaultKd);

    // Store the default deadZone value in EEPROM
    EEPROM.put(addrDeadZone, defaultDeadZone);

    // Write marker value to indicate initialization
    EEPROM.put(addrMarker, markerValue);

    Serial.println("EEPROM initialized with default PID values.");
  }
  else
  {
    Serial.println("EEPROM already initialized.");
  }
}

void loadPIDFromEEPROM()
{
  EEPROM.get(addrKp, Kp);
  EEPROM.get(addrKi, Ki);
  EEPROM.get(addrKd, Kd);
  myPID.SetTunings(Kp, Ki, Kd);
}

void loadCalibrationValuesFromEEPROM()
{
  EEPROM.get(addrMinSmoothedValue, minSmoothedValue);
  EEPROM.get(addrMaxSmoothedValue, maxSmoothedValue);
  Serial.print("Loaded Min Smoothed Value: ");
  Serial.println(minSmoothedValue);
  Serial.print("Loaded Max Smoothed Value: ");
  Serial.println(maxSmoothedValue);
}

void loadDeadZoneFromEEPROM()
{
    int storedDeadZone;
    EEPROM.get(addrDeadZone, storedDeadZone);

    // Validate the loaded value to ensure it's within a reasonable range
    if (storedDeadZone >= 0 && storedDeadZone <= 255)
    {
        deadZone = storedDeadZone;
        Serial.print("Loaded deadZone from EEPROM: ");
        Serial.println(deadZone);
    }
    else
    {
        // If the loaded value is invalid, use the default deadZone
        deadZone = 5; // Default deadZone value
        Serial.println("Invalid deadZone in EEPROM. Using default value: 5");
    }
}

void setup()
{
  setupLEDs(); //setup the LED
  setLEDColor(0, 0, 255); // Blue on to show initialization

  Wire.begin(SLAVE_ADDRESS);    // Initialize I2C communication as a slave
  Wire.onReceive(receiveEvent); // Register the receive event function
  Serial.begin(9600);           // Initialize serial communication for debugging

  pinMode(MotEnable, OUTPUT);
  digitalWrite(MotEnable, HIGH); // set the enable pin high always
  pinMode(MotFwd, OUTPUT);
  pinMode(MotRev, OUTPUT);

  //pinMode(rawValue, INPUT);  not needed because analog pins are inputs by default

  myPID.SetMode(AUTOMATIC);         // set PID in Auto mode
  myPID.SetSampleTime(100);         // refresh rate of PID controller
  myPID.SetOutputLimits(-255, 255); // this is the MAX PWM value to move motor,

  initializeEEPROM();                // Initialize EEPROM with default values if not already initialized
  loadPIDFromEEPROM();               // Load stored PID values from EEPROM
  loadCalibrationValuesFromEEPROM(); // Load stored calibration values from EEPROM
  loadDeadZoneFromEEPROM();          // Load stored deadzone values from EEPROM

  
  setLEDColor(0, 255, 0);// set LED green to show that setup has completed

  Serial.print("Loaded deadZone: ");
  Serial.println(deadZone);
}

// Function to be executed if the message is between 0 and 255 (Target angle for PID)
void target()
{
  Serial.println("Executing PID: target angle received");

  currentAngle = map(smoothedValue, 0, 1023, 0, 180); // Convert to angle (assuming 0-180 degrees)

  // Call the function to calculate PWM output based on current angle and target angle
  myPID.Compute();
  int pwmValue = (int)motorOutput;

  if (pwmValue > deadZone)
  {
    // Forward direction
    digitalWrite(MotFwd, HIGH);       // Set forward direction
    digitalWrite(MotRev, LOW);        // Ensure reverse is off
    analogWrite(MotEnable, pwmValue); // Set PWM for speed
  }
  else if (pwmValue < -deadZone)
  {
    // Reverse direction
    digitalWrite(MotFwd, LOW);             // Ensure forward is off
    digitalWrite(MotRev, HIGH);            // Set reverse direction
    analogWrite(MotEnable, abs(pwmValue)); // Set PWM for speed
  }
  else
  {
    // lock the motors within this deadzone
    digitalWrite(MotFwd, HIGH); // Ensure forward is off
    digitalWrite(MotRev, HIGH); // Set reverse direction
    analogWrite(MotEnable, 0);  // Disable the PWM signal (or leave it HIGH depending on your controller)
  }
}



// Function to be executed if the message is "SOS"
void safeMode()
{
    // Disable the motors
    analogWrite(MotEnable, 0);
    digitalWrite(MotFwd, LOW);
    digitalWrite(MotRev, LOW);

    // Set LED to continuous red
    setLEDColor(255, 0, 0);

    // Set the safe mode flag
    safeModeActive = true;

    Serial.println("Safe mode activated: Motors disabled.");
}

void loop()
{


  if (safeModeActive) {
        // If safe mode is active, do nothing
        setLEDColor(255, 0, 0); // set LED red to show safe mode is active
        return;
    }

    // Check if 2000 milliseconds have passed since the last message was received
    if (millis() - lastMessageTime >= 2000) {
        safeMode();  // Activate safe mode if no message was received within the timeout period
    }

  if (newDataReceived)
  {                                                                                 // if the flag is set true (new target) target the new angle
    smoothedValue = (alpha * analogRead(rawValue)) + ((1 - alpha) * smoothedValue); // refresh smoothed angle input
    target();                                                                       // Process the new target angle
    newDataReceived = false;                                                        // Reset the flag
  }

  // Call smoothe() 10 times with a 10 ms delay between each call
  if (!smootheComplete)
  {
    unsigned long currentMillis = millis(); // get the current time

    // Check if 10 milliseconds have passed and if smoothe() has been called less than 10 times
    if (currentMillis - previousMillis >= interval)
    {
      smoothedValue = (alpha * analogRead(rawValue)) + ((1 - alpha) * smoothedValue); // Apply EMA
      callCount++;                                                                    // increment the call counter

      // Check if smoothe() has been called 10 times
      if (callCount >= 10)
      {
        smootheComplete = true; // mark smoothe task as complete
        callCount = 0;          // reset the call counter for the next loop
      }
      previousMillis = currentMillis; // update the last time smoothe() was called
    }
  }
  else
  {
    // Perform the next task after smoothe() has been called 10 times

    // calls the target function which uses the last received target angle and the current potentiometer angle PID
    target();
    setLEDColor(0, 255, 0);// set LED color green to show normal operation

    // Once the next task is complete, reset the smootheComplete flag
    smootheComplete = false;
  }
}

// Calibration speed for sweeping the leg
//const int calibrationSpeed = 100;        // Adjust this value to control the sweep speed. Now gets passed through by the I2c CAL command
double lastSmoothedValue = 0;            // The last recorded smoothedValue
const unsigned long stallTimeout = 1000; // 1 second timeout to detect stall
const double changeThreshold = 1.0;      // Minimum change in smoothedValue to reset the timer

bool isMotorStalled()
{
  unsigned long currentMillis = millis(); // Get the current time
  static unsigned long lastChangeTime = 0;        // Time when the smoothedValue last changed

  // Check if the smoothedValue has changed significantly
  if (abs(smoothedValue - lastSmoothedValue) > changeThreshold)
  {
    // If it has changed, update the lastChangeTime and lastSmoothedValue
    lastChangeTime = currentMillis;
    lastSmoothedValue = smoothedValue;
  }

  // If no significant change has occurred for 1 second, return true (indicating a stall)
  if (currentMillis - lastChangeTime >= stallTimeout)
  {
    // Blink red fast to indicate motor stall
    for (int i = 0; i < 5; i++) {
        setLEDColor(255, 0, 0); // Red on
        delay(100);
        setLEDColor(0, 0, 0); // LED off
        delay(100);
    }
    return true;
  }

  return false;
}

// Function to sweep the leg from one extreme to the other while recording the minimum and maximum smoothed values
void sweepLegAndRecordExtremes(int calibrationSpeed)
{
  // Move the leg in one direction (e.g., open the leg)
  while (true)
  {
    analogWrite(MotEnable, calibrationSpeed);
    digitalWrite(MotFwd, HIGH); // Forward direction
    digitalWrite(MotRev, LOW);

    // Continuously update smoothedValue
    smoothedValue = (alpha * analogRead(rawValue)) + ((1 - alpha) * smoothedValue); // Apply EMA

    // Update the minimum and maximum smoothed values
    if (smoothedValue < minSmoothedValue)
    {
      minSmoothedValue = smoothedValue;
    }
    if (smoothedValue > maxSmoothedValue)
    {
      maxSmoothedValue = smoothedValue;
    }

    // Check if the leg has stopped moving in the forward direction (e.g., motor stall detection)
    if (isMotorStalled())
    {
      break;
    }

    delay(10); // Delay for smooth movement
  }

  // Brief pause before reversing direction
  delay(500);

  // Move the leg in the opposite direction (e.g., close the leg)
  while (true)
  {
    analogWrite(MotEnable, calibrationSpeed);
    digitalWrite(MotFwd, LOW); // Reverse direction
    digitalWrite(MotRev, HIGH);

    // Continuously update smoothedValue
    smoothedValue = (alpha * analogRead(rawValue)) + ((1 - alpha) * smoothedValue); // Apply EMA

    // Update the minimum and maximum smoothed values
    if (smoothedValue < minSmoothedValue)
    {
      minSmoothedValue = smoothedValue;
    }
    if (smoothedValue > maxSmoothedValue)
    {
      maxSmoothedValue = smoothedValue;
    }

    // Check if the leg has stopped moving in the reverse direction (e.g., motor stall detection)
    if (isMotorStalled())
    {
      break;
    }

    delay(10); // Delay for smooth movement
  }

  // Stop the motor after completing the sweep
  analogWrite(MotEnable, 0);
}

// Function to be executed if the message is "CAL"
void calibrateJoint(int calibrationSpeed)
{
  Serial.println("Starting calibration...");

  // Move the leg from fully open to fully closed position while recording the extremes
  sweepLegAndRecordExtremes(calibrationSpeed);

  // Store the min and max smoothed values to EEPROM
  EEPROM.put(addrMinSmoothedValue, minSmoothedValue);
  EEPROM.put(addrMaxSmoothedValue, maxSmoothedValue);
  Serial.println("Min and Max smoothed values stored to EEPROM.");

  // Print the calibration results
  Serial.print("Fully open position recorded: ");
  Serial.println(minSmoothedValue);
  Serial.print("Fully closed position recorded: ");
  Serial.println(maxSmoothedValue);

  Serial.println("Calibration complete. Use these values in the map function.");
}



void storePIDToEEPROM(float Kp, float Ki, float Kd)
{
  EEPROM.put(addrKp, Kp);
  EEPROM.put(addrKi, Ki);
  EEPROM.put(addrKd, Kd);
}



void reboot() {
    wdt_enable(WDTO_15MS);  // Enable watchdog timer to reset the system in 15ms
    while (true) {}         // Wait for the watchdog timer to reset the system
}




// This function is called whenever data is received from the master.
void receiveEvent(int numBytes)
{
    // Buffer to hold the received message
    char receivedMessage[8];
    int index = 0;

    // Read the message into the buffer
    while (Wire.available() && index < numBytes)
    {
        receivedMessage[index] = Wire.read();
        index++;
    }
    receivedMessage[index] = '\0'; // Null-terminate the string


  

    // Check if the message is "RES" (reboot command)
    if (strcmp(receivedMessage, "RES") == 0)
    {
        reboot(); // Call reboot() if the message is "RES"
    }

    // If safe mode is active, ignore all other commands except reboot
    if (safeModeActive) {
        return;
    }

    // Reset the last message time whenever a valid message is received
    lastMessageTime = millis();

     // Check if the received message is a single byte that represents a target angle
    if (numBytes == 1)
    {
        int receivedNumber = receivedMessage[0]; // Read the first byte as an integer
        if (receivedNumber >= 0 && receivedNumber <= 255)
        {
            targetAngle = receivedNumber; // Update the targetAngle
            newDataReceived = true; // Set the flag indicating new data received
            Serial.print("New target angle: ");
            Serial.println(targetAngle);
        }
    }
    // Handle different commands based on the message content
    else if (strncmp(receivedMessage, "CAL", 3) == 0)
    {
        int calValue = atoi(receivedMessage + 3); // Convert the number after "CAL" to an integer
        calibrateJoint(calValue); // Call calibrate function with the extracted number
    }
    else if (strcmp(receivedMessage, "SOS") == 0)
    {
        safeMode(); // Call safeMode if the message is "SOS"
    }
    else if (strncmp(receivedMessage, "DED", 3) == 0)
    {
        // Extract the deadZone value following "DED"
        int newDeadZone = atoi(&receivedMessage[3]);

        // Validate and update deadZone
        if (newDeadZone >= 0 && newDeadZone <= 255)
        {
            deadZone = newDeadZone;
            EEPROM.put(addrDeadZone, deadZone); // Save the new deadZone value to EEPROM
            Serial.print("New deadZone saved: ");
            Serial.println(deadZone);
        }
    }
    // Handle a message containing PID tuning parameters
    else if (numBytes == 7 && receivedMessage[0] == 'P')
    {
        byte KpBytes[2], KiBytes[2], KdBytes[2];
        KpBytes[0] = Wire.read();
        KpBytes[1] = Wire.read();
        KiBytes[0] = Wire.read();
        KiBytes[1] = Wire.read();
        KdBytes[0] = Wire.read();
        KdBytes[1] = Wire.read();

        // Combine the two bytes into a float or fixed-point representation
        Kp = (float)((KpBytes[0] << 8) | KpBytes[1]) / 100.0;
        Ki = (float)((KiBytes[0] << 8) | KiBytes[1]) / 100.0;
        Kd = (float)((KdBytes[0] << 8) | KdBytes[1]) / 100.0;

        // Update the PID controller with the new parameters
        myPID.SetTunings(Kp, Ki, Kd);

        // Store the new PID values in EEPROM
        storePIDToEEPROM(Kp, Ki, Kd);

        Serial.print("New Kp: ");
        Serial.println(Kp);
        Serial.print("New Ki: ");
        Serial.println(Ki);
        Serial.print("New Kd: ");
        Serial.println(Kd);
    }
}
