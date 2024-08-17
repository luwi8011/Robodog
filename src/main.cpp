#include <Arduino.h>
#include <Wire.h>
#include <PID_v1.h>
#include <EEPROM.h>

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

int rawValue = A0;        // Read potentiometer value (0 to 1023)
const int maxAngle = 180; // Maximum angle (e.g., 0 to 180 degrees)

#define MotEnable 6 // Motor Enamble pin Runs on PWM signal
#define MotFwd 9    // Motor Forward pin
#define MotRev 8    // Motor Reverse pin

// Define PID variables
double currentAngle; // Current angle from the potentiometer
double targetAngle;  // Desired target angle
double motorOutput;  // Output value for the motor controller (0-255 for PWM)

// Define PID tuning parameters
double Kp = 2.0, Ki = 5.0, Kd = 1.0;

// Create PID controller object
PID myPID(&currentAngle, &motorOutput, &targetAngle, Kp, Ki, Kd, DIRECT);

// If the PWM value sent to the motors is less than +- this value then hold the motors locked
int motorLockOffset = 5;

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

/*
Features checklist:
*Receive various message lengths from I2c master
*if message is int from 0-255 then use this number as the target angle in the PID loop
    *exponential moving average on the POT
    *Use PID library and make it capable of forward and reverse
    *Integrate LED color
*If message is three letter word check the following
    *If "CAL" then run calibrate function which sweeps full travel of joint and records min and max angles to EEPROM
      *Integrate LED color
    *If "SOS" then run safe mode where motors are set to 0 pwm
      *Integrate LED color



ToDo:
  make the calibration speed defined over I2c so that we can start with a slow PWM
  make an I2c message that defines the deadzone size and store it to EEPROM
  finish the SOS safemode. if we havnt heard a message is 2 seconds failsafe to motors off
  come up with more uses for the LED
  add a reset command over I2c to reset the arduino (might not be needed)

*/

void initializeEEPROM()
{
  // Check if EEPROM is initialized
  byte storedMarker;
  EEPROM.get(addrMarker, storedMarker);

  if (storedMarker != markerValue)
  {
    // EEPROM is not initialized, write default values
    float defaultKp = 2.0;
    float defaultKi = 5.0;
    float defaultKd = 1.0;

    EEPROM.put(addrKp, defaultKp);
    EEPROM.put(addrKi, defaultKi);
    EEPROM.put(addrKd, defaultKd);

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

void setup()
{
  Wire.begin(SLAVE_ADDRESS);    // Initialize I2C communication as a slave
  Wire.onReceive(receiveEvent); // Register the receive event function
  Serial.begin(9600);           // Initialize serial communication for debugging

  pinMode(MotEnable, OUTPUT);
  digitalWrite(MotEnable, HIGH); // set the enable pin high always
  pinMode(MotFwd, OUTPUT);
  pinMode(MotRev, OUTPUT);

  pinMode(rawValue, INPUT);

  myPID.SetMode(AUTOMATIC);         // set PID in Auto mode
  myPID.SetSampleTime(100);         // refresh rate of PID controller
  myPID.SetOutputLimits(-255, 255); // this is the MAX PWM value to move motor,

  initializeEEPROM();                // Initialize EEPROM with default values if not already initialized
  loadPIDFromEEPROM();               // Load stored PID values from EEPROM
  loadCalibrationValuesFromEEPROM(); // Load stored calibration values from EEPROM
}

// Function to be executed if the message is between 0 and 255 (Target angle for PID)
void target()
{
  Serial.println("Executing PID: target angle received");

  currentAngle = map(smoothedValue, 0, 1023, 0, 180); // Convert to angle (assuming 0-180 degrees)

  // Call the function to calculate PWM output based on current angle and target angle
  myPID.Compute();
  int pwmValue = (int)motorOutput;

  if (pwmValue > motorLockOffset)
  {
    // Forward direction
    digitalWrite(MotFwd, HIGH);       // Set forward direction
    digitalWrite(MotRev, LOW);        // Ensure reverse is off
    analogWrite(MotEnable, pwmValue); // Set PWM for speed
  }
  else if (pwmValue < -motorLockOffset)
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

void loop()
{

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

    // Once the next task is complete, reset the smootheComplete flag
    smootheComplete = false;
  }
}

// Calibration speed for sweeping the leg
const int calibrationSpeed = 100;        // Adjust this value to control the sweep speed
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
    return true;
  }

  return false;
}

// Function to sweep the leg from one extreme to the other while recording the minimum and maximum smoothed values
void sweepLegAndRecordExtremes()
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
void calibrateJoint()
{
  Serial.println("Starting calibration...");

  // Move the leg from fully open to fully closed position while recording the extremes
  sweepLegAndRecordExtremes();

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

// Function to be executed if the message is "SOS"
void safeMode()
{
  Serial.println("Executing safemode: SOS command received");
  // Add your code here to handle the safeMode command
}

void storePIDToEEPROM(float Kp, float Ki, float Kd)
{
  EEPROM.put(addrKp, Kp);
  EEPROM.put(addrKi, Ki);
  EEPROM.put(addrKd, Kd);
}

// This function is called whenever data is received from the master.
void receiveEvent(int numBytes)
{

  // filters for angle signals
  if (numBytes == 1)
  {
    int receivedNumber = Wire.read(); // Read the incoming byte as an integer
    if (receivedNumber >= 0 && receivedNumber <= 255)
    {
      targetAngle = receivedNumber;
      newDataReceived = true; // Set the flag indicating new data received so that the main loop runs target asap
    }
  }

  // If three bytes are received then check if its calling CAL for calibrate function or SOS for safe mode
  else if (numBytes == 3)
  {                          // If 3 bytes are received, check for "PID"
    char receivedMessage[4]; // Array to hold the received message
    for (int i = 0; i < 3; i++)
    {
      receivedMessage[i] = Wire.read(); // Read each byte into the array
    }
    receivedMessage[3] = '\0'; // Null-terminate the string

    if (strcmp(receivedMessage, "CAL") == 0)
    {
      calibrateJoint(); // Call calibrateJoint if the message is "CAL"
    }
    if (strcmp(receivedMessage, "SOS") == 0)
    {
      safeMode(); // Call safeMode if the message is "SOS"
    }
  }
  // Handle a message containing PID tuning parameters
  else if (numBytes == 7)
  {                                // 1 byte identifier + 6 bytes for Kp, Ki, Kd
    char identifier = Wire.read(); // Read the first byte (identifier)

    if (identifier == 'P')
    { // If the identifier indicates PID tuning parameters
      byte KpBytes[2], KiBytes[2], KdBytes[2];

      // Read the bytes for each PID parameter
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
}
