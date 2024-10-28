#include <NewPing.h>           // Library to interface with ultrasonic sensors for distance measurements
#include <LiquidCrystal_I2C.h> // I2C LCD library to control LCD display
#include <Wire.h>              // I2C communication library

// Pin numbers for left, middle, and right ultrasonic sensors for edge detection
const int echo_L = 2;
const int trig_L = 3;
const int echo_M = 4;
const int trig_M = 5;
const int echo_R = 7;
const int trig_R = 8;

// Pin numbers for motor control (L1, L2 control the left motor, R1, R2 control the right motor)
const int L1 = 6;
const int L2 = 9;
const int R1 = 10;
const int R2 = 11;

// Pin for controlling the pump used for wet cleaning mode
const int pump = 13;

// Pin numbers for additional ultrasonic sensor to detect cracks on the floor
const int echo_Crack = A0;
const int trig_Crack = A1;

// Pin for the SDS011 dust sensor to monitor air quality
const int dustSensorPin = A2;

// Pin for the water level sensor to check the cleaning water level
const int waterLevelPin = A3;
const int waterLevelThreshold = 200; // Threshold to indicate low water level

// Motor speed and sensor distance variables
int motor_speed = 255;                                                  // Motor speed (range: 125 to 255)
int max_distance = 200;                                                 // Maximum detection range for ultrasonic sensors (in cm)
int distance_L = 0, distance_M = 0, distance_R = 0, distance_Crack = 0; // Variables to store distance readings from sensors

// Serial input variable
char incomingByte;

// Flags for cleaning mode and water level
bool isWetCleaning = false; // True when in wet cleaning mode
bool isWaterLow = false;    // True when the water level is below the threshold

// Time tracking variables for various periodic checks and actions
unsigned long lastPumpTime = 0;
unsigned long lastSensorCheckTime = 0;
unsigned long lastDustCheckTime = 0;

// Initialize ultrasonic sensor objects with trigger and echo pins
NewPing sonar_L(trig_L, echo_L, max_distance);
NewPing sonar_M(trig_M, echo_M, max_distance);
NewPing sonar_R(trig_R, echo_R, max_distance);
NewPing sonar_Crack(trig_Crack, echo_Crack, max_distance);

// Initialize LCD with I2C address and display size
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup()
{
  // Set motor control and pump pins as outputs
  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);
  pinMode(R1, OUTPUT);
  pinMode(R2, OUTPUT);
  pinMode(pump, OUTPUT);

  // Set the initial motor state to stop
  setMotorState(LOW, LOW, LOW, LOW);
  digitalWrite(pump, LOW);

  // Initialize LCD display and print initial message
  lcd.begin(16, 2);
  lcd.print("Choose Mode");
  Serial.begin(9600); // Start serial communication for mode selection
  delay(2000);        // Delay to display the initial message
}

void loop()
{
  // Check for incoming data via serial to select a mode
  if (Serial.available() > 0)
  {
    char mode = Serial.read();
    selectMode(mode);
  }

  unsigned long currentMillis = millis(); // Get the current time

  // Periodic sensor checks (every 70ms)
  if (currentMillis - lastSensorCheckTime >= 70)
  {
    readAllSensors();
    checkCrackDetection();
    handleMovement();
    lastSensorCheckTime = currentMillis;
  }

  // Handle pump operation in wet cleaning mode
  if (isWetCleaning)
  {
    checkWaterLevel(); // Check water level sensor reading
    handlePump(currentMillis);
  }

  // Periodic dust level checks (every 5 seconds)
  if (currentMillis - lastDustCheckTime >= 5000)
  {
    checkDustLevel();
    lastDustCheckTime = currentMillis;
  }
}

// Function to select cleaning mode based on user input
void selectMode(char mode)
{
  if (mode == '1')
  {
    isWetCleaning = false;
    lcd.clear();
    lcd.print("Dry Cleaning");
  }
  else if (mode == '2')
  {
    isWetCleaning = true;
    lcd.clear();
    lcd.print("Wet Cleaning");
  }
  delay(1000); // Display message for 1 second
}

// Function to read all ultrasonic sensors and update distance variables
void readAllSensors()
{
  distance_L = readSensor(sonar_L);
  distance_M = readSensor(sonar_M);
  distance_R = readSensor(sonar_R);
  distance_Crack = readSensor(sonar_Crack);
}

// Function to check if a crack is detected by the crack detection sensor
void checkCrackDetection()
{
  if (distance_Crack <= 10)
  {                     // If distance to crack is 10cm or less
    displayDistances(); // Display distance readings on LCD
    delay(2000);        // Wait for 2 seconds
  }
}

// Function to handle robot movement based on sensor readings
void handleMovement()
{
  if (distance_M <= 20)
  { // Obstacle detected in the middle
    if (distance_R > distance_L)
    {
      if (distance_R <= 20 && distance_L <= 20)
      {
        moveBackward();
      }
      else
      {
        moveBackward();
        delay(500);
        moveRight();
      }
    }
    else
    {
      if (distance_R <= 20 && distance_L <= 20)
      {
        moveBackward();
      }
      else
      {
        moveBackward();
        delay(500);
        moveLeft();
      }
    }
  }
  else if (distance_R <= 15)
  {
    moveLeft();
  }
  else if (distance_L <= 15)
  {
    moveRight();
  }
  else
  {
    moveForward();
  }
}

// Function to handle pump operation based on water level
void handlePump(unsigned long currentMillis)
{
  if (!isWaterLow)
  {
    if (currentMillis - lastPumpTime >= 10000)
    { // Pump every 10 seconds
      activatePump();
      lastPumpTime = currentMillis;
    }
  }
  else
  {
    deactivatePump();
  }
}

// Function to check and respond to dust level readings
void checkDustLevel()
{
  int dustLevel = readDustSensor();
  if (dustLevel > 100)
  { // High dust level detected
    lcd.clear();
    lcd.print("Dust detected");
    lcd.setCursor(0, 1);
    lcd.print("Clean again!");
    moveStop(); // Stop the robot
    delay(2000);
  }
}

// Helper function to read distance from a given ultrasonic sensor
int readSensor(NewPing &sonar)
{
  int distance = sonar.ping_cm();
  return (distance == 0) ? 250 : distance; // If no reading, assume max distance (250cm)
}

// Function to read dust level using analog sensor
int readDustSensor()
{
  return analogRead(dustSensorPin);
}

// Function to check if the water level is below the threshold
void checkWaterLevel()
{
  int waterLevel = analogRead(waterLevelPin);
  isWaterLow = (waterLevel < waterLevelThreshold);
}

// Function to display sensor distance readings on the LCD
void displayDistances()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("L=");
  lcd.print(distance_L);
  lcd.print(" M=");
  lcd.print(distance_M);
  lcd.print(" R=");
  lcd.print(distance_R);
  lcd.setCursor(0, 1);
  lcd.print("Crack:");
  lcd.print(distance_Crack);
}

// Movement functions for robot navigation
void moveForward()
{
  setMotorState(LOW, motor_speed, motor_speed, LOW);
}

void moveBackward()
{
  setMotorState(motor_speed, LOW, LOW, motor_speed);
}

void moveLeft()
{
  setMotorState(motor_speed, LOW, motor_speed, LOW);
}

void moveRight()
{
  setMotorState(LOW, motor_speed, LOW, motor_speed);
}

void moveStop()
{
  setMotorState(LOW, LOW, LOW, LOW);
}

// Helper function to set motor states based on movement direction
void setMotorState(int L1State, int L2State, int R1State, int R2State)
{
  analogWrite(L1, L1State);
  analogWrite(L2, L2State);
  analogWrite(R1, R1State);
  analogWrite(R2, R2State);
}

// Function to activate the water pump for wet cleaning
void activatePump()
{
  digitalWrite(pump, HIGH);
  lcd.setCursor(0, 1);
  lcd.print("Spraying Water");
}

// Function to deactivate the water pump
void deactivatePump()
{
  digitalWrite(pump, LOW);
  lcd.setCursor(0, 1);
  lcd.print("Dry Cleaning");
}
