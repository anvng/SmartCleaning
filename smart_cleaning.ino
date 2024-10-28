#include <NewPing.h>    //import libraries
#include <LiquidCrystal_I2C.h>     // I2C LCD library
#include <Wire.h>                 // I2C library

//initialize pin numbers for edge detection ultrasonic sensors
const int echo_L = 2;                                
const int trig_L = 3;
const int echo_M = 4;
const int trig_M = 5;
const int echo_R = 7;
const int trig_R = 8;

//initialise pin numbers for movement control
const int L1 = 6;
const int L2 = 9;
const int R1 = 10;
const int R2 = 11;
//pump control for wet cleaning
const int pump = 13;
// Additional sensor for crack detection
const int echo_Crack = A0;
const int trig_Crack = A1;

// SDS011 dust sensor pin
const int dustSensorPin = A2; // Dust sensor

// Water level sensor pin
const int waterLevelPin = A3;
const int waterLevelThreshold = 200; // Threshold for low water level

int motor_speed = 255;    //speed of the motor can be set between 125 (minimum) and 255 (maximum)
int max_distance = 200;   //max distance of ultrasonic sensors is set to 200cm
int distance_L = 0, distance_M = 0, distance_R = 0, distance_Crack = 0;
char incomingByte;

bool isWetCleaning = false;  // Boolean flag for mode selection
bool isWaterLow = false;     // Boolean flag for low water level detection
unsigned long lastPumpTime = 0;
unsigned long lastSensorCheckTime = 0;
unsigned long lastDustCheckTime = 0;

NewPing sonar_L(trig_L, echo_L, max_distance);    
NewPing sonar_M(trig_M, echo_M, max_distance);
NewPing sonar_R(trig_R, echo_R, max_distance);
NewPing sonar_Crack(trig_Crack, echo_Crack, max_distance);  
LiquidCrystal_I2C lcd(0x27, 16, 2); 

void setup() {  
  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);
  pinMode(R1, OUTPUT);
  pinMode(R2, OUTPUT);
  pinMode(pump, OUTPUT);
  setMotorState(LOW, LOW, LOW, LOW);
  digitalWrite(pump, LOW);

  lcd.begin(16, 2); 
  lcd.print("Choose Mode");
  Serial.begin(9600);
  delay(2000);
}

void loop() {
  if (Serial.available() > 0) {
    char mode = Serial.read();
    selectMode(mode);
  }

  unsigned long currentMillis = millis();
  if (currentMillis - lastSensorCheckTime >= 70) {
    readAllSensors();
    checkCrackDetection();
    handleMovement();
    lastSensorCheckTime = currentMillis;
  }

  if (isWetCleaning) {
    checkWaterLevel();
    handlePump(currentMillis);
  }

  if (currentMillis - lastDustCheckTime >= 5000) {
    checkDustLevel();
    lastDustCheckTime = currentMillis;
  }
}

void selectMode(char mode) {
  if (mode == '1') {
    isWetCleaning = false;
    lcd.clear();
    lcd.print("Dry Cleaning");
  } else if (mode == '2') {
    isWetCleaning = true;
    lcd.clear();
    lcd.print("Wet Cleaning");
  }
  delay(1000);
}

void readAllSensors() {
  distance_L = readSensor(sonar_L);
  distance_M = readSensor(sonar_M);
  distance_R = readSensor(sonar_R);
  distance_Crack = readSensor(sonar_Crack);
}

void checkCrackDetection() {
  if (distance_Crack <= 10) {
    displayDistances();
    delay(2000);
  }
}

void handleMovement() {
  if (distance_M <= 20) {
    if (distance_R > distance_L) {
      if (distance_R <= 20 && distance_L <= 20) {
        moveBackward();
      } else {
        moveBackward();
        delay(500);
        moveRight();
      }
    } else {
      if (distance_R <= 20 && distance_L <= 20) {
        moveBackward();
      } else {
        moveBackward();
        delay(500);
        moveLeft();
      }
    }
  } else if (distance_R <= 15) {
    moveLeft();
  } else if (distance_L <= 15) {
    moveRight();
  } else {
    moveForward();
  }
}

void handlePump(unsigned long currentMillis) {
  if (!isWaterLow) {
    if (currentMillis - lastPumpTime >= 10000) {
      activatePump();
      lastPumpTime = currentMillis;
    }
  } else {
    deactivatePump();
  }
}

void checkDustLevel() {
  int dustLevel = readDustSensor();
  if (dustLevel > 100) {
    lcd.clear();
    lcd.print("Dust detected");
    lcd.setCursor(0, 1);
    lcd.print("Clean again!");
    moveStop();
    delay(2000);
  }
}

int readSensor(NewPing &sonar) {
  int distance = sonar.ping_cm();
  return (distance == 0) ? 250 : distance;
}

int readDustSensor() {
  return analogRead(dustSensorPin);
}

void checkWaterLevel() {
  int waterLevel = analogRead(waterLevelPin);
  isWaterLow = (waterLevel < waterLevelThreshold);
}

void displayDistances() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("L="); lcd.print(distance_L);
  lcd.print(" M="); lcd.print(distance_M);
  lcd.print(" R="); lcd.print(distance_R);
  lcd.setCursor(0, 1);
  lcd.print("Crack:"); lcd.print(distance_Crack);
}

void moveForward() {
  setMotorState(LOW, motor_speed, motor_speed, LOW);
}

void moveBackward() {
  setMotorState(motor_speed, LOW, LOW, motor_speed);
}

void moveLeft() {
  setMotorState(motor_speed, LOW, motor_speed, LOW);
}

void moveRight() {
  setMotorState(LOW, motor_speed, LOW, motor_speed);
}

void moveStop() {
  setMotorState(LOW, LOW, LOW, LOW);
}

void setMotorState(int L1State, int L2State, int R1State, int R2State) {
  analogWrite(L1, L1State); 
  analogWrite(L2, L2State);
  analogWrite(R1, R1State);
  analogWrite(R2, R2State);
}

void activatePump() {
  digitalWrite(pump, HIGH);
  lcd.setCursor(0, 1);
  lcd.print("Spraying Water");
}

void deactivatePump() {
  digitalWrite(pump, LOW);
  lcd.setCursor(0, 1);
  lcd.print("Dry Cleaning");
}
