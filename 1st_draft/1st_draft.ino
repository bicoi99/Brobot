/*
  1st draft
*/

//--------------- LIBRARIES AND DEFINITIONS ------------------

#include <Wire.h>                                             // I2C bus library
#include <math.h>                                             // math library
#include <Servo.h>                                            // Servo library


// I2C
#define MD25ADDRESS         0x58                              // address of the MD25
#define SPEED1              0x00                              // motor 1 speed (mode 0,1) or both motors speed (mode 2,3)
#define SPEED2              0x01                              // motor 2 speed (mode 0,1) or both motors speed (mode 2,3)
#define ENC1                0x02                              // encoder 1 position
#define ENC2                0x06                              // encoder 2 position
#define ACCELERATION        0xE                               // optional Acceleration register
#define COMMAND             0x10                              // reset of encoder counts and module address changes
#define MODE                0xF                               // mode of operation 

// Servo
Servo servo1;
Servo servo2;
#define HOLD1 30 // Hold atom for servo 1
#define DROP1 5  // Drop atom for servo 1
#define HOLD2 5 // Hold atom for servo 2
#define DROP2 27 // Drop atom for servo 2


//-------------- CONSTANTS (doesn't change) -----------------

const double circumference = 10.3 * M_PI; // circumference of wheel
const int globalAcceleration = 1;
const int straightSpeed = 30; // speed of wheel during a straight
const int straightOffset = 70;
const int cornerSpeed = 20;
const int cornerOffset = 0;
const int correctionSpeed = 5;
const int correctionOffset = 0;
const float d1 = 13; // radius of wheel arc
const float d2 = 13;
const int colourPin = 2; // Pin that control which side to go (1 = Purple, 0 = Yellow)
const int startPin = 3; // Pin that when pull will start the robot motion
const int trigPinFront = 6;
const int echoPinFront = 7;
const int trigPinBack = 3;
const int echoPinBack = 4;
const long stopTime = 95000; 
const float minGap = 7.00;
const unsigned long sonicTimeOut = 50 / 0.01715;
const int correctionCountMax = 1;

//---------------- VARIABLES (will change) -------------------

unsigned long currentMillis;
unsigned long previousMillis = 0;
unsigned long startMillis;
int correctionCount;

//------------------ FUNCTION PROTOTYPES ---------------------

// Motor functions
void resetEncoder();
long encoder();
void stopMotor();

// Sides functions
void purple();
void yellow();

// Supporting functions
int dis2tic(double distance);
bool timesUp();
bool collision_forward();
bool collision_backward();
void transmission(byte address, int value);

// Movement functions
void drive(int ticks, int speed1, int speed2, int offset,
           int acceleration = globalAcceleration,
           bool avoid = false, bool forward = false, 
           bool correction = true);
void straight(float distance, bool correction = true,
              bool avoid = true,
              int speed = straightSpeed,
              int acceleration = globalAcceleration,
              int offset = straightOffset);
void turn(float angle, int speed = cornerSpeed,
          int acceleration = globalAcceleration,
          int offset = cornerOffset);

// Action functions
void holdAtom();
void dropAtom();

//======================== SETUP =============================

void setup() {

  // Serial monitor
  Serial.begin(9600); // start serial monitor
  Serial.println("Eurobot 2019: Team Brobot");
  
  // I2C bus
  Wire.begin();                                               // start I2C
  delay(100);                                                 // wait for things to start up

  Wire.beginTransmission(MD25ADDRESS);                        // go to MD25 address
  Wire.write(MODE);                                           // go to mode selector
  Wire.write(0);                                              // select mode 2 where speed1 drives both motors and speed2 turns
  Wire.endTransmission();                                     // exit from MD25

  resetEncoder();                                             // function to reset encoder value to 0

  // Servos
  servo1.attach(9); // attach servo1 - left
  servo2.attach(10); // servo2 - right
  servo1.write(DROP1); // set servo to starting position
  servo2.write(DROP2);
  delay(100);

  // Assign Arduino pins
  pinMode(colourPin, INPUT_PULLUP);
  pinMode(startPin, INPUT_PULLUP);
  pinMode(trigPinFront,OUTPUT);
  pinMode(trigPinBack, OUTPUT);
  pinMode(echoPinFront, INPUT);
  pinMode(echoPinBack, INPUT);

  // Do nothing when start cord is not pulled
  //while (digitalRead(startPin) == LOW){/* do nothing */}

  // Record starting time
  startMillis = millis();
  Serial.print("Start: ");
  Serial.println(startMillis);
  
  // Decision between sides using a switch
  if (digitalRead(colourPin) == HIGH) {
    purple();
  } else if (digitalRead(colourPin) == LOW) {
    yellow();
  }
}

//----------------------- FUNCTIONS --------------------------
// Function that drive the wheels
void drive(int ticks, int speed1, int speed2, int offset,
           int acceleration, bool avoid, bool forward,
           bool correction) {
  Serial.println((speed1 > 0 && speed2 > 0) || (speed1 < 0 && speed2 < 0));
  Serial.print("Ticks: ");
  Serial.println(ticks);
  int target = ticks - offset;
//  Serial.print("Target: ");
//  Serial.println(target);
  resetEncoder();                                             // reset encoder to 0
  delay(50);
//  while (abs(encoder(1)) < 10 && abs(encoder(2)) < 10){
//    transmission(ACCELERATION, acceleration); // acceleration
//    transmission(SPEED1, 128 + 5); // right wheel
//    transmission(SPEED2, 128 + 5); // left wheel
//  }
  while (abs(encoder(1)) < target && abs(encoder(2)) < target) {
    if (timesUp()) {
      return;
    }
    if (avoid) {
      if (collision(forward)) {
        stopMotor();
      } else {
        transmission(ACCELERATION, acceleration); // acceleration
        transmission(SPEED1, 128 + speed1); // right wheel
        transmission(SPEED2, 128 + speed2); // left wheel
      }
    } else {
      transmission(ACCELERATION, acceleration); // acceleration
      transmission(SPEED1, 128 + speed1); // right wheel
      transmission(SPEED2, 128 + speed2); // left wheel
    }
    // Serial monitor
    Serial.print("Encoders: ");
    Serial.print(encoder(1));
    Serial.print("\t");
    Serial.println(encoder(2));
  }
  // Post processing
  stopMotor();                                                // stop motor
  delay(200);
  Serial.print("After stopping: ");
  Serial.print(encoder(1));
  Serial.print("\t");
  Serial.println(encoder(2));

  if (correction){
    if (correctionCount < correctionCountMax) {
      correctionCount++;
      int correctionSpeed1, correctionSpeed2;
      int diff = abs((abs(encoder(1)) + abs(encoder(2))) / 2 - ticks);
      Serial.print("Diff: ");
      Serial.println(diff);
      if (speed1 > 0) {
        correctionSpeed1 = correctionSpeed;
      } else if (speed1 < 0) {
        correctionSpeed1 = -correctionSpeed;
      }
      if (speed2 > 0) {
        correctionSpeed2 = correctionSpeed;
      } else if (speed2 < 0) {
        correctionSpeed2 = -correctionSpeed;
      }

//      if ((speed1 > 0 && speed2 > 0) || (speed1 < 0 && speed2 < 0)){
//        if (abs(encoder(1)) > abs(encoder(2))){
//          drive(abs(encoder(1)) - abs(encoder(2)), -correctionSpeed1, 0, correctionOffset);
//        } else if (abs(encoder(1)) < abs(encoder(2))){
//          drive(abs(encoder(1)) - abs(encoder(2)), 0, -correctionSpeed2, correctionOffset);
//        }
//      }
  
      if (abs(encoder(1)) > ticks && abs(encoder(2)) > ticks) {
        drive(diff, -correctionSpeed1, -correctionSpeed2, correctionOffset);
      } else if (abs(encoder(1)) < ticks && abs(encoder(2)) < ticks) {
        drive(diff, correctionSpeed1, correctionSpeed2, correctionOffset);
      }
    }
  }

  correctionCount = 0;
}


// Function that drive straight
void straight(float distance, bool correction, bool avoid,
              int speed, int acceleration, int offset) {
  bool forward = true;
  if (distance < 0) {
    forward = false;
    drive(-dis2tic(distance), -speed, -speed, offset, acceleration, avoid, forward, correction);
  } else {
    drive(dis2tic(distance), speed, speed, offset, acceleration, avoid, forward, correction);
  }
}


// Function that turn through and angle (degrees)
void turn(float angle, int speed, int acceleration,
          int offset) {
  if (angle > 0) { // left turn
    drive(dis2tic(angle * (M_PI / 180) * d1), -speed, speed, offset,
          acceleration);
  } else {
    drive(-dis2tic(angle * (M_PI / 180) * d1), speed, -speed, offset,
          acceleration);
  }
}


// Function to reset encoder value to 0
void resetEncoder() {
  transmission(COMMAND, 0x20);
  delay(50);                                                  // wait for things to settles
}


// Function to get the value(number of ticks = degrees of wheel turn) of the encoder 1
long encoder(int encNumber) {
  Wire.beginTransmission(MD25ADDRESS);                        // go to MD25 address
  if (encNumber == 1) {
    Wire.write(ENC1);
  } else if (encNumber == 2) {
    Wire.write(ENC2);
  }
  Wire.endTransmission();

  Wire.requestFrom(MD25ADDRESS, 4);                           // request 4 bytes from MD25
  while (Wire.available() < 4) {
    /*do nothing*/
  };              // wait for 4 bytes to arrive
  long dist = Wire.read();                                   // read first byte
  dist <<= 8;                                                // shift the dist1 variable 1 byte to make room to store 2nd byte
  dist += Wire.read();                                       // read second byte
  dist <<= 8;
  dist += Wire.read();                                       // read third byte
  dist <<= 8;
  dist += Wire.read();                                       // read fourth byte
  delay(5);                                                   // wait for all the bytes to come through

  return (dist);
}


// Function to stop motor
void stopMotor() {
  transmission(ACCELERATION, 3);
  transmission(SPEED1, 128);
  transmission(SPEED2, 128);
  delay(50);
}


// Purple side function
void purple() {
  straight(61);
  turn(-90);
  straight(-15, false);
  straight(56);
  turn(90);
  straight(20, false, false);
  holdAtom();
  straight(-15, true, false);
  turn(-90);
  straight(-60, false);
  straight(10);
  turn(-90);
  straight(-50);
  straight(-15, false, false);
  straight(14);
  turn(110);
  straight(20);
  turn(-20);
  dropAtom();
  straight(-20);
  turn(180);
  straight(25, false, false);
  straight(-95);
  straight(25, true, false, 5);
}


// Yellow side function
void yellow() {
  straight(75);
  turn(90);
  straight(-15, false);
  straight(56);
  turn(-90);
  straight(20, false, false);
  holdAtom();
  straight(-15, true, false);
  turn(90);
  straight(-60, false);
  straight(10);
  turn(90);
  straight(-50);
  straight(-15, false, false);
  straight(14);
  turn(-110);
  straight(20);
  turn(20);
  dropAtom();
  straight(-20);
  turn(-180);
  straight(25, false, false);
  straight(-95);
  straight(25, true, false, 5);
}


// Function that converts distance(cm) into number of encoder ticks
int dis2tic(double distance) {
  double rev = distance / circumference;                      // number of revolutions needed to turn
  int ticks = int(rev * 360);                                 // number of ticks encoder need to count
  return ticks;
}


// Function to determine if 100s is up or not
bool timesUp() {
  unsigned long currentMillis = millis();
  if (currentMillis - startMillis >= stopTime) {
    return true;
  } else {
    return false;
  }
}


// Function to find distances from ultrasonic sensors
bool collision(bool forward) {
  float duration, distance;

  if (forward) {
    // Send out signal
    digitalWrite(trigPinFront, LOW);
    delayMicroseconds(5);
    digitalWrite(trigPinFront, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinFront, LOW);

    // Get signal
    duration = pulseIn(echoPinFront, HIGH, sonicTimeOut);
  } else {
    // Send out signal
    digitalWrite(trigPinBack, LOW);
    delayMicroseconds(5);
    digitalWrite(trigPinBack, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinBack, LOW);

    // Get signal
    duration = pulseIn(echoPinBack, HIGH, sonicTimeOut);
  }
  
  // Calculate distance in cm
  delay(10);
  distance = (duration / 2) * 0.0343;
//  Serial.print("Distance: ");
//  Serial.println(distance);

  if (distance == 0) {
    return false;
  } else if (distance < minGap){
    return true;
  } else{
    return false;
  }
}


// Function that transmit information to MD25 board
void transmission(byte address, int value) {
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission();
}


// Function to engage servos to grab atoms
void holdAtom(){
  if (timesUp()){
    return;
  }
  servo1.write(HOLD1);
  servo2.write(HOLD2);
  delay(50);
}


// Function to disengage servos to release atoms
void dropAtom(){
  if (timesUp()){
    return;
  }
  servo1.write(DROP1);
  servo2.write(DROP2);
  delay(50);
}


// loop function is not needed
void loop() {}
