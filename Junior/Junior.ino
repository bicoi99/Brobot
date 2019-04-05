/*
  Junior code
*/

//--------------- LIBRARIES AND DEFINITIONS ------------------

#include <Wire.h>                                             // I2C bus library
#include <math.h>                                             // math library
#include <Servo.h>                                            // Servo library
#include <IRLibAll.h>


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
Servo servoG;
Servo servoR;
Servo servoL;
IRsend IRSender;
#define IDLE_R 90
#define ACT_R 0
#define IDLE_L 0
#define ACT_L 80
#define DROP 10
#define HOLD 45


//-------------- CONSTANTS (doesn't change) -----------------

const double circumference = 10.3 * M_PI; // circumference of wheel
const int globalAcceleration = 1;
const int straightSpeed = 30; // speed of wheel during a straight
const int straightOffset = 30; // 18
const int cornerSpeed = 20;
const int cornerOffset = 0;
const int correctionSpeed = 5;
const int correctionOffset = 0; //3
const float d1 = 11; // radius of wheel arc
const float d2 = 11;
const int colourPin = 11; // Pin that control which side to go (1 = Purple, 0 = Yellow)
const int startPin = 2; // Pin that when pull will start the robot motion
const int trigPinBack = 6;
const int echoPinBack = 7;
const int trigPinFront = 4;
const int echoPinFront = 5;
const float minGap = 17; // 7 cm
const long stopTime = 95000; // 95 s
const int correctionCountMax = 1;
const unsigned long sonicTimeOut = 50 / 0.01715;

//---------------- VARIABLES (will change) -------------------

float wheel1, wheel2; // offsets account for deceleration error
int motor1, motor2;
int offsetSpeed2;
unsigned long currentMillis;
unsigned long previousMillis = 0;
unsigned long startMillis;
int correctionCount = 0;

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
  servoG.attach(8);
  servoR.attach(9);
  servoL.attach(10);

  servoG.write(DROP);
  servoR.write(IDLE_R);
  servoL.write(IDLE_L);
  delay(100);

  // Assign Arduino pins
  pinMode(colourPin, INPUT_PULLUP);
  pinMode(startPin, INPUT_PULLUP);
  pinMode(trigPinFront, OUTPUT);
  pinMode(trigPinBack, OUTPUT);
  pinMode(echoPinFront, INPUT);
  pinMode(echoPinBack, INPUT);

  // Do nothing when start cord is not pulled
  //while (digitalRead(startPin) == LOW){/* do nothing */}

  // Record starting time
  startMillis = millis();
  Serial.print("Start time: ");
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
//  Serial.print("Ticks: ");
//  Serial.println(ticks);
  int target = ticks - offset;
//  Serial.print("Target: ");
//  Serial.println(target);
  resetEncoder();                                             // reset encoder to 0
  delay(50);
  while (abs(encoder(1)) < target && abs(encoder(2)) < target) {
    if (timesUp()) {
      return;
    }
    if (avoid) {
      Serial.print("Collision: ");
      Serial.println(collision(forward));
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
  
      if (abs(encoder(1)) > target && abs(encoder(2)) > target) {
        drive(diff, -correctionSpeed1, -correctionSpeed2, correctionOffset);
      } else if (abs(encoder(1)) < target && abs(encoder(2)) < target) {
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
  IRtransmission();
  straight(25, false);
  straight(10, false, false);
  straight(-20, false, false);
  turn(90);
  straight(-18, false);
  straight(160);
  turn(90);
  straight(-25, false, false);
  straight(14);
  turn(90);
  servoL.write(ACT_L);
  delay(600);
  straight(-8, false, true, straightSpeed - 10);
  servoL.write(IDLE_L);
  straight(-19);
  turn(-90);
  straight(-20, false, false);
  straight(20);
  turn(90);
  straight(-39, true, true, straightSpeed - 10);
  turn(90);
  straight(33, false, false);
  servoG.write(HOLD);
  turn(5, 5);
  delay(500);
  straight(-20);
  turn(-90);
  straight(90);
  turn(-90);
  straight(-30, false, false);
  straight(130);
  straight(20, false, false);
  servoG.write(DROP);
}


// Yellow side function
void yellow() {
  IRtransmission();
  straight(25, false);
  straight(10, false, false);
  straight(-20, false, false);
  turn(-90);
  straight(-20, false);
  straight(157);
  turn(-90);
  straight(-25, false, false);
  straight(14, true);
  turn(-90);
  servoR.write(ACT_R);
  delay(600);
  straight(-8, false, true, straightSpeed - 10);
  servoR.write(IDLE_R);
  straight(-19);
  turn(90);
  straight(-20, false, false);
  straight(20);
  turn(-90);
  straight(-39, true, true, straightSpeed - 0);
  turn(-87);
  straight(25, false, false);
  servoG.write(HOLD);
  straight(-60);
  turn(90);
  straight(93);
  turn(90);
  straight(-55);
  straight(-20, false, false);
  straight(130);
  straight(20, false, false);
  servoG.write(DROP);
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
  Serial.print("Distance: ");
  Serial.println(distance);

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


// Function that send IR signal
void IRtransmission(){
  IRSender.send(SONY,0xa8bca, 20);
  delay(100);
  IRSender.send(SONY,0xa8bca, 20);
}


// loop function is not needed
void loop() {}
