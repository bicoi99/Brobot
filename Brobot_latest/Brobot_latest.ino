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

enum FUN { fwd, bwd, rtn, ltn}; // definition of actions
const double circumference = 10.3 * M_PI; // circumference of wheel
int straightSpeed = 30; // speed of wheel during a straight
const int cornerSpeed = 20;
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
const unsigned long sonicTimeOut = 100 / 0.01715;

//---------------- VARIABLES (will change) -------------------

float wheel1, wheel2; // offsets account for deceleration error
int motor1, motor2;
int offsetSpeed2;
unsigned long currentMillis;
unsigned long previousMillis = 0;
unsigned long startMillis;
float offsetS = (15.0 / 360.0) * float(circumference);
float offsetC = (9.0 / 360.0) * float(circumference);

//------------------ FUNCTION PROTOTYPES ---------------------

// Motor functions
void resetEncoder();
long encoder1();
long encoder2();
void stopMotor();

// Sides functions
void purple();
void yellow();

// Supporting functions
int dis2tic(double distance);
bool timesUp();
bool collision(bool forward);
void transmission(byte address, int value);

// Movement functions
void forward(bool avoid);
void backward(bool avoid);
void rightTurn();
void leftTurn();
void movement(FUN fun, double distance, float angle, bool avoid = true);

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
  while (digitalRead(startPin) == LOW){/* do nothing */}

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

  
//  straightSpeed = 40;
//  movement(bwd, 90, 0);
//  movement(rtn, 0, 10 * (M_PI / 180));
//  movement(ltn, 0, 5 * (M_PI / 180));
//  straightSpeed = 30;
//  movement(fwd, 75, 0);
//  movement(ltn, 0, 90 * (M_PI / 180));
//  movement(bwd, 20, 0);
//  movement(fwd, 10, 0);
//  movement(rtn, 0, 90 * (M_PI / 180));
//  movement(fwd, 20, 0);
//  movement(bwd, 110, 0);
}

//----------------------- FUNCTIONS --------------------------

// Function to reset encoder value to 0
void resetEncoder() {
  Wire.beginTransmission(MD25ADDRESS);                        // go to MD25 address
  Wire.write(COMMAND);                                        // go to command register
  Wire.write(0x20);                                           // reset encoder value to 0
  Wire.endTransmission();                                     // exit from MD25
  delay(50);                                                  // wait for things to settles
}


// Function to get the value(number of ticks = degrees of wheel turn) of the encoder 1
long encoder1() {
  Wire.beginTransmission(MD25ADDRESS);                        // go to MD25 address
  Wire.write(ENC1);                                           // send a byte to read encoder 1 value
  Wire.endTransmission();

  Wire.requestFrom(MD25ADDRESS, 4);                           // request 4 bytes from MD25
  while (Wire.available() < 4) {/*do nothing*/};              // wait for 4 bytes to arrive
  long dist1 = Wire.read();                                   // read first byte
  dist1 <<= 8;                                                // shift the dist1 variable 1 byte to make room to store 2nd byte
  dist1 += Wire.read();                                       // read second byte
  dist1 <<= 8;
  dist1 += Wire.read();                                       // read third byte
  dist1 <<= 8;
  dist1 += Wire.read();                                       // read fourth byte
  delay(5);                                                   // wait for all the bytes to come through

  return (dist1);
}


// Function to get the value of the encoder 2
long encoder2() {
  Wire.beginTransmission(MD25ADDRESS);                        // go to MD25 address
  Wire.write(ENC2);                                           // send a byte to read encoder 2 value
  Wire.endTransmission();

  Wire.requestFrom(MD25ADDRESS, 4);                           // request 4 bytes from MD25
  while (Wire.available() < 4) {/*do nothing*/};              // wait for 4 bytes to arrive
  long dist2 = Wire.read();                                   // read first byte
  dist2 <<= 8;                                                // shift the dist2 variable 2 byte to make room to store 2nd byte
  dist2 += Wire.read();                                       // read second byte
  dist2 <<= 8;
  dist2 += Wire.read();                                       // read third byte
  dist2 <<= 8;
  dist2 += Wire.read();                                       // read fourth byte
  delay(5);                                                   // wait for all the bytes to come through

  return (dist2);
}


// Function to stop motor
void stopMotor() {
  // Set acceleration
  Wire.beginTransmission(MD25ADDRESS);                        // go to MD25 address
  Wire.write(ACCELERATION);                                   // go to acceleration register
  Wire.write(10);                                             // set acceleration to acceleration register 10
  Wire.endTransmission();

  // Set both motors speed
  Wire.beginTransmission(MD25ADDRESS);                        // go to MD25 address
  Wire.write(SPEED1);                                         // go to speed1 register
  Wire.write(128);                                            // set speed to 0
  Wire.endTransmission();

  // Stop turning motors
  Wire.beginTransmission(MD25ADDRESS);                        // go to MD25 address
  Wire.write(SPEED2);                                         // go to speed2 register
  Wire.write(128);                                            // set speed to 0
  Wire.endTransmission();

  delay(50);
}


// Purple side function
void purple() {
  movement(fwd, 75, 0);
  movement(rtn, 0, 90 * (M_PI / 180));
  movement(bwd, 15, 0);
  movement(fwd, 56, 0);
  movement(ltn, 0, 90 * (M_PI / 180));
  movement(fwd, 19.3, 0, false);
  holdAtom();
  movement(bwd, 15, 0, false);
  movement(rtn, 0, 90 * (M_PI / 180));
  movement(bwd, 60, 0);
  movement(fwd, 10, 0);
  movement(rtn, 0, 90 * (M_PI / 180));
  movement(bwd, 60, 0);
  movement(fwd, 14, 0);
  movement(ltn, 0, 110 * (M_PI / 180));
  movement(fwd, 20, 0);
  movement(rtn, 0, 20 * (M_PI / 180));
  dropAtom();
  movement(bwd, 20, 0);
  movement(rtn, 0, 180 * (M_PI / 180));
  movement(fwd, 25, 0);
  movement(bwd, 95, 0);
  straightSpeed = 5;
  movement(rtn, 0, 10 * (M_PI / 180));
  movement(bwd, 25, 0);
  straightSpeed = 30;
}


// Yellow side function
void yellow() {
  movement(fwd, 75, 0);
  movement(ltn, 0, 92 * (M_PI / 180));
  movement(bwd, 15, 0);
  movement(fwd, 57 , 0);
  movement(rtn, 0, 91 * (M_PI / 180));
  movement(fwd, 19.3, 0, false);
  holdAtom();
  movement(bwd, 15, 0, false);
  movement(ltn, 0, 90 * (M_PI / 180));
  movement(bwd, 60, 0);
  movement(fwd, 10, 0);
  movement(ltn, 0, 90 * (M_PI / 180));
  movement(bwd, 60, 0);
  movement(fwd, 14, 0);
  movement(rtn, 0, 110 * (M_PI / 180));
  movement(fwd, 20, 0);
  movement(ltn, 0, 20 * (M_PI / 180));
  dropAtom();
  movement(bwd, 20, 0);
  movement(ltn, 0, 185 * (M_PI / 180));
  movement(fwd, 25, 0);
  movement(bwd, 90, 0);
  straightSpeed = 5;
  movement(bwd, 35, 0);
  straightSpeed = 30;
}


// Function that converts distance(cm) into number of encoder ticks
int dis2tic(double distance) {
  double rev = distance / circumference;                      // number of revolutions needed to turn
  int ticks = int(rev * 360);                                 // number of ticks encoder need to count
  return ticks;
}


/*// Function to go forward
void forward() {
  do {
//    // Time check
//    if (timesUp()){
//      break;
//    }
    
    Wire.beginTransmission(MD25ADDRESS);                      // go to MD25 address
    Wire.write(ACCELERATION);                                 // go to acceleration register
    Wire.write(1);                                            // set acceleration register 1
    Wire.endTransmission();

    Wire.beginTransmission(MD25ADDRESS);                      // go to MD25 address
    Wire.write(SPEED1);                                       // go to speed1 register
    Wire.write(128 + straightSpeed);                          // set absolute value (> 128) of the motor speed for left wheel (1)
    Wire.endTransmission();

    // Set both motor speed
    Wire.beginTransmission(MD25ADDRESS);                      // go to MD25 address
    Wire.write(SPEED2);                                       // go to speed2 register (right)
    Wire.write(128 + straightSpeed);                          // set absolute value (>128) of motor speed for right wheel (2)
    Wire.endTransmission();
    
    // Serial monitor
    Serial.print("Encoders: ");
    Serial.print(encoder1());
    Serial.print("\t");
    Serial.println(encoder2());
  } while (encoder1() < dis2tic(wheel1) && encoder2() < dis2tic(wheel2));
}


// Function to go backward
void backward() {
  do {
//    // Time check
//    if (timesUp){
//      break;
//    }
    
    Wire.beginTransmission(MD25ADDRESS);                      // go to MD25 address
    Wire.write(ACCELERATION);                                 // go to acceleration register
    Wire.write(1);                                            // set acceleration register 1
    Wire.endTransmission();

    Wire.beginTransmission(MD25ADDRESS);                      // go to MD25 address
    Wire.write(SPEED2);                                       // go to speed1 register
    Wire.write(128 - straightSpeed);                          // set absolute value (< 128) of the motor speed for left wheel (1)
    Wire.endTransmission();

    // Set both motor speed
    Wire.beginTransmission(MD25ADDRESS);                      // go to MD25 address
    Wire.write(SPEED1);                                       // go to speed2 register
    Wire.write(128 - straightSpeed);                          // set absolute value (< 128) of motor speed for right wheel (2)
    Wire.endTransmission();
    
    // Serial monitor
    Serial.print("Encoders: ");
    Serial.print(encoder1());
    Serial.print("\t");
    Serial.println(encoder2());
    
  } while (encoder1() > dis2tic(wheel1) && encoder2() > dis2tic(wheel2));
}


// Function to turn right (degrees given)
void rightTurn() {
  do {
//    // Time check
//    if (timesUp){
//      break;
//    }
    
    Wire.beginTransmission(MD25ADDRESS);                      // go to MD25 address
    Wire.write(ACCELERATION);                                 // go to acceleration register
    Wire.write(5);                                            // set acceleration register 5
    Wire.endTransmission();

    Wire.beginTransmission(MD25ADDRESS);                      // go to MD25 address
    Wire.write(SPEED1);                                       // go to speed1 register
    Wire.write(128 + cornerSpeed);                            // set absolute value (forward) of motor speed for left wheel (1)
    Wire.endTransmission();

    // Set both motor speed
    Wire.beginTransmission(MD25ADDRESS);                      // go to MD25 address
    Wire.write(SPEED2);                                       // go to speed2 register
    Wire.write(128 - cornerSpeed);                            // set absolute value (backward) of motor speed for right wheel (2)
    Wire.endTransmission();
    
    // Serial monitor
    Serial.print("Encoders: ");
    Serial.print(encoder1());
    Serial.print("\t");
    Serial.println(encoder2());
  } while (encoder1() < dis2tic(wheel1));                     // move until left wheel moved enough distance since left move more
}


// Function to turn left
void leftTurn(){
  do {
//    // Time check
//    if (timesUp){
//      break;
//    }
    
    Wire.beginTransmission(MD25ADDRESS);                      // go to MD25 address
    Wire.write(ACCELERATION);                                 // go to acceleration register
    Wire.write(5);                                            // set acceleration register 5
    Wire.endTransmission();

    Wire.beginTransmission(MD25ADDRESS);                      // go to MD25 address
    Wire.write(SPEED1);                                       // go to speed1 register
    Wire.write(128 - cornerSpeed);                            // set absolute value (backward) of motor speed for left wheel (1)
    Wire.endTransmission();  
    
    // Set both motor speed
    Wire.beginTransmission(MD25ADDRESS);                      // go to MD25 address
    Wire.write(SPEED2);                                       // go to speed2 register
    Wire.write(128 + cornerSpeed);                            // set absolute value (forward) of motor speed for right wheel (2)
    Wire.endTransmission();
    
    // Serial monitor
    Serial.print("Encoders: ");
    Serial.print(encoder1());
    Serial.print("\t");
    Serial.println(encoder2());
  } while (encoder2() < dis2tic(wheel2));                     // move until right wheel moved enough distance since right move more
}*/


// Function to go forward
void forward(bool avoid){
  while (encoder1() < dis2tic(wheel1) && encoder2() < dis2tic(wheel2)){
    if (timesUp()){
      return;
    }
    if (avoid){
      if (collision(1)){
        stopMotor();
      } else{
        transmission(ACCELERATION, 1); // acceleration
        transmission(SPEED1, 128 + straightSpeed); // right wheel
        transmission(SPEED2, 128 + straightSpeed); // left wheel
        // Serial monitor
        Serial.print("Encoders: ");
        Serial.print(encoder1());
        Serial.print("\t");
        Serial.println(encoder2());
      }
    } else{
      transmission(ACCELERATION, 1); // acceleration
      transmission(SPEED1, 128 + straightSpeed); // right wheel
      transmission(SPEED2, 128 + straightSpeed); // left wheel
      // Serial monitor
      Serial.print("Encoders: ");
      Serial.print(encoder1());
      Serial.print("\t");
      Serial.println(encoder2());
    }
  }
}


// Function to go backward
void backward(bool avoid){
  while (encoder1() > dis2tic(wheel1) && encoder2() > dis2tic(wheel2)){
    if (timesUp()){
      return;
    }
    if (avoid){
      if (collision(0)){
        stopMotor();
      } else{
        transmission(ACCELERATION, 1); // acceleration
        transmission(SPEED1, 128 - straightSpeed); // right wheel
        transmission(SPEED2, 128 - straightSpeed); // left wheel
        // Serial monitor
        Serial.print("Encoders: ");
        Serial.print(encoder1());
        Serial.print("\t");
        Serial.println(encoder2());
      }
    } else{
      transmission(ACCELERATION, 1); // acceleration
      transmission(SPEED1, 128 - straightSpeed); // right wheel
      transmission(SPEED2, 128 - straightSpeed); // left wheel
      // Serial monitor
      Serial.print("Encoders: ");
      Serial.print(encoder1());
      Serial.print("\t");
      Serial.println(encoder2());
    }
  }
}


//Function to turn right
void rightTurn(){
  while (encoder1() < dis2tic(wheel1) && encoder2() > dis2tic(wheel2)){
    if (timesUp()){
      return;
    }
    transmission(ACCELERATION, 1); // acceleration
    transmission(SPEED1, 128 + cornerSpeed); // right wheel
    transmission(SPEED2, 128 - cornerSpeed); // left wheel
    // Serial monitor
    Serial.print("Encoders: ");
    Serial.print(encoder1());
    Serial.print("\t");
    Serial.println(encoder2());
  }
}


//Function to turn left
void leftTurn(){
  while (encoder1() > dis2tic(wheel1) && encoder2() < dis2tic(wheel2)){
    if (timesUp()){
      return;
    }
    transmission(ACCELERATION, 1); // acceleration
    transmission(SPEED1, 128 - cornerSpeed); // right wheel
    transmission(SPEED2, 128 + cornerSpeed); // left wheel
    // Serial monitor
    Serial.print("Encoders: ");
    Serial.print(encoder1());
    Serial.print("\t");
    Serial.println(encoder2());
  }
}


// Function that produces robot movements
void movement(FUN fun, double distance, float angle, bool avoid) { // angle in radians
  switch (fun) {                                              // check the first function input
    case fwd: {                                               // if forward is chosen
        wheel1 = distance - offsetS;                                      // set left wheel travel distance
        wheel2 = distance - offsetS;                                      // set right wheel travel distance
        forward(avoid);                                              // call function to move forward
        break;                                                  // exit the case
    }
    case bwd: {
        wheel1 = -(distance - offsetS);                                     // negative since moving in reverse
        wheel2 = -(distance - offsetS);
        backward(avoid);                                             // move backward
        break;
    }
    case rtn: {
        wheel1 = d1 * angle - offsetC;                                    // set left wheel travel distance based on angle
        wheel2 = -(d2 * angle - offsetC);                                   // set right wheel travel distance (negative since right wheel move in reverse)
        rightTurn();                                            // turn right standing still
        break;
    }
    case ltn: {
        wheel1 = -(d1 * angle - offsetC);                                   // negative since left wheel move in reverse
        wheel2 = d2 * angle - offsetC;
        leftTurn();                                             // turn left standing still
        break;
    }
  }

  // Post processing
  stopMotor();                                                // stop motor
  Serial.print("After stopping: ");
  Serial.print(encoder1());
  Serial.print("\t");
  Serial.println(encoder2());
  resetEncoder();                                             // reset encoder to 0
  delay(50);
}


// Function to determine if 100s is up or not
bool timesUp(){
  unsigned long currentMillis = millis();
  if (currentMillis - startMillis >= stopTime){
    return true;
  } else{
    return false;
  }
}


// Function to find distance from ultrasonic sensor
bool collision(bool forward){
  float duration, distance;

  if (forward){
    // Send out signal 
    digitalWrite(trigPinFront, LOW);
    delayMicroseconds(5);
    digitalWrite(trigPinFront, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinFront, LOW);
  
    // Get signal
    duration = pulseIn(echoPinFront, HIGH, sonicTimeOut);
  } else{
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
  distance = duration * 0.01715;

  if (distance == 0){
    return false;
  } else if (distance <= minGap){
    return true;
  } else{
    return false;
  }
}


// Function that transmit information to MD25 board
void transmission(byte address, int value){
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
