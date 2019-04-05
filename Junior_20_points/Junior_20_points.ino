/*
  Junior code
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
Servo servoG;
Servo servoR;
Servo servoL;
#define IDLE_R 90
#define ACT_R 0
#define IDLE_L 0
#define ACT_L 80
#define DROP 10
#define HOLD 45


//-------------- CONSTANTS (doesn't change) -----------------

enum FUN { fwd, bwd, rtn, ltn}; // definition of actions
const double circumference = 10.3 * M_PI; // circumference of wheel
const int straightSpeed = 30; // speed of wheel during a straight
const int cornerSpeed = 20;
const float d1 = 11; // radius of wheel arc
const float d2 = 11;
const int colourPin = 3; // Pin that control which side to go (1 = Purple, 0 = Yellow)
const int startPin = 2; // Pin that when pull will start the robot motion

//---------------- VARIABLES (will change) -------------------

float wheel1, wheel2; // offsets account for deceleration error
int motor1, motor2;
int offsetSpeed2;
unsigned long currentMillis;
unsigned long previousMillis = 0;
unsigned long startMillis;
float offsetS = (15.0 / 360.0) * float(circumference) + 2;
float offsetC = 2; //(9.0 / 360.0) * float(circumference) +

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

// Movement functions
void forward();
void backward();
void rightTurn();
void leftTurn();
void movement(FUN fun, double distance, float angle);

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
  delay(1000);

  // Assign Arduino pins
  pinMode(colourPin, INPUT_PULLUP);
  pinMode(startPin, INPUT_PULLUP);

  // Do nothing when start cord is not pulled
  //while (digitalRead(startPin) == LOW){/* do nothing */}

  // Record starting time
  startMillis = millis();
  
  // Decision between sides using a switch
  if (digitalRead(colourPin) == HIGH) {
    purple();
  } else if (digitalRead(colourPin) == LOW) {
    yellow();
  }
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
  Wire.write(3);                                             // set acceleration to acceleration register 10
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
  movement(bwd, 36, 0);
  movement(fwd, 20, 0);
  movement(ltn, 0, 90 * (M_PI / 180));
  movement(fwd, 18, 0);
  movement(bwd, 128, 0);
  movement(ltn, 0, 90 * (M_PI / 180));
  movement(fwd, 17, 0);
  movement(bwd, 90, 0);
  movement(rtn, 0, 90 * (M_PI / 180));
  movement(bwd, 25, 0);
  movement(rtn, 0, 90 * (M_PI / 180));
  movement(bwd, 95, 0);
  movement(fwd, 13, 0);
  movement(ltn, 0, 90 * (M_PI / 180));
  movement(bwd, 4, 0);
  servoL.write(ACT_L);
  delay(600);
  servoL.write(IDLE_L);
  movement(bwd, 57, 0);
  movement(ltn, 0, 90 * (M_PI / 180));
  movement(fwd, 10, 0);
}


// Yellow side function
void yellow() {
  movement(bwd, 36, 0);
  movement(fwd, 20, 0);
  movement(rtn, 0, 90 * (M_PI / 180));
  movement(fwd, 18, 0);
  movement(bwd, 128, 0);
  movement(rtn, 0, 90 * (M_PI / 180));
  movement(fwd, 17, 0);
  movement(bwd, 90, 0);
  movement(ltn, 0, 90 * (M_PI / 180));
  movement(bwd, 25, 0);
  movement(ltn, 0, 90 * (M_PI / 180));
  movement(bwd, 95, 0);
  movement(fwd, 13, 0);
  movement(rtn, 0, 90 * (M_PI / 180));
  movement(bwd, 5, 0);
  servoR.write(ACT_R);
  delay(600);
  servoR.write(IDLE_R);
  movement(bwd, 60, 0);
  movement(rtn, 0, 90 * (M_PI / 180));
  movement(fwd, 10, 0);
}


// Function that converts distance(cm) into number of encoder ticks
int dis2tic(double distance) {
  double rev = distance / circumference;                      // number of revolutions needed to turn
  int ticks = int(rev * 360);                                 // number of ticks encoder need to count
  return ticks;
}


// Function to go forward
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
    Wire.write(1);                                            // set acceleration register 5
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
    Wire.write(1);                                            // set acceleration register 5
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
}


// Function that produces robot movements
void movement(FUN fun, double distance, float angle) { // angle in radians
  switch (fun) {                                              // check the first function input
    case fwd: {                                               // if forward is chosen
        wheel1 = distance - offsetS;                                      // set left wheel travel distance
        wheel2 = distance - offsetS;                                      // set right wheel travel distance
        forward();                                              // call function to move forward
        break;                                                  // exit the case
    }
    case bwd: {
        wheel1 = -(distance - offsetS);                                     // negative since moving in reverse
        wheel2 = -(distance - offsetS);
        backward();                                             // move backward
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
  delay(200);
  resetEncoder();                                             // reset encoder to 0
  Serial.print("After reset: ");
  Serial.print(encoder1());
  Serial.print("\t");
  Serial.println(encoder2());
  delay(50);
}


// Function to determine if 100s is up or not
bool timesUp(){
  unsigned long currentMillis = millis();
  if (currentMillis - startMillis >= 100000){
    return true;
  } else{
    Serial.print("Time: ");
    Serial.println(currentMillis);
    return false;
  }
}


// loop function is not needed
void loop() {}
