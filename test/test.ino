#include<Servo.h>
#include<IRremote.h>

unsigned long startTime;
const int trig = 4;
const int echo = 5;
const int startPin = 9;
const unsigned long timeOut = 100 / 0.01715;
const float minGap = 7.00;
const int receiverPin = 12;
const int ledPin = 11;

unsigned long duration;
float distance;

void setup(){
  Serial.begin(9600);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(startPin, INPUT_PULLUP);
  irrecv.enableIRIn(); // Start the receiver
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  delay(1000);
}

void loop(){
  Serial.print("Collision: ");
  Serial.println(collision());
//  collision();
}

bool collision(){
  // Send out signal 
  digitalWrite(trig, LOW);
  delayMicroseconds(5);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  duration = pulseIn(echo, HIGH, timeOut);
  delay(10);

  // Calculate distance in cm
  distance = duration * 0.01715;
  Serial.println(distance);
  if (distance == 0){
    return false;
  } else if (distance < minGap){
    return true;
  } else{
    return false;
  }
}
