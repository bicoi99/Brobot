#include <Servo.h>
Servo servo1;  // create myservo
Servo servo2;
Servo servo3;

void setup() {
//    servo1.attach(9); 
//    servo2.attach(10);
    servo3.attach(8);
    servo1.write(27);
    servo2.write(5);
    servo3.write(40);
    Serial.begin(9600);
}

void loop() {
//  servo1.write(27);
//  servo2.write(23);
//  delay(1000);
//  servo1.write(2);
//  servo2.write(5);
//  delay(1000);
}
