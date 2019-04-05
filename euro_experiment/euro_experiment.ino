#include <IRremote.h>

int ir_pin = 9;
int motor_pin_1 = 10;
int motor_pin_2 = 11;
int LED_pin = 2;

IRrecv irrecv(ir_pin);
decode_results signals;


void setup()
{
  //Prepare the motor pins
  pinMode(motor_pin_1, OUTPUT);
  pinMode(motor_pin_2, OUTPUT);
  pinMode(LED_pin, OUTPUT);
  digitalWrite(motor_pin_1, LOW);
  digitalWrite(motor_pin_2, LOW);
  
  irrecv.enableIRIn(); // enable input from IR receiver

}


void loop() {
  /*
  if (irrecv.decode(&signals)) {
    Serial.println(signals.value, HEX); 
    irrecv.resume(); // get the next signal
   }
   */
  //wait for IR input
  if (irrecv.decode(&signals)) {
    digitalWrite(motor_pin_1, HIGH);
    digitalWrite(LED_pin, HIGH);
    delay(110000);
    digitalWrite(motor_pin_1, LOW);
    digitalWrite(LED_pin, LOW);
    irrecv.resume();
   }
}
