void setup() {
  pinMode(10, OUTPUT);
  pinMode(2, INPUT);
  Serial.begin(9600);
}

void loop() {
  if(digitalRead(2) == HIGH){
    digitalWrite(10, HIGH);
  } else if (digitalRead(2) == LOW){
    digitalWrite(10, LOW);
  }
  Serial.println(digitalRead(2));
  delay(10);
}
