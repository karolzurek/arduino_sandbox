void setup() {
  // put your setup code here, to run once:
  pinMode(12, INPUT);
  pinMode(4, OUTPUT);
  Serial.begin (9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (digitalRead(12) == HIGH) {
   digitalWrite(4, HIGH);
  } else {
    digitalWrite(4, LOW);
  }
  Serial.println(digitalRead(12));
}
