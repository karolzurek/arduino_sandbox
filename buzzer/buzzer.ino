void setup() {
  // put your setup code here, to run once:
  pinMode(10, OUTPUT);
  digitalWrite(10, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  for (int i = 1 ; i<1000; i=i+10 ) {
    buzuj(i, i);
  }
}

void buzuj(int czas, int czas_cisza) {
  digitalWrite(10, HIGH);
  delay(czas);
  digitalWrite(10, LOW);
  delay(czas_cisza);
}
