#define trigPin 12
#define echoPin 11

long czas;

void setup() {
  // put your setup code here, to run once:
  Serial.begin (9600);
  pinMode(10, OUTPUT);
  pinMode(trigPin, OUTPUT); //Pin, do którego podłączymy trig jako wyjście
  pinMode(echoPin, INPUT); //a echo, jako wejście
  digitalWrite(10, LOW);
}

void loop() {
  
  // put your main code here, to run repeatedly:
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  czas = pulseIn(11, HIGH);
  Serial.print(czas);
  Serial.println(" ");
  buzuj(100, czas/300);
  //delay(10);
}
//130000 = 500
//300 = 1 

void buzuj(int czas, int czas_cisza) {
  digitalWrite(10, HIGH);
  delay(czas);
  digitalWrite(10, LOW);
  delay(czas_cisza);
}
