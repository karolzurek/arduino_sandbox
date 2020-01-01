#include "dht.h"

dht DHT22;
#define DHT22PIN 7


void setup() {
  // put your setup code here, to run once:

  pinMode(10, OUTPUT);
  digitalWrite(10, LOW);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  Serial.begin(115200);                    //inicjalizacja monitora szeregowego

}

void loop() {
  // put your main code here, to run repeatedly:
  
  int chk = DHT22.read(DHT22PIN);         //sprawdzenie stanu sensora
 
  switch (chk)
  {
    case DHTLIB_OK: 
      Serial.print("Wilgotnosc (%): ");              //wyświetlenie wartości wilgotności
      Serial.print((float)DHT22.humidity, 2);
      Serial.print(" ");
      Serial.print("Temperatura (C): ");           //wyświetlenie temperatury
      Serial.println((float)DHT22.temperature, 2);
      if (DHT22.humidity > 72) {
        digitalWrite(10, HIGH);
        digitalWrite(13, HIGH);
      } else if (DHT22.humidity < 67) {
        digitalWrite(10, LOW);
        digitalWrite(13, LOW);
      } else {
        digitalWrite(13, LOW);
        delay(300);
        digitalWrite(13, HIGH);
        delay(300);
        digitalWrite(13, LOW);
        delay(300);
        digitalWrite(13, HIGH);
      }
    break;
    case DHTLIB_ERROR_CHECKSUM: 
    //Serial.println("Błąd sumy kontrolnej"); 
    break;
    case DHTLIB_ERROR_TIMEOUT: 
    //Serial.println("Koniec czasu oczekiwania - brak odpowiedzi"); 
    break;
    default: 
    //Serial.println("Nieznany błąd"); 
    break;
  }
  
  delay(1000);

}
