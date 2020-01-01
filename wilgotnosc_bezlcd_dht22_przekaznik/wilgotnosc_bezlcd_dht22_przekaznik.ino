#include "dht.h"

dht DHT22;
#define DHT22PIN 7


void setup() {
  // put your setup code here, to run once:

  pinMode(10, OUTPUT);
  digitalWrite(10, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  int chk = DHT22.read(DHT22PIN);         //sprawdzenie stanu sensora
 
  switch (chk)
  {
    case DHTLIB_OK: 
      if (DHT22.humidity > 72) {
        digitalWrite(10, HIGH);
      } else if (DHT22.humidity < 67) {
        digitalWrite(10, LOW);
      } else {
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
