#include <LiquidCrystal.h>
#include "dht.h"

dht DHT22;
#define DHT22PIN 7

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

void setup() {
  // put your setup code here, to run once:

  pinMode(10, OUTPUT);
  digitalWrite(10, LOW);
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.setCursor(0, 0);
  lcd.print("Wilg/Temp/Went");
}

void loop() {
  // put your main code here, to run repeatedly:
  
  int chk = DHT22.read(DHT22PIN);         //sprawdzenie stanu sensora
 
  switch (chk)
  {
    case DHTLIB_OK: 
      lcd.setCursor(0, 1);
      lcd.print((float)DHT22.humidity, 1);
      lcd.print("/");
      lcd.print((float)DHT22.temperature, 1);
      lcd.print("/");
      if (DHT22.humidity > 72) {
        digitalWrite(10, HIGH);
        lcd.print("ON ");
      } else if (DHT22.humidity < 68) {
        digitalWrite(10, LOW);
        lcd.print("OFF");
      } else {
        lcd.print("CHG");
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
