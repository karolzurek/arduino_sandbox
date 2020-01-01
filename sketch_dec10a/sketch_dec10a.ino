#include <LiquidCrystal.h>
#include "dht.h"

dht DHT22;
#define DHT22PIN 7

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

void setup() {
  // put your setup code here, to run once:
  pinMode(10, OUTPUT);
  digitalWrite(10, LOW);

  //Serial.begin(115200);                    //inicjalizacja monitora szeregowego
  //Serial.println("Program testowy DHT22"); 
  //Serial.println();

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print("Dzien Dobry");
  delay(3000);
  lcd.setCursor(0, 0);
  lcd.print("Wilg/Temp/Went");
}

void loop() {
  // put your main code here, to run repeatedly:

  int chk = DHT22.read(DHT22PIN);         //sprawdzenie stanu sensora, a następnie wyświetlenie komunikatu na monitorze szeregowym
 
  switch (chk)
  {
    case DHTLIB_OK: 
      //Serial.print("Wilgotnosc (%): ");              //wyświetlenie wartości wilgotności
      //Serial.print((float)DHT22.humidity, 2);
      //Serial.print(" ");
      //Serial.print("Temperatura (C): ");           //wyświetlenie temperatury
      //Serial.println((float)DHT22.temperature, 2);
      lcd.setCursor(0, 1);
      // print the number of seconds since reset:
      lcd.print((float)DHT22.humidity, 1);
      lcd.print("/");
      lcd.print((float)DHT22.temperature, 1);
      lcd.print("/");
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
  
  if (DHT22.humidity > 70) {
    digitalWrite(10, HIGH);
    lcd.print("ON ");
  } else {
    digitalWrite(10, LOW);
    lcd.print("OFF");
  }
  
  delay(1000);

}
