#include "dht.h"
#include "U8g2lib.h"
#include "SPI.h"
#include "Wire.h"

dht DHT22;
#define DHT22PIN 7
U8X8_SSD1306_128X32_UNIVISION_SW_I2C u8x8(3, 4);

void setup() {
  // put your setup code here, to run once:
//musza byc 3 na cewe i wystarczy jeden na dht i wyswietlczacz albo zasilic to z zasilacza
  pinMode(10, OUTPUT);//sterowanie przekaznikiem
  pinMode(2, OUTPUT);//zasilanie oled
  pinMode(9, OUTPUT);//zasilanie dht
  digitalWrite(10, LOW);
  digitalWrite(2, HIGH);
  digitalWrite(9, HIGH);
  //pinMode(13, OUTPUT);
  //digitalWrite(13, LOW);

  Serial.begin(115200);                    //inicjalizacja monitora szeregowego
  u8x8.begin();
  u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
  u8x8.drawString(0,0,"Wilg:   Temp:");
  u8x8.setFont(u8x8_font_8x13B_1x2_f );
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
      u8x8.setCursor(1, 2);
      u8x8.print((float)DHT22.humidity, 2);
      u8x8.setCursor(9, 2);
      u8x8.print((float)DHT22.temperature, 2);
      if (DHT22.humidity > 72) {
        digitalWrite(10, HIGH);
        //digitalWrite(13, HIGH);
      } else if (DHT22.humidity < 67) {
        digitalWrite(10, LOW);
        //digitalWrite(13, LOW);
      } else {
        /*digitalWrite(13, LOW);
        delay(100);
        digitalWrite(13, HIGH);
        delay(100);
        digitalWrite(13, LOW);
        delay(100);
        digitalWrite(13, HIGH);
        */
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
