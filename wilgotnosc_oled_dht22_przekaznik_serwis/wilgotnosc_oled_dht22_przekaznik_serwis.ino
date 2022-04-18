#include "DHT.h"
#include "U8g2lib.h"
#include "SPI.h"
#include "Wire.h"


#define DHTPIN 7
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);
U8X8_SSD1306_128X32_UNIVISION_SW_I2C u8x8(3, 2);

void setup() {
  // put your setup code here, to run once:
//musza byc 3 na cewe i wystarczy jeden na dht i wyswietlczacz albo zasilic to z zasilacza
  pinMode(10, OUTPUT);//sterowanie
  pinMode(11, OUTPUT);//zasilanie
  pinMode(12, OUTPUT);//zasilanie
  digitalWrite(10, LOW);
  digitalWrite(11, HIGH);
  digitalWrite(12, HIGH);
  //pinMode(13, OUTPUT);
  //digitalWrite(13, LOW);

  dht.begin(); //dht

  Serial.begin(115200);                    //inicjalizacja monitora szeregowego

  u8x8.begin(); //wyswietlacz
  u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
  u8x8.drawString(0,0,"Wilg:   Temp:");
  u8x8.setFont(u8x8_font_8x13B_1x2_f );
  
}

void loop() {
  // put your main code here, to run repeatedly:
  
  float h = dht.readHumidity();
  float t = dht.readTemperature();
 
  if (!isnan(h) && !isnan(t)) {
      Serial.print("Wilgotnosc (%): ");              //wyświetlenie wartości wilgotności
      Serial.print((float)h, 2);
      Serial.print(" ");
      Serial.print("Temperatura (C): ");           //wyświetlenie temperatury
      Serial.println((float)t, 2);
      u8x8.setCursor(1, 2);
      u8x8.print((float)h, 2);
      u8x8.setCursor(9, 2);
      u8x8.print((float)t, 2);
      
      if (h > 72) {
        digitalWrite(10, HIGH);
        //digitalWrite(13, HIGH);
      } else if (h < 67) {
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
  }
  
  delay(2000);

}
