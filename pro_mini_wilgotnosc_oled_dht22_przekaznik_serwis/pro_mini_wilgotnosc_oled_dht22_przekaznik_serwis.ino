#include <DHT.h>

#include "U8g2lib.h"
#include "SPI.h"
#include "Wire.h"

#define DHTPIN 7
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);
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

  dht.begin(); //dht

  Serial.begin(115200);                    //inicjalizacja monitora szeregowego
  u8x8.begin();
  u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
  u8x8.drawString(0,0,"Wilg:   Temp:");
  u8x8.setFont(u8x8_font_8x13B_1x2_f );
}

void loop() {
  // put your main code here, to run repeatedly:
  float temp_hum_val[2] = {0};

  if (!dht.readTempAndHumidity(temp_hum_val)) {
    float h = 17.00 + temp_hum_val[0]; //offfset zeby zsynchronizowac z innym
    float t = temp_hum_val[1];
  
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
