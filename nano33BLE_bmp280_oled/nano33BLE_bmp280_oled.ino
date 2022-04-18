/*
nano 33 ble + bmp180 + sensor fusion + kalman + oled + glosnik? + przycisk??
audio vario
kompensacja
ahrs
asystent krazenia
auto widok asystenta
averager noszenia 20s
*/

#include <Arduino.h>
#include <Wire.h>
#include <Arduino_LSM9DS1.h> //IMU
#include <SensorFusion.h> //SF
#include <SimpleKalmanFilter.h>
#include "U8g2lib.h" //display
#include <MS5611.h> //MS5611


/*
#include "wiring_private.h" //gps?
#include "HardwareSerial.h" //gps? 
#include <TinyGPS.h> //gps
*/


const long SERIAL_REFRESH_TIME = 1000; //gps
const long SERIAL_REFRESH_TIME_20 = 20; //gps

float gx, gy, gz, ax, ay, az, mx, my, mz;
float pitch, roll, yaw;
float deltat;
unsigned long refresh_time, refresh_time_20; //gps
float previous_estimated_altitude;
float vario, vario_g;
float altitude, estimated_altitude, base_pressure;

//long lat, lon;
//unsigned long fix_age, time_z , date_z;


//UART mySerial (digitalPinToPinName(6), digitalPinToPinName(5), NC, NC); //gpsimu
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, 5, 4); //wyswietlacz

//TinyGPS gps; //gps

MS5611 MS5611(0x77); //ms5611


SF fusion; //sensor fusion
SimpleKalmanFilter pressureKalmanFilter(0.7, 1, 0.1); //measurement uncertainty, estimation uncertainty, process variance

float toneFreq, toneFreqLowpass, pressure, lowpassFast, lowpassSlow;

int ddsAcc;

long satek; //gps
int year;//gps
byte month, day, hour, minute, second, hundredths; //gps


void setup() {

  Serial.begin(115200);

  //zasilanie pressure
  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);

  //glosnik
  pinMode(9, OUTPUT); 
  tone(9, 1510, 10);
  delay(100);
  noTone(9); 

  // start communication with IMU 
  if (!IMU.begin()) {
    Serial.println("IMU initialization unsuccessful");
    while(1);
  }

  //MS5611 init
  MS5611.init();

  //OLED init
  u8g2.begin();

 /* mySerial.begin(9600); // serial to display data //gps
  while(!mySerial) {}
 */ 
   
  MS5611.read();
  base_pressure = MS5611.getPressure()* 0.01;
  lowpassFast = lowpassSlow = pressure;
}




void loop() {
  
  // read the IMU sensor
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
  }
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
    gx = DEG_TO_RAD * gx; //deg to rad calulation
    gy = DEG_TO_RAD * gy;
    gz = DEG_TO_RAD * gz;
  }
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mx, my, mz);
  }

  deltat = fusion.deltatUpdate(); //this have to be done before calling the fusion update
  
  //fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, deltat);  //mahony is suggested if there isn't the mag and the mcu is slow
  fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);  //else use the magwick, it is slower but more accurate

  pitch = fusion.getPitch();
  roll = fusion.getRoll();
  yaw = fusion.getYaw();

  MS5611.read();
  pressure = MS5611.getPressure()* 0.01;
  altitude = 44330 * (1.0 - pow(pressure / base_pressure, 0.1903));
  
  estimated_altitude = pressureKalmanFilter.updateEstimate(altitude);

  vario = (estimated_altitude-previous_estimated_altitude)/deltat;
  previous_estimated_altitude = estimated_altitude;

// petla co 20 ms
  if (millis() > refresh_time_20) {
    refresh_time_20=millis()+SERIAL_REFRESH_TIME_20;
    pressure = -estimated_altitude*10;
    lowpassFast = lowpassFast + (pressure - lowpassFast) * 0.1;
    lowpassSlow = lowpassSlow + (pressure - lowpassSlow) * 0.05;
    toneFreq = (lowpassSlow - lowpassFast) * 50;
    toneFreqLowpass = toneFreqLowpass + (toneFreq - toneFreqLowpass) * 0.1;
    toneFreq = constrain(toneFreqLowpass, -500, 500);
    ddsAcc += toneFreq * 100 + 2000;
  
    if (toneFreq < 0 || ddsAcc > 0) 
    {
      tone(9, toneFreq + 510);  
    }
    else
    {
      noTone(9);
    }
  }
  // petla co 20 ms  


  u8g2.firstPage();

  do {
    u8g2.setFont(u8g2_font_p01type_tr);
    u8g2.drawStr(0, 4,"p: ");
    u8g2.setCursor(10, 4);
    u8g2.print((float)pitch, 1);
    u8g2.drawStr(45, 4,"r: ");
    u8g2.setCursor(55, 4);
    u8g2.print((float)roll, 1);
    u8g2.drawStr(90, 4,"y: ");
    u8g2.setCursor(100, 4);
    u8g2.print((float)yaw, 1);
    u8g2.drawStr(0, 12,"x: ");
    u8g2.setCursor(10, 12);
    u8g2.print((float)ax, 1);
    u8g2.drawStr(45, 12,"y: ");
    u8g2.setCursor(55, 12);
    u8g2.print((float)ay, 1);
    u8g2.drawStr(90, 12,"z: ");
    u8g2.setCursor(100, 12);
    u8g2.print((float)az, 1);
    u8g2.drawStr(0, 20,"x: ");
    u8g2.setCursor(10, 20);
    u8g2.print((float)mx, 1);
    u8g2.drawStr(45, 20,"y: ");
    u8g2.setCursor(55, 20);
    u8g2.print((float)my, 1);
    u8g2.drawStr(90, 20,"z: ");
    u8g2.setCursor(100, 20);
    u8g2.print((float)mz, 1);

    //VARIO
    vario_wsk(vario);
    //vario pasek
    pasek_vario(vario);
    //horyzont
    horyzont(pitch, roll);    
    

    //wysokosc
    u8g2.setFont(u8g2_font_p01type_tr);
    u8g2.drawStr(0, 60,"est: ");
    u8g2.setCursor(20, 60);
    u8g2.print((float)estimated_altitude, 1);
    u8g2.drawStr(60, 60,"wys: ");
    u8g2.setCursor(80, 60);
    u8g2.print((float)altitude, 1);



  } while ( u8g2.nextPage() );
  
  
  delay(1);
}

//pasek a'la wskazowka
void pasek_vario (float vario) {
      int szer=8;
      int poz0_x=120;
      int poz0_y=32;
      int vario_p = constrain(vario, -5, 5);
      //u8g2.drawBox(poz0_x, poz0_y, szer, -6.4*(vario_p));
      for (int i=0; i<szer; i++) {
        u8g2.drawLine(poz0_x+i, poz0_y, poz0_x+i, -6.2*(vario_p)+32);//
      }
      u8g2.drawLine(poz0_x-3, poz0_y, poz0_x+szer+3, poz0_y);//zero poz
}

//numeryczne info plus strzalki gora dol
void vario_wsk (float vario) {
    int poz0_x = 24;
    int poz0_y = 40;
    
    u8g2.setFont(u8g2_font_helvB12_tr);
    u8g2.setCursor(poz0_x, poz0_y);
    if (vario > 0) {
      u8g2.drawStr(poz0_x, poz0_y,"/\\");
    } else {
      u8g2.drawStr(poz0_x, poz0_y,"\\/");
    }
        
    if (vario < 0) { //zeby nie lataly wskazania na prawo i lewo
      u8g2.drawStr(poz0_y+18, poz0_y-2,"-");
    } else {
      u8g2.drawStr(poz0_y+16, poz0_y-2,"+");
    }
    u8g2.setCursor(poz0_y+30, poz0_y);
    u8g2.print(abs((float)vario), 1);

}

void horyzont(float pitch, float roll) {
    int poz0_x = 24;
    int poz0_y = 40;
    
    //u8g2.drawEllipse(64, 32, 8, 6, U8G2_DRAW_ALL);//samolocik 
    u8g2.drawLine(0, ((pitch-90)/-2.8)+(((roll+45)/1.35)-33), 128, ((pitch-90)/-2.8)-((roll+45)/1.35)+33);        
}
