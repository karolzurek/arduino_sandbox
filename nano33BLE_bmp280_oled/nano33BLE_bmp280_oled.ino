/*
nano 33 ble + bmp280 + sensor fusion + kalman + oled + glosnik + przycisk??
audio vario
kompensacja
ahrs
asystent krazenia
auto widok asystenta
*/

#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <MPU9250.h>
#include <SensorFusion.h> //SF
#include <SimpleKalmanFilter.h>
#include "U8g2lib.h"
#include <SPI.h>

const int BMP_SCK = 13;
const int BMP_MISO = 12;
const int BMP_MOSI = 11;
const int BMP_CS = 10;
const long SERIAL_REFRESH_TIME = 1000;

float gx, gy, gz, ax, ay, az, mx, my, mz;
float pitch, roll, yaw;
float deltat;
int status;
long refresh_time;
float previous_estimated_altitude;
float vario;
float altitude, estimated_altitude;

SF fusion;
MPU9250 IMU(Wire,0x68); // an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
Adafruit_BMP280 bmp; // I2C
U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0, 5, 6);
SimpleKalmanFilter pressureKalmanFilter(0.15, 1, 0.01); //0.1 to 0.2 are acceptable
float toneFreq, toneFreqLowpass, pressure, lowpassFast, lowpassSlow ;

int ddsAcc;

void setup() {

  u8g2.begin();
  
  //pinMode(8, OUTPUT);
  Serial.begin(115200); // serial to display data
  while(!Serial) {}

  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }
  lowpassFast = lowpassSlow = pressure;
}

void loop() {
  // read the sensor mpu 9255
  IMU.readSensor();
  ax = IMU.getAccelX_mss();
  ay = IMU.getAccelY_mss();
  az = IMU.getAccelZ_mss();
  gx = IMU.getGyroX_rads();
  gy = IMU.getGyroY_rads();
  gz = IMU.getGyroZ_rads();
  mx = IMU.getMagX_uT();
  my = IMU.getMagY_uT();
  mz = IMU.getMagZ_uT();


  deltat = fusion.deltatUpdate(); //this have to be done before calling the fusion update
  //choose only one of these two:
  //fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, deltat);  //mahony is suggested if there isn't the mag and the mcu is slow
  fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);  //else use the magwick, it is slower but more accurate

  pitch = fusion.getPitch();
  roll = fusion.getRoll();    //you could also use getRollRadians() ecc
  yaw = fusion.getYaw();

  altitude = bmp.readAltitude(1030.25);
  estimated_altitude = pressureKalmanFilter.updateEstimate(altitude);
  vario = (estimated_altitude-previous_estimated_altitude)/deltat;
  previous_estimated_altitude = estimated_altitude;

  pressure = -estimated_altitude*10;
  lowpassFast = lowpassFast + (pressure - lowpassFast) * 0.1;
  lowpassSlow = lowpassSlow + (pressure - lowpassSlow) * 0.05;
  toneFreq = (lowpassSlow - lowpassFast) * 50;
  toneFreqLowpass = toneFreqLowpass + (toneFreq - toneFreqLowpass) * 0.1;
  toneFreq = constrain(toneFreqLowpass, -500, 500);
  ddsAcc += toneFreq * 100 + 2000;
  
  if (toneFreq < 0 || ddsAcc > 0) 
  {
    tone(8, toneFreq + 510);  
  }
  else
  {
    noTone(8);
  }
  
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
    u8g2.drawEllipse(64, 32, 8, 6, U8G2_DRAW_ALL);//samolocik
    
    u8g2.drawLine(0, ((pitch-90)/-2.8)+(((roll+45)/1.35)-33), 128, ((pitch-90)/-2.8)-((roll+45)/1.35)+33);        
  } while ( u8g2.nextPage() );
  
  if (millis() > refresh_time) {
    /**/
    Serial.print("Pitch:\t"); Serial.print(pitch);Serial.print("\t");
    Serial.print("Roll:\t"); Serial.print(roll);Serial.print("\t");
    Serial.print("Yaw:\t"); Serial.print(yaw);Serial.print("\t");
    /**/
    Serial.print(altitude,6);
    Serial.print("\t");
    Serial.print(estimated_altitude,6);
    Serial.print("\t");
    Serial.print(vario,3);
    Serial.println();    
    refresh_time=millis()+SERIAL_REFRESH_TIME;

  }

  delay(1);
}
