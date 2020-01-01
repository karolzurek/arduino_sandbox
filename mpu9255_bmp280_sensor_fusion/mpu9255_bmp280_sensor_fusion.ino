/*
mpu9250 + bmp280
*/

#include <Wire.h>
#include <Adafruit_BMP280.h>
#include "MPU9250.h"
#include "SensorFusion.h" //SF
SF fusion;

float gx, gy, gz, ax, ay, az, mx, my, mz;
float pitch, roll, yaw;
float deltat;

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

Adafruit_BMP280 bmp; // I2C

void setup() {
  // serial to display data
  Serial.begin(115200);
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


  
  // display the data
  /*
  Serial.print(ax,6);
  Serial.print("\t");
  Serial.print(ay,6);
  Serial.print("\t");
  Serial.print(az,6);
  Serial.print("\t");
  Serial.print(gx,6);
  Serial.print("\t");
  Serial.print(gy,6);
  Serial.print("\t");
  Serial.print(gz,6);
  Serial.print("\t");
  Serial.print(mx,6);
  Serial.print("\t");
  Serial.print(my,6);
  Serial.print("\t");
  Serial.print(mz,6);
  Serial.print("\t");
  Serial.print(IMU.getTemperature_C(),6);
*/
//bmp280
/*Serial.print(F("Temperature = "));
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");
*/
  //Serial.print(F("Approx altitude = "));
  Serial.print("Altitude:\t");
  Serial.println(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
    
    
  Serial.println("");

  deltat = fusion.deltatUpdate(); //this have to be done before calling the fusion update
  //choose only one of these two:
  //fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, deltat);  //mahony is suggested if there isn't the mag and the mcu is slow
  fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);  //else use the magwick, it is slower but more accurate

  pitch = fusion.getPitch();
  roll = fusion.getRoll();    //you could also use getRollRadians() ecc
  yaw = fusion.getYaw();

  Serial.print("Pitch:\t"); Serial.println(pitch);
  Serial.print("Roll:\t"); Serial.println(roll);
  Serial.print("Yaw:\t"); Serial.println(yaw);
  Serial.println();
  
  
  delay(10);
}
