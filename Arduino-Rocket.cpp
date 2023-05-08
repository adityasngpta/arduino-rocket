// The following code is designed to run on an Arduino.
// It was converted from .ino (Arduino IDE File) to .cpp (C++) for compatability with GitHub.

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>
#include <SFE_BMP180.h>
#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

const int chipSelect = 4;

String output;

float t_launch = 0;
float t = 0;

float starting_alt;
float altitude;
float last_altitude;

bool parachute_triggered = false;

bool rocket_launched = false;
float accel = 0;

const float alpha = 0.3;
float filter(float input, float prev_output, float alpha) {
  return (alpha * input) + ((1 - alpha) * prev_output);
}

Adafruit_MPU6050 mpu;
Adafruit_BMP085 bmp;
Servo parachute;
Servo camera;

void setup() {

  Serial.begin(9600);
  Serial.println("Setup");

  Serial.println("Wire");
  Wire.begin();
  
  // Servos
  Serial.println("Servos");
  parachute.attach(9);
  parachute.write(180);
  parachute_triggered = false;

  // MPU
  Serial.println("MPU"); 
  if(!mpu.begin()) {
    Serial.println("Error"); 
  }
  else {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  }

  
  // Altimeter
  Serial.println("Altimeter"); 
  if (!bmp.begin()) {
    Serial.println("Error"); 
  }
  else {
   starting_alt = bmp.readAltitude(1013.25); 
  }
  altitude = 0;
  

  // SD
  Serial.println("SD"); 
  SD.begin(chipSelect);
  
  Serial.println("Done"); 
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);  
  accel = abs(a.acceleration.x) + abs(a.acceleration.y) + abs(a.acceleration.z);
  //Serial.println(accel);
  pinMode(3, OUTPUT);
  digitalWrite(3, HIGH);

  if (accel > 14 || rocket_launched || t > 60) {
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
  
  if(parachute_triggered){
    digitalWrite(3, HIGH);
  }
  else {
    digitalWrite(3, LOW);
  }
  
  rocket_launched = true;
  
  last_altitude = altitude;
  altitude = bmp.readAltitude(1013.25) - starting_alt; // sea level pressure in hPa

  
  if ( (t_launch > 4) && (!parachute_triggered) ) {
    parachute.write(0);
    delay(1000);
    File dataFile = SD.open("data.csv", FILE_WRITE);
    dataFile.println("parachute_triggered");
    dataFile.close();
    parachute.write(0);
    parachute_triggered = true;
  } 

  File dataFile = SD.open("data.csv", FILE_WRITE);
  output = String(altitude) + String(",") + String(accel) + String(",") + String(t_launch);
  Serial.println(output);
  dataFile.println(output);
  dataFile.close();
  t_launch += 0.1;

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
  delay(100);
  t += 0.1;

  }
}

