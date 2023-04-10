// This program controls the LEDs and BME 280 via manual control

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BNO08x.h>
#include "yaw_pitch_roll.h"

yaw_pitch_roll imuSensor;

// Define LED pin properties
const int blue_LED_pin = 4;
const int red_LED_pin = 5;
const int freq = 5000; // PWM frequency
const int ledChannel = 0; // PWM channel
const int resolution = 8; // PWM bit resolution

// Define Sea level reference pressure (hPa)
#define SEALEVELPRESSURE_HPA (1013.25)

// Create BME object and check for identification
Adafruit_BME280 bme;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  imuSensor.imuSetup();
  pinMode(blue_LED_pin, OUTPUT);
  pinMode(red_LED_pin, OUTPUT);
  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(red_LED_pin, ledChannel);

  bool status;
  status = bme.begin(0x76);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
  while (1);
  }
  Serial.println("Enter A Number between 0 and 8191: ");
}

void loop() {
  // Read command from serial monitor
  if (Serial.available() >0){
    int command = Serial.parseInt();
    while (Serial.available()>0){
      char t;
      t = Serial.read();
    }
    Serial.print("Command Entered: ");
    Serial.println(command);
    if ((command < 0) || (command> 8191)){
      Serial.println("ENTER A NUMBER FROM 0 TO 8191");
    }
    else{
      if(bitRead(command,0)){
        Serial.print("Temperature = ");
        Serial.print(bme.readTemperature());
        Serial.println(" *C");
      }

      if(bitRead(command,1)){
        Serial.print("Humidity = ");
        Serial.print(bme.readHumidity());
        Serial.println(" %");
      }

      if(bitRead(command,2)){
        Serial.print("Pressure = ");
        Serial.print(bme.readPressure());
        Serial.println(" hPa");
      }

      if(bitRead(command,3)){
        Serial.print("Altitude = ");
        Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
        Serial.println(" m");
      }

      //Add extra bit for reading yaw, pitch, & roll from BNO085 sensor
      if(bitRead(command,4))
      {
        Serial.print("Yaw, Pitch, Roll: ");
        imuSensor.imuGetYawPitchRoll();
      }

      int blue_LED_bit = bitRead(command,5);
      int red_LED_value = command >> 6;
      digitalWrite(blue_LED_pin, blue_LED_bit);
      ledcWrite(ledChannel, red_LED_value);
      Serial.print("Blue LED Value: ");
      Serial.println(blue_LED_bit);
      Serial.print("Red LED Value: ");
      Serial.println(red_LED_value);
      Serial.println("");
      Serial.flush();
    }
  Serial.println("Enter A Number between 0 and 8191: ");
  }
}