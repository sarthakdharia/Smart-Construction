#include <Arduino.h>
#if defined(ESP32)
#include <WiFi.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#endif
#include <Firebase_ESP_Client.h>

//Provide the token generation process info.
#include "addons/TokenHelper.h"
//Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)
Adafruit_BMP280 bmp; // I2C

// Insert your network credentials
#define WIFI_SSID "WiFi Name"
#define WIFI_PASSWORD "WiFi Password"

// Insert Firebase project API Key
#define API_KEY "API KEY"

// Insert RTDB URLefine the RTDB URL */
#define DATABASE_URL "Real Time Data Base URL" 

//Define Firebase Data object
FirebaseData fbdo;

FirebaseAuth auth;
FirebaseConfig config;

unsigned long sendDataPrevMillis = 0;
int count = 0;
bool signupOK = false;

int led = 13;                // the pin that the LED is atteched to
int sensor = 15;              // the pin that the sensor is atteched to
int state = LOW;             // by default, no motion detected
int val = 0; 

//new start

int measurePin = 0; //Connect dust sensor to Arduino A0 pin
int ledPower = 2;   //Connect 3 led driver pins of dust sensor to Arduino D2
  
int samplingTime = 280;
int deltaTime = 40;
int sleepTime = 9680;
  
float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;

void setup(){
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  /* Assign the api key (required) */
  config.api_key = API_KEY;

  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;

  /* Sign up */
  if (Firebase.signUp(&config, &auth, "", "")){
    Serial.println("ok");
    signupOK = true;
  }
  else{
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h
  
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  Serial.println(F("BMP280 test"));
  if (!bmp.begin(0x76)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  pinMode(led, OUTPUT);      // initalize LED as an output
  pinMode(sensor, INPUT);
}



void loop(){
  if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 15000 || sendDataPrevMillis == 0)){
    sendDataPrevMillis = millis();
    // readTemperature
    if (Firebase.RTDB.setFloat(&fbdo, "values/temperature", bmp.readTemperature())){
      Serial.println("Updated");
      Serial.println("Temperature Value: " + String(bmp.readTemperature()));
    }
    else {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }
    // readPressure
    if (Firebase.RTDB.setFloat(&fbdo, "values/pressure", bmp.readPressure())){
      Serial.println("Updated");
      Serial.println("Pressure Value: " + String(bmp.readPressure()));
    }
    else {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }
    // readAltitude
    if (Firebase.RTDB.setFloat(&fbdo, "values/altitude", bmp.readAltitude())){
      Serial.println("Updated");
      Serial.println("Altitude value: " + String(bmp.readAltitude()));
    }
    else {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }
    // Airquality
    if (Firebase.RTDB.setInt(&fbdo, "values/airquality", 51 + analogRead(A0))){
      Serial.println("Updated");
      Serial.println("Airquality value: " + String(analogRead(A0)));
    }
    else {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }
    digitalWrite(ledPower,LOW); // power on the LED
    delayMicroseconds(samplingTime);
    voMeasured = analogRead(measurePin); // read the dust value
    delayMicroseconds(deltaTime);
    digitalWrite(ledPower,HIGH); // turn the LED off
    delayMicroseconds(sleepTime);
    calcVoltage = voMeasured * (5.0 / 1024.0);
    dustDensity = 0.17 * calcVoltage - 0.1;
    if (Firebase.RTDB.setFloat(&fbdo, "values/dustDensity", dustDensity)){
      Serial.println("Updated");
      Serial.println("Dust Density value: " + String(dustDensity));
    }
    else {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }
    val = digitalRead(sensor);   // read sensor value
    if (val == HIGH) {           // check if the sensor is HIGH
      digitalWrite(led, HIGH);   // turn LED ON
      delay(100);                // delay 100 milliseconds 
      if (state == LOW) {
        // Serial.println("Motion detected!");
        if (Firebase.RTDB.setBool(&fbdo, "values/motion", true)){
          Serial.println("Updated");
          Serial.println("Airquality value: " + String(analogRead(A0)));
        }
        else {
          Serial.println("FAILED");
          Serial.println("REASON: " + fbdo.errorReason());
        }
        state = HIGH;       // update variable state to HIGH
      }
    } 
    else {
      digitalWrite(led, LOW); // turn LED OFF
      delay(200);             // delay 200 milliseconds   
      if (state == HIGH){
        // Serial.println("Motion stopped!");
        if (Firebase.RTDB.setBool(&fbdo, "values/motion", false)){
          Serial.println("Updated");
          Serial.println("Airquality value: " + String(analogRead(A0)));
        }
        else {
          Serial.println("FAILED");
          Serial.println("REASON: " + fbdo.errorReason());
        }
        state = LOW;       // update variable state to LOW
      }
    }
  }
}
