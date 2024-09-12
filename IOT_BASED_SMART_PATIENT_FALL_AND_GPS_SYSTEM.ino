// BLYNK APP CONTROL
#define BLYNK_TEMPLATE_ID "TMPL2tUf03wad"
#define BLYNK_TEMPLATE_NAME "IOT BASED SMART WEARABLE AUTHENTICATION HEALTHCARE"
#define BLYNK_AUTH_TOKEN "pC5YvSKd7O8JpiKN3xXZhsA788vKGzxm"

#include <PulseSensorPlayground.h>     // Includes the PulseSensorPlayground Library
#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <SimpleTimer.h>
#include "Wire.h"       
#include "I2Cdev.h"     
#include "MPU6050.h"    
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include "MAX30100_PulseOximeter.h"
int ds18b20=  5; // ESP32 pin GIOP17 connected to DS18B20 sensor's DATA pin
#include <LiquidCrystal_I2C.h> // library for I2C LCD  
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
String latitude, longitude;
#define rxPin 16
#define txPin 17
SoftwareSerial neogps(rxPin, txPin);

TinyGPSPlus gps;

//#define REPORTING_PERIOD_MS    1000
//
const int PulseWire = 0;       // 'S' Signal pin connected to A0
const int LED13 = 13;          // The on-board Arduino LED
int Threshold = 550;           // Determine which Signal to "count as a beat" and which to ignore
                               
PulseSensorPlayground pulseSensor;  // Creates an object


//float BPM, SpO2, bodytemperature;

OneWire oneWire(ds18b20);
DallasTemperature DS18B20(&oneWire);
LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display
//SimpleTimer timer;
//BlynkTimer timer;
float temp_C; // temperature in Celsius
float temp_F; // temperature in Fahrenheit

// Create a PulseOximeter object
//PulseOximeter pox;
//
//// Time at which the last beat occurred
//uint32_t tsLastReport = 0;
//
//// Callback routine is executed when a pulse is detected
//

//------------Indicators of the system -------
int greenled_Pin = 14;
int redled_Pin = 12;
int Activebuzzer_Pin = 27;
//-----------------------------------------

//Temperature sensor declarations
// GND - GND ESP32
// VCC + 3.3V ESP32
int DHTPin = 15;          // What digital pin we're connected to
#define Type DHT22     // DHT 11
DHT dht(DHTPin, Type);
float humidity;
float tempC;
float tempF;


//BLYNK APP AND INTENET CONNECTIONS
char auth[] = FIREBASE_AUTH;
char ssid[] = "Setsom-tech";
char pass[] = "0614444259";

MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int ledTest = 4;
struct MyData {
  byte X;
  byte Y;
};

MyData data;
//void onBeatDetected() {
//    Serial.println("Beat!");
//}

void setup() {
  Serial.begin(9600);
  DS18B20.begin();
  Blynk.begin(auth, ssid, pass); 
  pinMode(greenled_Pin, OUTPUT);
  pinMode(redled_Pin, OUTPUT);
  pinMode(Activebuzzer_Pin, OUTPUT);
  
  lcd.begin();
  lcd.backlight();
  lcd.clear();  
  dht.begin();

  lcd.setCursor(0,0);
  lcd.print("IOT PATIENT HEALTH");
  lcd.setCursor(0,1);
  lcd.print("CONTROL SYSTEM");
  delay(5000);
  lcd.clear();
  
  lcd.setCursor(0,0);
  lcd.print("TEMP=0");

  lcd.setCursor(9,0);
  lcd.print("BDY=0");

  lcd.setCursor(2,1);
  lcd.print("HEART=0");

      timer.setInterval(4000L, Body_Sensor);
//    timer.setInterval(10000L, DHTSensor2);
//    timer.setInterval(1000L, Heart_Sensor); 

    Serial.print("Initializing pulse oximeter..");

    // Initialize sensor
    if (!pox.begin()) {
        Serial.println("FAILED");
        for(;;);
    } else {
        Serial.println("SUCCESS");
    }

  // Configure sensor to use 7.6mA for LED drive
  pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);

    // Register a callback routine
  pox.setOnBeatDetectedCallback(onBeatDetected);
}
//--------------------

void Body_Sensor() { 
  DS18B20.requestTemperatures();       // send the command to get temperatures
  temp_C = DS18B20.getTempCByIndex(0);  // read temperature in °C
  temp_F = tempC * 9 / 5 + 32; // convert °C to °F
  Blynk.virtualWrite(V0, temp_F); //display the moisture percent.  
  Serial.print("Temperature: ");
  Serial.print(temp_C);    // print the temperature in °C
  Serial.print("°C");
  Serial.print("  ~  ");  // separator between °C and °F
  Serial.print(temp_F);    // print the temperature in °F
  
  Serial.println("°F");
  if (temp_C>=38){
    //Blynk.virtualWrite(V5, HIGH);
    digitalWrite(redled_Pin,HIGH);
    digitalWrite(Activebuzzer_Pin,HIGH);
    delay(900);
    //Blynk.virtualWrite(V5, LOW);
    digitalWrite(redled_Pin,LOW);
    digitalWrite(Activebuzzer_Pin,LOW);
    delay(900);    
  }
  else{
    //Blynk.virtualWrite(V5, LOW);
    digitalWrite(redled_Pin,LOW);
    digitalWrite(Activebuzzer_Pin,LOW);
  }
  //delay(500);
  lcd.setCursor(9,0);
  lcd.print("BODY=");
  lcd.print(temp_C);
  Blynk.virtualWrite(V0, temp_C); //display the moisture percent.
  
  } 

  //----------------------------------- Second Function ---------------------------
//---------------GPS DATA---------------------------------
void getGPS()
{

   while (neogps.available() > 0)
    if (gps.encode(neogps.read()))
//      displayInfo();

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }
//  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    latitude = String(gps.location.lat(), 6);
    longitude = String(gps.location.lng(), 6);
    
    Serial.print("LATITUDE: ");
    Serial.println(latitude);
    Serial.print("LONGITUDE: ");
    Serial.println(longitude);
    delay(1000);
  }
   String databasePathLATITUDE = "/LATITUDE";
   String databasePathLONGITUDE = "/LONGITUDE";
     if(Firebase.setString(firebaseData3, databasePathLATITUDE, latitude),
   (Firebase.setString(firebaseData3, databasePathLONGITUDE, longitude))){

    Serial.println("Data set in Firebase successfully.");
  } else {
    Serial.println("Error setting data in Firebase:");
    Serial.println(firebaseData3.errorReason());
  } //display the moisture percent.

    Serial.println("*********************************");
    Serial.println();

//-----------------------------------------------------------------------    
}

//----------------------------------- Third Function ---------------------------

void Fall_MPU6050_Sensor()
{
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  data.X = map(ax, -17000, 17000, 0, 255); // X axis data
  data.Y = map(ay, -17000, 17000, 0, 255);  // Y axis data
  //delay(500);
  Serial.print("Axis X = ");
  Serial.print(data.X);
  Serial.print("  ");
  Serial.print("Axis Y = ");
  Serial.println(data.Y);
   if (data.Y < 80) { //gesture : down 
    Serial.println("gesture 1");
    digitalWrite(ledTest, LOW);
    }
 if (data.Y > 145) {//gesture : up
  digitalWrite(ledTest, HIGH);
  Serial.println("gesture 2");
    }
 if (data.X > 155) {//gesture : left
  Serial.println("gesture 3");
    }
 if (data.X < 80) {//gesture : right
  Serial.println("gesture 4");
    }
 if (data.X > 100 && data.X < 170 && data.Y > 80 && data.Y < 130) { //gesture : little bit down
    Serial.println("gesture 5");
    }
    String databasePathFall = "/Fall_Detection";
      if(Firebase.setString(firebaseData4, databasePathFall, ledTest)){

    Serial.println("Data set in Firebase successfully.");
  } else {
    Serial.println("Error setting data in Firebase:");
    Serial.println(firebaseData4.errorReason());
  } //display the moisture percent.

    Serial.println("*********************************");
    Serial.println();

//-----------------------------------------------------------------------    
delay(1000);
}


//-------------
void loop() {
    // Read from the sensor
 digitalWrite(greenled_Pin,HIGH);
    timer.run();
    Blynk.run();   
    pox.update();
    // Grab the updated heart rate and SpO2 levels
    if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
      
        Serial.print("Heart rate:");
        Serial.print(pox.getHeartRate());       
        lcd.print(pox.getHeartRate());        
        Blynk.virtualWrite(V1,pox.getHeartRate());       
        tsLastReport = millis();
    }

}
