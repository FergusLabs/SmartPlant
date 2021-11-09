/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "c:/Users/School/Documents/IoT/SmartPlant/smartWaterSystem/src/smartWaterSystem.ino"
/*
 * Project smartWaterSystem
 * Description: Self Monitoring Plant Watering System
 * Author: Casey Fergus 
 * Date: 11/9/21
 */


#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX_RK.h>
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT.h" 
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h" 
#include <Adafruit_BME280.h>
#include "Air_Quality_Sensor.h"
#include "JsonParserGeneratorRK.h"
#include <Wire.h>
#include "credentials.h"

void setup();
void loop();
void publishTimeCheck (void);
void moistureCheck (void);
void runPump (void);
float celToFar(float tempC);
float paToInHg(float pressPA);
void MQTT_connect();
#line 20 "c:/Users/School/Documents/IoT/SmartPlant/smartWaterSystem/src/smartWaterSystem.ino"
Adafruit_BME280 bme;
#define OLED_RESET D4
Adafruit_SSD1306 display(OLED_RESET);
TCPClient TheClient; 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY);

Adafruit_MQTT_Publish bmeFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/bmeFeed");
Adafruit_MQTT_Publish moistureFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/moistureFeed");
Adafruit_MQTT_Subscribe wateringFeed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/wateringFeed");

const int sensorPin = A0;
const int relayPin = D11;
String DateTime, TimeOnly;
int moistureRead;
bool waterRead;
long last;
struct enviroData {
  float tempF;
  float relHumid;
  float pressInHg;
};
enviroData roomData;
void bmePublish (enviroData roomData);
void bmeCheck (enviroData *checkData);
void oledShow (enviroData *checkData);

void setup() {
  Serial.begin(9600);
  pinMode(sensorPin, INPUT);
  pinMode(relayPin, OUTPUT);
  Time.zone(-7);
  Particle.syncTime();
  bme.begin();

  mqtt.subscribe(&wateringFeed);
 
  display.begin(SSD1306_SWITCHCAPVCC, 0x3c);
  display.clearDisplay();
  display.display();
}


void loop() {
  MQTT_connect();
  DateTime = Time.timeStr();
  TimeOnly = DateTime.substring(11,16);

//Publish function calls most of the other functions
  publishTimeCheck();

// Ping MQTT Broker every 2 minutes to keep connection alive
  if ((millis()-last)>120000) {
      Serial.printf("Pinging MQTT \n");
      if(! mqtt.ping()) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
  }

//This subloop waits for incoming subscription packets
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(1000))) {
    if (subscription == &wateringFeed) {
      waterRead = atof((char *)wateringFeed.lastread);
        if (waterRead) {
          Serial.printf("Watered Plant\n");
          runPump();
        }
    }
  }
}

void publishTimeCheck (void) {
  static int lastUpdate;
  if((millis()-lastUpdate)>30000){
    moistureCheck();
    bmePublish(roomData);
    lastUpdate = millis();
  
  }
}

void bmePublish (enviroData roomData) {
  bmeCheck(&roomData);
  oledShow(&roomData);
  JsonWriterStatic<256> jw; { 
    JsonWriterAutoObject obj(&jw);
    jw.insertKeyValue("TempF", roomData.tempF);
    jw.insertKeyValue("Humidity", roomData.relHumid);
    jw.insertKeyValue("Pressure", roomData.pressInHg);
    Serial.printf("Published data to Adafruit Cloud\n");
  }
  if(mqtt.Update()) {
    bmeFeed.publish(jw.getBuffer());
  }    
}

void bmeCheck (enviroData *checkData) {
  float tempC, pressPA;
  tempC = bme.readTemperature();
  pressPA = bme.readPressure();
  checkData->relHumid = bme.readHumidity();
  checkData->tempF = celToFar(tempC);
  checkData->pressInHg = paToInHg(pressPA);
}

void moistureCheck (void) {
  moistureRead = analogRead(sensorPin);
  if(mqtt.Update()) {
      moistureFeed.publish(moistureRead);
  }
}

void runPump (void) {
  digitalWrite(relayPin, HIGH);
  delay(1000);
  digitalWrite(relayPin, LOW);
}

void oledShow (enviroData *checkData) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.printf("Moisture Read = %i,\nRoom TempF=%f\nRoom Humidity=%f\n@time = %s",moistureRead,checkData->tempF,checkData->relHumid,TimeOnly.c_str());
  display.display();
}

float celToFar(float tempC) {
  return  map(tempC,0.0,100.0,32.0,212.0);
}

float paToInHg(float pressPA) { 
   return (pressPA*0.00029530);   
}

// Function to connect and reconnect as necessary to the MQTT server.
void MQTT_connect() {
  int8_t ret;
 
  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }
 
  Serial.print("Connecting to MQTT... ");
 
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.printf("%s\n",(char *)mqtt.connectErrorString(ret));
       Serial.printf("Retrying MQTT connection in 5 seconds..\n");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
  }
  Serial.printf("MQTT Connected!\n");
}
