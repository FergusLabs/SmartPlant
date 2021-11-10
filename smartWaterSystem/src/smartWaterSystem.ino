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

Adafruit_BME280 bme;
#define OLED_RESET D4
Adafruit_SSD1306 display(OLED_RESET);
AirQualitySensor AQsensor(A0);
TCPClient TheClient; 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY);

Adafruit_MQTT_Publish bmeFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/bmeFeed");
Adafruit_MQTT_Publish moistureFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/moistureFeed");
Adafruit_MQTT_Publish AQFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/airQualityFeed");
Adafruit_MQTT_Publish dustFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/dustFeed");
Adafruit_MQTT_Subscribe wateringFeed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/wateringFeed");

String DateTime, TimeOnly;

const int relayPin = D11;
unsigned long last, startTime;

const int moistureSensorPin = A5;
int moistureRead;
bool waterRead;

const int dustSensorPin = A4;
unsigned long sampletime_ms = 30000;

int airQuality;
float dustConcentration = 0;

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
  
  mqtt.subscribe(&wateringFeed);

  pinMode(moistureSensorPin, INPUT);
  pinMode(relayPin, OUTPUT);
  pinMode(dustSensorPin, INPUT);

  Time.zone(-7);
  Particle.syncTime();
  
  bme.begin();
 
  display.begin(SSD1306_SWITCHCAPVCC, 0x3c);
  display.clearDisplay();
  display.display();

  startTime = millis();
}

void loop() {
  MQTT_connect();
  DateTime = Time.timeStr();
  TimeOnly = DateTime.substring(11,16);

//Publish function calls most of the other functions
  publishTimeCheck();
//dustCheck runs on its own timeline
  dustCheck();

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
          runPump();
        }
    }
  }
}

void publishTimeCheck (void) {
  static int lastUpdate;
  if((millis()-lastUpdate)>60000){
    AQcheck();
    moistureCheck();
    dustPublish();
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
  moistureRead = analogRead(moistureSensorPin);
  if(mqtt.Update()) {
    moistureFeed.publish(moistureRead);
  }
  if ((moistureRead)>3000) {
    runPump();
  }
}

void AQcheck (void) {
  airQuality = AQsensor.slope();
    if(mqtt.Update()) {
      AQFeed.publish(AQsensor.getValue());
      if (airQuality == AirQualitySensor::FORCE_SIGNAL) {
        AQFeed.publish("High pollution! Force signal active.");
      } else if (airQuality == AirQualitySensor::HIGH_POLLUTION) {
        AQFeed.publish("High pollution!");
      } else if (airQuality == AirQualitySensor::LOW_POLLUTION) {
        AQFeed.publish("Low pollution!");
      } else if (airQuality == AirQualitySensor::FRESH_AIR) {
        AQFeed.publish("Fresh air.");
        } 
    }
}

void dustCheck(void) {
  unsigned long duration, lastCheck;
  unsigned long lowpulseoccupancy = 0;
  unsigned long sampletime_ms = 30000;
  float ratio = 0;
  if ((millis()-lastCheck)>30001) {
    duration = pulseIn(dustSensorPin, LOW);
    lowpulseoccupancy = lowpulseoccupancy+duration;
      if ((millis()-startTime) > sampletime_ms) {
        ratio = lowpulseoccupancy/(sampletime_ms*10.0);  // Integer percentage 0=>100
        dustConcentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // using spec sheet curve
        lowpulseoccupancy = 0;
        startTime = millis();
      }
    lastCheck = millis();
  }
}

void dustPublish (void) {
  if(mqtt.Update()) {
    dustFeed.publish(dustConcentration);
  }
}

void runPump (void) {
  digitalWrite(relayPin, HIGH);
  delay(1000);
  digitalWrite(relayPin, LOW);
  Serial.printf("Watered Plant\n");
}

void oledShow (enviroData *checkData) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.printf("Moisture Read = %i,\nRoom TempF = %2.2f\nHumidity = %2.2f\nAir Quality =%i\n,Dust Concentration =%4.3f\n@time = %s",moistureRead,checkData->tempF,checkData->relHumid,airQuality,dustConcentration,TimeOnly.c_str());
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
