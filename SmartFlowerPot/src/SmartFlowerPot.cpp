/* 
 * Project Smart flower pot
 * Author: CKCooper
 * Date: 11-6-2023
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
#include "Adafruit_BME280.h"
#include "Adafruit_Sensor.h"
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "Grove_Air_quality_Sensor.h"
#include "credentials.h"


#define OLED_RESET D4 //OLED 
#define XPOS 0;
#define YPOS 1;
String DateTime, TimeOnly;
// constants and variables for everything
const int PUMPPIN=D19;
int dustPin=D8;
int duration;
int startTime;
int sampleTime_ms=30000;
int lowPulseOccupancy=0;
int plantPin=A1;
int currentQuality=-1;
unsigned int last, lastTime;
float tempC;
float pressPa;
float humidRH;
float tempF;
float inHg;
float moisture;
float subValue,pubValue;
float ratio=0;
float concentration=0; 
bool status;

// classes
AirQualitySensor sensor(A0);
TCPClient TheClient; 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY);
Adafruit_MQTT_Publish moistureFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/plantmoisture");
Adafruit_MQTT_Publish tempFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/roomtemp");
Adafruit_MQTT_Publish psFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pressure");
Adafruit_MQTT_Publish rhFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidity");
Adafruit_MQTT_Publish aqFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/air-quality");
Adafruit_MQTT_Publish dustFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/dust-particulates");
Adafruit_MQTT_Subscribe buttonFeed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/buttononoff"); 
Adafruit_SSD1306 display(OLED_RESET);//OLED display
Adafruit_BME280 bme;
//functions
void myAutoWater(int moisture, int PUMPPIN);

void MQTT_connect();
bool MQTT_ping();

SYSTEM_MODE(AUTOMATIC);

SYSTEM_THREAD(ENABLED);

// setup() runs once, when the device is first turned on
void setup() {//start serial monitor
Serial.begin(9600);
waitFor(Serial.isConnected, 15000);

WiFi.on();//start the wifi
WiFi.connect();
while(WiFi.connecting()) {
Serial.printf(".");
}
Serial.printf("\n\n");

status = bme.begin(0x76);//start BME sensor
  if(status==false){
        Serial.printf("BME at address 0x%02X failed to start", 0x76);
  }
pinMode(plantPin, INPUT);//all the pins
pinMode(dustPin, INPUT);
pinMode(PUMPPIN, OUTPUT);
digitalWrite (PUMPPIN, LOW);
startTime=millis();

Particle.syncTime();//time
Time.zone(-7);

display.begin(SSD1306_SWITCHCAPVCC, 0x3D);//start oled

if (sensor.init()) {
  Serial.printf("Sensor ready.");
}
else {
  Serial.printf("Sensor ERROR!");
}
mqtt.subscribe(&buttonFeed);
}

// loop() runs over and over again, as quickly as it can execute.
void loop() {
 MQTT_connect();
 MQTT_ping();

  tempC=bme.readTemperature();//BME, date and time, moisture, sensor etc
  pressPa=bme.readPressure();
  humidRH=bme.readHumidity();
  DateTime=Time.timeStr();
  moisture=analogRead(plantPin);
  currentQuality=sensor.slope();
  duration = pulseIn(dustPin, LOW);

  tempF=map(tempC,0.0,100.0,32.0,212.0);//BME maps
  inHg=map(pressPa,3386.0,108364.0,1.0,32.0); 

lowPulseOccupancy=lowPulseOccupancy+duration;

if((millis()-startTime)>sampleTime_ms){

  }
ratio=lowPulseOccupancy/(sampleTime_ms*10.0);
concentration=1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62;
Serial.printf("%0.2f LPO\n", lowPulseOccupancy);
Serial.printf("%0.2f ratio\n", ratio);
Serial.printf("%0.2f concentration\n", concentration);
startTime=millis();


if((millis()-lastTime > 60000)) {
    if(mqtt.Update()) {
      moistureFeed.publish(moisture);
      Serial.printf("Moisture %0.2f \n",moisture);
      tempFeed.publish(tempF);
      Serial.printf("Temp %0.2f \n",tempF); 
      psFeed.publish(inHg);
      Serial.printf("BP %0.2f \n",inHg);
      rhFeed.publish(humidRH);
      Serial.printf("Humidity %0.1 \n",humidRH);
      aqFeed.publish(currentQuality);
      Serial.printf("AQ %i \n",currentQuality);
      dustFeed.publish(concentration);
      Serial.printf("DP %0.2f \n",concentration);
      }
      lastTime = millis();
      } 
//OLED
display.setTextSize(1);
display.setTextColor(WHITE);
display.setCursor(0,0);
display.clearDisplay();
display.printf("TF %0.1f\n BP %0.1f\n HM %0.1f\n MS %0.1f\n AQ %i\n DP %0.1f\n", tempF, inHg, humidRH, moisture, currentQuality, concentration);
// display.setTextSize(1);
display.printf("Time %s\n",DateTime.c_str());
display.display();

myAutoWater(moisture, PUMPPIN);//water function call

Adafruit_MQTT_Subscribe *subscription;//adafruit subscription
while ((subscription = mqtt.readSubscription(10000))) {
   if (subscription == &buttonFeed) {
      subValue = atof((char *)buttonFeed.lastread);
      Serial.printf("Button Subscription %f \n", subValue);
  }
  if(subValue==1){//adafruit water plant button
      digitalWrite(PUMPPIN,HIGH);
      delay(500);
      digitalWrite(PUMPPIN,LOW);
  }
  else{
      digitalWrite(PUMPPIN,LOW);
  }
}
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;
 
  // Return if already connected.
  if (mqtt.connected()) {
    return;
  }
 
  Serial.print("Connecting to MQTT... ");
 
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.printf("Error Code %s\n",mqtt.connectErrorString(ret));
       Serial.printf("Retrying MQTT connection in 5 seconds...\n");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds and try again
  }
  Serial.printf("MQTT Connected!\n");
}

bool MQTT_ping() {
  static unsigned int last;
  bool pingStatus;

  if ((millis()-last)>120000) {//this lets adafruit know we're still here
      Serial.printf("Pinging MQTT \n");
      pingStatus = mqtt.ping();
      if(!pingStatus) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
  }
  return pingStatus;
}
void myAutoWater(int moisture, int PUMPPIN){// water the plant if it is dry, check every half hour function
static unsigned int longTime;

 if ((millis()-longTime) >180000){
  if (moisture>2000){
    digitalWrite(PUMPPIN, HIGH);
    delay(500);
    digitalWrite(PUMPPIN, LOW);
    } 
    longTime = millis();
  }

 }
