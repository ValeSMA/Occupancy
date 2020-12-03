
////////////////////////////////////////////////////
/***************************************************
                    LIBRARIES
***************************************************/
////////////////////////////////////////////////////

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Wire.h>
#include <ezTime.h>
#include <WiFi.h>
#include <PubSubClient.h>

#include "certs.h" // Header with certificates
#include <WiFiClientSecure.h> //Bucket
#include <ArduinoJson.h>

#include "header_n.h"
Eloquent::ML::Port::SVM classifier;

/***************************************************
Connecting to AWS IOT (via MQTT)
***************************************************/

const char* aws_iot_hostname = "a234v890jpbcco-ats.iot.us-west-2.amazonaws.com";
const char* aws_iot_sub_topic = "topic/occupancy";
const char* aws_iot_pub_topic = "another/topic/occupancy";
const char* aws_iot_python = "another/topic/occupancyPython";
const char* client_id = "MyIoT";

WiFiClientSecure client;
PubSubClient mqtt(client);

/***************************************************
BME sensor
***************************************************/

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;

/***************************************************
Button pins
***************************************************/

#define B1 (33)
#define B2 (32)
#define B3 (35)
#define B4 (34)
#define B5 (25)
int b1_empty = 0, b2_low = 0, b3_medium = 0, b4_high = 0, b5_testing = 0;

/***************************************************
Led pins
***************************************************/

#define p1_empty (12)
#define p2_low (14)
#define p3_medium (27)
#define p4_high (26)
#define p5_testing (13)

/***************************************************
Training time
***************************************************/

#define minutos 5 // Por el momento 5 segundos
unsigned long Tiempo = 1000*minutos; //El tiempo se mide en milisegundos, está separado en 1000*60*minutos
unsigned long TiempoAhora = 0;

float temp[minutos], hum[minutos], alt[minutos], pres[minutos], features[3];
String ttime[minutos];

/***************************************************
WiFi credentials
***************************************************/

#define ssid "INFINITUM3742"
#define password "iM6o0WrHJ2"

////////////////////////////////////////////////////
/***************************************************
                      SETUP
***************************************************/
////////////////////////////////////////////////////

void setup() {
  ButtonLedMode();
  Serial.begin(9600);
  Wire.begin();
  WiFiconfig();
  Serial.println("Mac Address: " + WiFi.macAddress());
  Timeconfig();
  BMEconfig();
  client.setCACert(ca_certificate);
  client.setCertificate(iot_certificate);
  client.setPrivateKey(iot_privatekey);

  // AWS IoT MQTT uses port 8883
  mqtt.setServer(aws_iot_hostname, 8883);
  mqtt.setCallback(sub_callback);
}

////////////////////////////////////////////////////
/***************************************************
                      LOOP
***************************************************/
////////////////////////////////////////////////////

void loop() {
  b1_empty = digitalRead(B1);
  b2_low = digitalRead(B2);
  b3_medium = digitalRead(B3);
  b4_high = digitalRead(B4);
  b5_testing = digitalRead(B5);

  while (!mqtt.connected()) {
    Serial.print("Now connecting to AWS IoT: ");
    if (mqtt.connect(client_id)) {
      Serial.println("connected!");
      mqtt.subscribe(aws_iot_sub_topic); //subscribe to the topic
    } else {
      Serial.print("failed with status code ");
      Serial.print(mqtt.state());
      Serial.println(" trying again in 5 seconds...");
      delay(5000);
    }
  }

  if(b1_empty){
    int ter = p1_empty;
    String eti = "Empty";
    //int eti = 1;
    ImprimirValores(b1_empty, ter, eti);
  }

  else if(b2_low){
    int ter = p2_low;
    String eti = "Low";
    //int eti = 2;
    ImprimirValores(b2_low, ter, eti);
  }

  else if(b3_medium){
    int ter = p3_medium;
    String eti = "Medium";
    //int eti = 3;
    ImprimirValores(b3_medium, ter, eti);
  }

  else if(b4_high){
    int ter = p4_high;
    String eti = "High";
    //int eti = 4;
    ImprimirValores(b4_high, ter, eti);
  }

  else if(b5_testing){
    digitalWrite(p5_testing, HIGH);
    classify();
    digitalWrite(p5_testing, LOW);
  }

  mqtt.loop();
    
  
}


////////////////////////////////////////////////////
/***************************************************
                     FUNCTIONS
***************************************************/
////////////////////////////////////////////////////

void ImprimirValores(int but, int led, String etiqueta){ 
  int i = 0; 
  TiempoAhora = millis();
  digitalWrite(but, HIGH);
  Timezone Mexico;
  Mexico.setLocation("America/Matamoros");
  while(millis()<= TiempoAhora + Tiempo){
      Serial.println("Button is pressed. Publishing MQTT message..."); 
      digitalWrite(led, HIGH);
      delay(500);
      ttime[i] = String(Mexico.dateTime());
      temp[i] = bme.readTemperature();
      hum[i] = bme.readHumidity();
      alt[i] = bme.readAltitude(SEALEVELPRESSURE_HPA);
      pres[i] = bme.readPressure() / 100.0F;
      digitalWrite(led, LOW);
      delay(500);
      i++;
    }
  publishMessage(etiqueta); 
}

void BMEconfig(){
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
}

void WiFiconfig() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  Serial.println("Connecting to Wifi");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.println(".");
    delay(500);
  }
  Serial.println("Connected to " + String(ssid) + " on address: " + WiFi.localIP().toString());
}

void sub_callback(const char* topic, byte* payload, unsigned int length) {
  Serial.print("Topic: ");
  Serial.println(topic);

  Serial.print("Message: ");
  for (int i = 0; i < length; i++)
    Serial.print((char) payload[i]);
  Serial.println();
}

void Timeconfig(){
  while (!waitForSync(3)) {
   Serial.println("Retrying timeSync");
  }
  Serial.println("Got timeSync");
}

void ButtonLedMode(){
  //Buttons
  pinMode(B1, INPUT);
  pinMode(B2, INPUT);
  pinMode(B3, INPUT);
  pinMode(B4, INPUT);
  pinMode(B5, INPUT);

  //Leds
  pinMode(p1_empty, OUTPUT);
  pinMode(p2_low, OUTPUT);
  pinMode(p3_medium, OUTPUT);
  pinMode(p4_high, OUTPUT);
  pinMode(p5_testing, OUTPUT);
  digitalWrite(p1_empty, LOW);
  digitalWrite(p2_low, LOW);
  digitalWrite(p3_medium, LOW);
  digitalWrite(p4_high, LOW);
  digitalWrite(p5_testing, LOW);
}

void publishMessage(String etiqueta)
{
  
  for(int i = 0; i < minutos; i++){
    StaticJsonDocument<200> doc;
    doc["Date"] = ttime[i];
    doc["MAC_Address"] = WiFi.macAddress();
    doc["Temperature"] = temp[i];
    doc["Pressure"] = pres[i];
    doc["Altitude"] = alt[i];
    doc["Humidity"] = hum[i];
    doc["Label"] = etiqueta;

    char jsonBuffer[512];
    serializeJson(doc, jsonBuffer);
    mqtt.publish(aws_iot_pub_topic, jsonBuffer);
  }; 
}

void MAC_AddressToPython()
{  
    StaticJsonDocument<200> doc;
    doc["MAC_Address"] = WiFi.macAddress();

    char jsonBuffer[512];
    serializeJson(doc, jsonBuffer);
    mqtt.publish(aws_iot_python, jsonBuffer);  
}
void classify() {
    float x_sample[] = {bme.readTemperature(), bme.readPressure() / 100.0F, bme.readHumidity()};

    Serial.print("Predicted class: ");
    Serial.println(classifier.predict(x_sample));
    delay(5000);
}
