// Basic demo for magnetometer readings from Adafruit LIS3MDL

#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <MQTT.h>
#include <Arduino_JSON.h>
#include <MicrocontrollerID.h>
#include "time.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ElegantOTA.h>
#include "config.h"

#define AXIS y




//  Meter Size  |   Pulse/Gallon
//--------------+----------------
//   5/8"       |    231.24
//   3/4"       |    129.04
//   1"         |    60.32
//   1 1/2"     |    27.03
//   2"         |    14.92
//
// So for a 5/8" Neptune T-10, you will set this to 0.000864902 ( 2 / 231.24)
float ppg = 2.0 / 231.24;


//-------------------------------------------------
int previousReading = 0;
float previousVal = 0.0;
unsigned long previousMillis = millis();
long long totalPulses = 0;
int loopCount = 0;

int status = WL_IDLE_STATUS;

Adafruit_LIS3MDL lis3mdl;
WiFiClient net;
MQTTClient client;


String availabilityTopic = baseTopic;
String stateTopic = baseTopic;
char sn [41];

WebServer server(80);

unsigned long ota_progress_millis = 0;

void onOTAStart() {
  // Log when OTA has started
  Serial.println("OTA update started!");
  // <Add your own code here>
}

void onOTAProgress(size_t current, size_t final) {
  // Log every 1 second
  if (millis() - ota_progress_millis > 1000) {
    ota_progress_millis = millis();
    Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
  }
}

void onOTAEnd(bool success) {
  // Log when OTA has finished
  if (success) {
    Serial.println("OTA update finished successfully!");
  } else {
    Serial.println("There was an error during OTA update!");
  }
  // <Add your own code here>
}

void enableOTA() {
  server.on("/", []() {
    server.send(200, "text/plain", "ESP to MQTT");
  });


  ElegantOTA.begin(&server);    // Start ElegantOTA
  // ElegantOTA callbacks
  ElegantOTA.onStart(onOTAStart);
  ElegantOTA.onProgress(onOTAProgress);
  ElegantOTA.onEnd(onOTAEnd);
  ElegantOTA.setAutoReboot(true);


  server.begin();
  Serial.println("HTTP server started");
  Serial.print("OTA updates performed at http://"); Serial.print(WiFi.localIP()); Serial.println("/update");
}

void connect() {
  Serial.print("Connectingto WiFi...");

  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("Connected!");
  printWifiStatus();

  Serial.print("Connecting to MQTT server...");
  while (!client.connect(deviceName, username, password)) {
     delay(500);
     Serial.print(".");
  }
  Serial.println("Connected!");
  availabilityTopic.concat("/"); availabilityTopic.concat("status");
  stateTopic.concat("/"); stateTopic.concat(sn); stateTopic.concat("/state");
  autoConfig();
  enableOTA();
}

String getUniqueId(String topic) {
  String rVal = baseTopic;
  rVal.concat("_");
  rVal.concat(sn);
  rVal.concat("_");
  rVal.concat(topic);
  return rVal;
}

void autoConfig() {
  String discoveryTopic = String("homeassistant/sensor/"); discoveryTopic.concat(baseTopic); discoveryTopic.concat("_"); discoveryTopic.concat(sn); discoveryTopic.concat("/"); 
  
  JSONVar identifiers;
  String id = baseTopic; id.concat("_"); id.concat(sn);
  identifiers[0] = id;

  JSONVar device;
  device["name"] = "T10 Water Meter";
  device["identifiers"] = identifiers;
  device["sw_version"] = "2024.09.21";
  device["model"] = "T10";
  device["via_device"] = "esp32_to_mqtt";
  device["manufacturer"] = "Neptune";

  JSONVar consumptionPayLoad;
  consumptionPayLoad["name"] = "Water Consumption";
  consumptionPayLoad["unique_id"] = getUniqueId("consumption");
  consumptionPayLoad["state_topic"] = stateTopic;
  consumptionPayLoad["value_template"] = "{{ value_json.consumption }}";
  consumptionPayLoad["unit_of_measurement"] = "gal";
  consumptionPayLoad["icon"] = "mdi:gauge";
  consumptionPayLoad["force_update"] = true;
  consumptionPayLoad["device_class"] = "water";
  consumptionPayLoad["state_class"] = "total_increasing";
  consumptionPayLoad["availability_topic"] = availabilityTopic;
  consumptionPayLoad["device"] = device;
  consumptionPayLoad["platform"] = "mqtt";

  String consumptionTopic = discoveryTopic; consumptionTopic.concat("consumption/config");
  client.publish(consumptionTopic.c_str(), JSON.stringify(consumptionPayLoad).c_str());

  JSONVar flowRatePayLoad;
  flowRatePayLoad["name"] = "Water Flow Rate";
  flowRatePayLoad["unique_id"] = getUniqueId("flowrate");
  flowRatePayLoad["state_topic"] = stateTopic;
  flowRatePayLoad["value_template"] = "{{ value_json.flowrate }}";
  flowRatePayLoad["unit_of_measurement"] = "gal/min";
  flowRatePayLoad["icon"] = "mdi:gauge";
  flowRatePayLoad["force_update"] = true;
  flowRatePayLoad["device_class"] = "water";
  flowRatePayLoad["state_class"] = "measurement";
  flowRatePayLoad["availability_topic"] = availabilityTopic;
  flowRatePayLoad["device"] = device;   
  flowRatePayLoad["platform"] = "mqtt";

  String flowRateTopic = discoveryTopic; flowRateTopic.concat("flowrate/config");
  client.publish(flowRateTopic.c_str(), JSON.stringify(flowRatePayLoad).c_str());


}

void setup(void) {
  MicroID.getUniqueIDString(sn);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  if (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

    // attempt to connect to Wifi network:
  Serial.print("Attempting to connect to SSID: ");
  Serial.println(ssid);

 

  // Try to initialize!
  if (! lis3mdl.begin_I2C(0x1e, &Wire)) {          // hardware I2C mode, can pass in address & alt Wire
  //if (! lis3mdl.begin_SPI(LIS3MDL_CS)) {  // hardware SPI mode
  //if (! lis3mdl.begin_SPI(LIS3MDL_CS, LIS3MDL_CLK, LIS3MDL_MISO, LIS3MDL_MOSI)) { // soft SPI
    Serial.println("Failed to find LIS3MDL chip");
    while (1) { delay(10); }
  }
  Serial.println("LIS3MDL Found!");

  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  Serial.print("Performance mode set to: ");
  switch (lis3mdl.getPerformanceMode()) {
    case LIS3MDL_LOWPOWERMODE: Serial.println("Low"); break;
    case LIS3MDL_MEDIUMMODE: Serial.println("Medium"); break;
    case LIS3MDL_HIGHMODE: Serial.println("High"); break;
    case LIS3MDL_ULTRAHIGHMODE: Serial.println("Ultra-High"); break;
  }

  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  Serial.print("Operation mode set to: ");
  // Single shot mode will complete conversion and go into power down
  switch (lis3mdl.getOperationMode()) {
    case LIS3MDL_CONTINUOUSMODE: Serial.println("Continuous"); break;
    case LIS3MDL_SINGLEMODE: Serial.println("Single mode"); break;
    case LIS3MDL_POWERDOWNMODE: Serial.println("Power-down"); break;
  }

  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  // You can check the datarate by looking at the frequency of the DRDY pin
  Serial.print("Data rate set to: ");
  switch (lis3mdl.getDataRate()) {
    case LIS3MDL_DATARATE_0_625_HZ: Serial.println("0.625 Hz"); break;
    case LIS3MDL_DATARATE_1_25_HZ: Serial.println("1.25 Hz"); break;
    case LIS3MDL_DATARATE_2_5_HZ: Serial.println("2.5 Hz"); break;
    case LIS3MDL_DATARATE_5_HZ: Serial.println("5 Hz"); break;
    case LIS3MDL_DATARATE_10_HZ: Serial.println("10 Hz"); break;
    case LIS3MDL_DATARATE_20_HZ: Serial.println("20 Hz"); break;
    case LIS3MDL_DATARATE_40_HZ: Serial.println("40 Hz"); break;
    case LIS3MDL_DATARATE_80_HZ: Serial.println("80 Hz"); break;
    case LIS3MDL_DATARATE_155_HZ: Serial.println("155 Hz"); break;
    case LIS3MDL_DATARATE_300_HZ: Serial.println("300 Hz"); break;
    case LIS3MDL_DATARATE_560_HZ: Serial.println("560 Hz"); break;
    case LIS3MDL_DATARATE_1000_HZ: Serial.println("1000 Hz"); break;
  }
  
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  Serial.print("Range set to: ");
  switch (lis3mdl.getRange()) {
    case LIS3MDL_RANGE_4_GAUSS: Serial.println("+-4 gauss"); break;
    case LIS3MDL_RANGE_8_GAUSS: Serial.println("+-8 gauss"); break;
    case LIS3MDL_RANGE_12_GAUSS: Serial.println("+-12 gauss"); break;
    case LIS3MDL_RANGE_16_GAUSS: Serial.println("+-16 gauss"); break;
  }

  lis3mdl.setIntThreshold(500);
  lis3mdl.configInterrupt(false, false, true, // enable z axis
                          true, // polarity
                          false, // don't latch
                          true); // enabled!

  client.begin(host, net);
  client.onMessage(messageReceived);

  connect();

}

void publish( long pulses ) {
  time_t now = millis();
  float ratePerMin = (totalPulses * ppg - previousVal) / (now - previousMillis) * 60 * 1000;

  JSONVar payLoad;
  payLoad["consumption"] = String(totalPulses * ppg,2);
  payLoad["flowrate"] = String(ratePerMin,2);
  payLoad["rssi"] = WiFi.RSSI();

  client.publish(stateTopic.c_str(), JSON.stringify(payLoad).c_str());
  Serial.print("Gallons: "); Serial.println(totalPulses * ppg);
  previousMillis = now;
  previousVal = totalPulses * ppg;


}

void loop() {
  server.handleClient();
  ElegantOTA.loop();

  loopCount = loopCount + 1;
  if ( loopCount % 25 == 0 ) {
    digitalWrite(LED_BUILTIN, HIGH);  }
  if ( loopCount > 50 ) {
    loopCount = 0;
    digitalWrite(LED_BUILTIN, LOW);
    client.publish(availabilityTopic.c_str(), "online");
    publish(totalPulses);
  }
  lis3mdl.read();      // get X Y and Z data at once

  /* Or....get a new sensor event, normalized to uTesla */
  sensors_event_t event; 
  lis3mdl.getEvent(&event);
  /* Display the results (magnetic field is measured in uTesla) */
  //Serial.print("\tX: "); Serial.print(event.magnetic.x);
  //Serial.print(" \tY: "); Serial.print(event.magnetic.y); 
  //Serial.print(" \tZ: "); Serial.print(event.magnetic.z); 
  //Serial.print(" uTesla ");

  // Zero cross
  if (!signbit(previousReading) != !signbit(event.magnetic.AXIS)) {
    totalPulses += 1;
    publish(totalPulses);
  
  }
  previousReading = event.magnetic.AXIS;

  delay(100); 
}


void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void messageReceived(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);

  // Note: Do not use the client in the callback to publish, subscribe or
  // unsubscribe as it may cause deadlocks when other things arrive while
  // sending and receiving acknowledgments. Instead, change a global variable,
  // or push to a queue and handle it in the loop after calling `client.loop()`.
}

