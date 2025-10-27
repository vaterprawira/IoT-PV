#include <WiFi.h>
extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#include <Preferences.h>

Preferences preferences;

#define WIFI_SSID "krpl"
#define WIFI_PASSWORD "Pr0j3ctLEAP!"

// #define WIFI_SSID "infinix"
// #define WIFI_PASSWORD "qwertyuiop"

#define MQTT_HOST IPAddress(203, 189, 123, 207)  //MQTT BROKER IP ADDRESS
/*for example:
#define MQTT_HOST IPAddress(192, 168, 1, 106)*/
#define MQTT_PORT 1883
#define BROKER_USER ""
#define BROKER_PASS ""

#define ACS_TOPIC_PV "krpl/tambakrejo/plts/i_panel_surya"
#define ACS_TOPIC_BATT "krpl/tambakrejo/plts/i_battery"

#define VOLT_TOPIC_PV "krpl/tambakrejo/plts/v_panel_surya"
#define VOLT_TOPIC_BATT "krpl/tambakrejo/plts/v_battery"

#define INVERTER_TOPIC_I "krpl/tambakrejo/plts/i_inverter"
#define INVERTER_TOPIC_V "krpl/tambakrejo/plts/v_inverter"
//#define INVERTER_TOPIC_W "krpl/tambakrejo/plts/daya/inverter"
#define BUZZER_TOPIC "krpl/tambakrejo/plts/buzzer"
#define OFFSET_PV "krpl/tambakrejo/plts/i_panel_surya_offset"
#define OFFSET_BATT "krpl/tambakrejo/plts/i_battery_offset"

#define ZCV_TOPIC_PV "krpl/tambakrejo/plts/zcv_pv"
#define ZCV_TOPIC_BATT "krpl/tambakrejo/plts/zcv_batt"

#define pin_buzzer 19

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

unsigned long previousMillis = 0;  // Stores last time a message was published
unsigned long previousACSMillis = 0;
const long interval = 10000;  // Interval at which to publish values

//pin sesnor tegangan
#define pinPV 32
#define pinBatt 33
int adcPV, adcBatt, voltagePV, voltageBatt;

const int LoopDelay = 2;  // Loop Delay in Seconds

float SensorRatioFactor = 0.04;  // ACS758-50B 60mV/A is the factor @ 5V
float ADCFactorVoltage = 3.3 / 4095.0;

float SensorVoltageBatt;
float SensorCalculatedCurrentBatt;
float SensorZeroVoltageBatt;

float SensorVoltagePV;
float SensorCalculatedCurrentPV;
float SensorZeroVoltagePV;

int buzzer_stat = 1;
float offset_batt = 1.5;
float offset_pv = 1.5;

//pin sensor arus
#define CurrentSensorBatt 34  //yg ada merah
#define CurrentSensorPV 35    //yg gk ada merah

//relay
#define relayPV 5
#define relayBatt 18

float INITIAL_PV;
float INITIAL_BATT;

#include <PZEM004Tv30.h>
#if defined(ESP32)
PZEM004Tv30 pzem(Serial2, 16, 17);
#else
PZEM004Tv30 pzem(Serial2);
#endif

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0);  // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);

  // Subscribe to topic "led" when it connects to the broker
  uint16_t packetIdSub = mqttClient.subscribe(BUZZER_TOPIC, 2);
  uint16_t packetIdSub1 = mqttClient.subscribe(OFFSET_PV, 2);
  uint16_t packetIdSub2 = mqttClient.subscribe(OFFSET_BATT, 2);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println();
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  // Do whatever you want when you receive a message
  // Save the message in a variable
  String receivedMessage;
  for (int i = 0; i < len; i++) {
    Serial.println((char)payload[i]);
    receivedMessage += (char)payload[i];
  }

  if (strcmp(topic, BUZZER_TOPIC) == 0) {
    if (receivedMessage == "true") {
      buzzer_stat = 1;
    }
    if (receivedMessage == "false") {
      buzzer_stat = 0;
    }
    preferences.putInt("buzzer_stat", buzzer_stat);
  }

  if (strcmp(topic, OFFSET_PV) == 0) {
    offset_pv = receivedMessage.toFloat();
    Serial.print("offset pv: ");
    Serial.println(receivedMessage);
  }

  if (strcmp(topic, OFFSET_BATT) == 0) {
    offset_batt = receivedMessage.toFloat();
    Serial.print("offset batt: ");
    Serial.println(receivedMessage);
  }


  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(topic);
  Serial.print("  qos: ");
  Serial.println(properties.qos);
  Serial.print("  dup: ");
  Serial.println(properties.dup);
  Serial.print("  retain: ");
  Serial.println(properties.retain);
  Serial.print("  len: ");
  Serial.println(len);
  Serial.print("  index: ");
  Serial.println(index);
  Serial.print("  total: ");
  Serial.println(total);
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup() {
  pinMode(relayPV, OUTPUT);
  pinMode(relayBatt, OUTPUT);
  pinMode(CurrentSensorBatt, INPUT);
  pinMode(CurrentSensorPV, INPUT);
  pinMode(pin_buzzer, OUTPUT);
  delay(250);

  Serial.begin(115200, SERIAL_8N1);  //115200
  while (!Serial)
    ;
  Serial.println("");

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(BROKER_USER, BROKER_PASS);

  connectToWifi();

  preferences.begin("memory", false);
  INITIAL_PV = preferences.getFloat("offset-pv", offset_pv);
  INITIAL_BATT = preferences.getFloat("offset-batt", offset_batt);
  Serial.print("INITIAL PV: ");
  Serial.println(INITIAL_PV);

  Serial.print("INITIAL BATT: ");
  Serial.println(INITIAL_BATT);

  buzzer_stat = preferences.getInt("buzzer_stat");
}

void loop() {

  if (millis() - previousACSMillis > 30000) {
    previousACSMillis = millis();
    ACS();
    PZEM();
    VoltDiv();

    preferences.putFloat("offset-pv", offset_pv);
    preferences.putFloat("offset-batt", offset_batt);
    // Serial.print("EEPROM PV: ");
    // Serial.println(preferences.getFloat("offset-pv", offset_pv));

    // Serial.print("EEPROM BATT: ");
    // Serial.println(preferences.getFloat("offset-batt", offset_batt));
  }
  //ACS();
}

void ACS() {
  // ACS758
  // Serial.println("ACS758: ");
  // Serial.print("ADC Raw Batt: ");
  // Serial.print(analogRead(CurrentSensorBatt));

  // Serial.print(" ADC Raw PV: ");
  // Serial.print(analogRead(CurrentSensorPV));

  SensorZeroVoltageBatt = ADCFactorVoltage * analogRead(CurrentSensorBatt);  // Read the voltage from sensor
  Serial.print("\tZCV Batt: ");
  Serial.print(SensorZeroVoltageBatt);

  SensorZeroVoltagePV = ADCFactorVoltage * analogRead(CurrentSensorPV);  // Read the voltage from sensor
  Serial.print("\tZCV PV: ");
  Serial.print(SensorZeroVoltagePV);

  offset_batt = preferences.getFloat("offset-batt", 1.51);
  offset_pv = preferences.getFloat("offset-pv", 1.51);

  SensorZeroVoltageBatt = offset_batt;  // yg ada merah2nya, out 15
  SensorZeroVoltagePV = offset_pv;      //out 4

  //Serial.print("SensorZeroVoltageBatt: ");
  //Serial.println(SensorZeroVoltageBatt);

  //Serial.print("SensorZeroVoltagePV: ");
  //Serial.println(SensorZeroVoltagePV);

  SensorVoltageBatt = ADCFactorVoltage * analogRead(CurrentSensorBatt);
  SensorVoltageBatt = SensorVoltageBatt - SensorZeroVoltageBatt;

  SensorVoltagePV = ADCFactorVoltage * analogRead(CurrentSensorPV);
  SensorVoltagePV = SensorVoltagePV - SensorZeroVoltagePV;

  SensorCalculatedCurrentBatt = SensorVoltageBatt / SensorRatioFactor;
  // Serial.print("\nI Batt: ");
  // Serial.print(SensorCalculatedCurrentBatt, 2);
  // Serial.print("A");

  SensorCalculatedCurrentPV = SensorVoltagePV / SensorRatioFactor;
  // Serial.print("\tI PV: ");
  // Serial.print(SensorCalculatedCurrentPV, 2);
  // Serial.println("A");

  // Serial.println("");

  uint16_t packetIdPub1 = mqttClient.publish(ACS_TOPIC_PV, 0, false, String(SensorCalculatedCurrentPV).c_str());
  uint16_t packetIdPub2 = mqttClient.publish(ACS_TOPIC_BATT, 0, false, String(SensorCalculatedCurrentBatt).c_str());

  uint16_t packetIdPub9 = mqttClient.publish(ZCV_TOPIC_PV, 0, false, String(SensorZeroVoltagePV).c_str());
  uint16_t packetIdPub10 = mqttClient.publish(ZCV_TOPIC_BATT, 0, false, String(SensorZeroVoltageBatt).c_str());

  //uint16_t packetIdPub3 = mqttClient.publish(ACS_TOPIC_PV, 0, false, "Haloo");
}

void PZEM() {
  // Serial.println("PZEM-004t");
  // Serial.print("Custom Address:");
  // Serial.println(pzem.readAddress(), HEX);
  digitalWrite(pin_buzzer, LOW);

  // Read the data from the sensor
  float voltage = pzem.voltage();
  float current = pzem.current();
  float power = pzem.power();
  float energy = pzem.energy();
  float frequency = pzem.frequency();
  float pf = pzem.pf();

  // Check if the data is valid
  if (isnan(voltage)) {
    Serial.println("Error reading voltage");

    //Serial.println("Buzzer Stat: ");
    //Serial.print (buzzer_stat);

    if (buzzer_stat == 1) {
      digitalWrite(pin_buzzer, HIGH);
    } else {
      digitalWrite(pin_buzzer, LOW);
    }
  } else if (isnan(current)) {
    Serial.println("Error reading current");
  } else if (isnan(power)) {
    Serial.println("Error reading power");
  } else if (isnan(energy)) {
    Serial.println("Error reading energy");
  } else if (isnan(frequency)) {
    Serial.println("Error reading frequency");
  } else if (isnan(pf)) {
    Serial.println("Error reading power factor");
  } else {

    // Print the values to the Serial console
    // Serial.print("Voltage: ");
    // Serial.print(voltage);
    // Serial.println("V");
    // Serial.print("Current: ");
    // Serial.print(current);
    // Serial.println("A");
    // Serial.print("Power: ");
    // Serial.print(power);
    // Serial.println("W");
    // Serial.print("Energy: ");
    // Serial.print(energy, 3);
    // Serial.println("kWh");
    // Serial.print("Frequency: ");
    // Serial.print(frequency, 1);
    // Serial.println("Hz");
    // Serial.print("PF: ");
    // Serial.println(pf);
  }
  Serial.println();

  uint16_t packetIdPub4 = mqttClient.publish(INVERTER_TOPIC_I, 0, false, String(current).c_str());
  uint16_t packetIdPub5 = mqttClient.publish(INVERTER_TOPIC_V, 0, false, String(voltage).c_str());
  //uint16_t packetIdPub6 = mqttClient.publish(INVERTER_TOPIC_W, 0, false, String(energy).c_str());
}

void VoltDiv() {
  // Serial.println("Voltage Divider");
  digitalWrite(relayPV, LOW);
  digitalWrite(relayBatt, LOW);

  delay(100);  //kasih waktu bentar sebelum baca

  adcPV = analogRead(pinPV);                                                                                                                          
  adcBatt = analogRead(pinBatt);

  Serial.print("adc pv : ");
  Serial.print(adcPV);
  Serial.print("\t adc batt : ");
  Serial.print(adcBatt);

  voltagePV = map(adcPV, 0, 4095, 0, 72);
  voltageBatt = map(adcBatt, 0, 4095, 0, 30) + 3;
  Serial.print("\t voltage pv : ");
  Serial.print(voltagePV);
  // Serial.print("\t voltage batt : ");
  // Serial.println(voltageBatt);

  uint16_t packetIdPub7 = mqttClient.publish(VOLT_TOPIC_PV, 0, false, String(voltagePV).c_str());
  uint16_t packetIdPub8 = mqttClient.publish(VOLT_TOPIC_BATT, 0, false, String(voltageBatt).c_str());

  delay(1000);

  digitalWrite(relayPV, HIGH);
  digitalWrite(relayBatt, HIGH);

  delay(LoopDelay * 1000);
}