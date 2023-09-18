#include "config.h"
#include <Wire.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <ArduinoJson.h>
#include <ArduinoMqttClient.h>
#include <CooperativeMultitasking.h>
#include "indicators.h"
#include "secrets.h"

const byte BOARD_VOLTAGE = BRI_HW_ADC_BOARD_VOLTAGE;
const byte PWR_BOARD = BRI_HW_PWR_BOARD ;
const byte PWR_PICO = BRI_HW_PWR_PICO ;
const byte PWR_GPIO = BRI_HW_PWR_GPIO ;
const byte PWR_MAIN_A = BRI_HW_PWR_MAIN_A ;
const byte PWR_MAIN_B = BRI_HW_PWR_MAIN_B ;
const byte STATUS_NEOPIXELS_PIN = BRI_HW_STATUS_NEOPIXELS_PIN ;
const byte STATUS_NEOPIXELS_CNT = BRI_HW_STATUS_NEOPIXELS_CNT ;
const byte USER_RESET = BRI_HW_USER_RESET ;

const byte I2C_SDA = BRI_HW_I2C_SDA;
const byte I2C_SCL = BRI_HW_I2C_SCL;

const int MILLIS_WATCHDOG = 15000;
const char WIFI_SSID[] = BRI_PRIVATE_WIFI_SSID;
const char WIFI_PSK[] = BRI_PRIVATE_WIFI_PSK;

const int SERIAL_SPEED = BRI_HW_SERIAL_SPEED;

String GAMEPAD_TOPIC  = String("robot/") + BRI_PUBLIC_TEAM_NUMBER + "/gamepad";
String LOCATION_TOPIC = String("robot/") + BRI_PUBLIC_TEAM_NUMBER + "/location";
String STATS_TOPIC    = String("robot/") + BRI_PUBLIC_TEAM_NUMBER + "/stats";

StaticJsonDocument<384> cstateJSON;
StaticJsonDocument<64> fstateJSON;

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

CooperativeMultitasking tasks;
Continuation superviseMQTT;
Continuation superviseWiFi;
Continuation doStatsReport;
Continuation doFailsafeLED;
Continuation doMQTTPoll;
Guard networkIsAvailable;

StatusIndicators status(STATUS_NEOPIXELS_PIN, STATUS_NEOPIXELS_CNT);

struct CState {
  byte Axis0;
  byte Axis1;
  byte Axis2;
  byte Axis3;
  byte Axis4;
  byte Axis5;
  byte Axis6;
  byte Axis7;

  byte Button0;
  byte Button1;
  byte Button2;
  byte Button3;
  byte Button4;
  byte Button5;
  byte Button6;
  byte Button7;
  byte Button8;
  byte Button9;
};

struct BoardState {
  int VBat;
  int WatchdogRemaining;
  byte RSSI;

  bool WatchdogOK;
  bool PwrBoard;
  bool PwrPico;
  bool PwrGPIO;
  bool PwrMainA;
  bool PwrMainB;
};

String cmd;
CState cstate;
BoardState boardState;

String fieldLocation;
String messageTopic;

void setup() {
  pinMode(PWR_BOARD, INPUT);
  pinMode(PWR_PICO, INPUT);
  pinMode(PWR_GPIO, INPUT);
  pinMode(PWR_MAIN_A, INPUT);
  pinMode(PWR_MAIN_B, INPUT);
  pinMode(USER_RESET, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(SERIAL_SPEED);

  Serial.println();
  Serial.println();
  Serial.println("I'm Alive!");

  tasks.now(superviseWiFi);
  tasks.now(doFailsafeLED);
  tasks.now(doStatsReport);
  status.Update();

  tasks.ifForThen(networkIsAvailable, 5000, superviseMQTT);
}

void doMQTTPoll() {
  mqttClient.poll();
  tasks.after(20, doMQTTPoll);
}

void doFailsafeLED() {
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  tasks.after(50, doFailsafeLED);
}

bool networkIsAvailable() {
  return WiFi.localIP().isSet();
}

void superviseWiFi() {
  status.SetWifiConnected(WiFi.localIP().isSet());
  switch (WiFi.status()) {
  case WL_IDLE_STATUS:
    if (!WiFi.localIP().isSet()) {
      WiFi.mode(WIFI_STA);
      WiFi.begin(WIFI_SSID, WIFI_PSK);
      Serial.println("WiFi connected");
      Serial.printf("My IP is: ");
      Serial.println(WiFi.localIP());
    }
    break;
  case WL_CONNECTED:
    tasks.after(30000, superviseWiFi);
    return;
  case WL_NO_SHIELD:
    Serial.println("no wifi device, hardware fault!");
    return;
  case WL_CONNECT_FAILED:
    Serial.println("wifi connect failed");
    break;
  case WL_CONNECTION_LOST:
    Serial.println("wifi connection lost");
    break;
  case WL_DISCONNECTED:
    Serial.println("wifi disconnected");
    break;
  }
  tasks.after(10000, superviseWiFi);
}

void superviseMQTT() {
  // Don't bother if network is not connected
  if (!networkIsAvailable()) {
    tasks.after(5000, superviseMQTT);
  }

  // Connect if required
  if (!mqttClient.connected()) {
    status.SetControlConnected(false);
    Serial.println("MQTT is not connected, reconnecting...");
    if (!mqttClient.connect(BRI_PUBLIC_MQTT_BROKER, 1883)) {
      Serial.print("MQTT connection failed! Error code = ");
      Serial.println(mqttClient.connectError());
      tasks.after(2000, superviseMQTT);
      return;
    }
    Serial.println("MQTT Connected");

    // Like and Subscribe
    mqttClient.onMessage(doParseMQTTMessage);
    mqttClient.subscribe(GAMEPAD_TOPIC);
    mqttClient.subscribe(LOCATION_TOPIC);
    Serial.println("MQTT Subscribed");

    tasks.after(10000, superviseMQTT);
    tasks.now(doMQTTPoll);
  }
}

void loop() {
  tasks.run();
  status.Update();
}

void readBoardStatus() {
  boardState.VBat = analogRead(BOARD_VOLTAGE);
  boardState.WatchdogRemaining = 0;
  boardState.WatchdogOK = false;
  boardState.RSSI = WiFi.RSSI();
  boardState.PwrBoard = digitalRead(PWR_BOARD);
  boardState.PwrPico  = digitalRead(PWR_PICO);
  boardState.PwrGPIO  = digitalRead(PWR_GPIO);
  boardState.PwrMainA = digitalRead(PWR_MAIN_A);
  boardState.PwrMainB = digitalRead(PWR_MAIN_B);
}

void doStatsReport() {
  // Fetch updated information
  readBoardStatus();

  // Serialize State
  String output;
  StaticJsonDocument<164> posting;
  posting["VBat"] = boardState.VBat;
  posting["WatchdogRemaining"] = boardState.WatchdogRemaining;
  posting["RSSI"] = boardState.RSSI;
  posting["PwrBoard"] = boardState.PwrBoard;
  posting["PwrPico"] = boardState.PwrPico;
  posting["PwrGPIO"] = boardState.PwrGPIO;
  posting["PwrMainA"] = boardState.PwrMainA;
  posting["PwrMainB"] = boardState.PwrMainB;
  posting["WatchdogOK"] = boardState.WatchdogOK;
  serializeJson(posting, output);

  // Push to the FMS
  mqttClient.beginMessage(STATS_TOPIC);
  mqttClient.print(output);
  mqttClient.endMessage();

  // Again in a second
  tasks.after(1000, doStatsReport);
}

void doParseMQTTMessage(int messageSize) {
  if (mqttClient.messageTopic() == GAMEPAD_TOPIC) {
    doParseControlData();
  } else if (mqttClient.messageTopic() == LOCATION_TOPIC) {
    doParseLocation();
  }
}

void doParseLocation() {
  deserializeJson(fstateJSON, mqttClient);
  status.SetFieldNumber(fstateJSON["Field"]);

  if (fstateJSON["Quadrant"] == "RED") {
    status.SetFieldQuadrant(BRI_QUAD_RED);
  } else if (fstateJSON["Quadrant"] == "BLUE") {
    status.SetFieldQuadrant(BRI_QUAD_BLUE);
  } else if (fstateJSON["Quadrant"] == "GREEN") {
    status.SetFieldQuadrant(BRI_QUAD_GREEN);
  } else if (fstateJSON["Quadrant"] == "YELLOW") {
    status.SetFieldQuadrant(BRI_QUAD_YELLOW);
  }
  status.SetControlConnected(true);
  return;
}

void doParseControlData() {
  deserializeJson(cstateJSON, mqttClient);
  cstate.Button0 = cstateJSON["ButtonBack"];
  cstate.Button1 = cstateJSON["ButtonStart"];
  cstate.Button2 = cstateJSON["ButtonLeftStick"];
  cstate.Button3 = cstateJSON["ButtonRightStick"];
  cstate.Button4 = cstateJSON["ButtonX"];
  cstate.Button5 = cstateJSON["ButtonY"];
  cstate.Button6 = cstateJSON["ButtonA"];
  cstate.Button7 = cstateJSON["ButtonB"];
  cstate.Button8 = cstateJSON["ButtonLShoulder"];
  cstate.Button9 = cstateJSON["ButtonRShoulder"];

  cstate.Axis0 = cstateJSON["AxisLX"];
  cstate.Axis1 = cstateJSON["AxisLY"];
  cstate.Axis2 = cstateJSON["AxisRX"];
  cstate.Axis3 = cstateJSON["AxisRY"];
  cstate.Axis4 = cstateJSON["AxisLT"];
  cstate.Axis5 = cstateJSON["AxisRT"];
  cstate.Axis6 = cstateJSON["AxisDX"];
  cstate.Axis7 = cstateJSON["AxisDY"];
}

void setup1() {
  Wire1.setSDA(I2C_SDA);
  Wire1.setSCL(I2C_SCL);
  Wire1.begin(8);
  Wire1.onRequest(writeStateToI2C);
}

void writeStateToI2C() {
  byte toSend[18] = {
    cstate.Axis0,
    cstate.Axis1,
    cstate.Axis2,
    cstate.Axis3,
    cstate.Axis4,
    cstate.Axis5,
    cstate.Axis6,
    cstate.Axis7,
    cstate.Button0,
    cstate.Button1,
    cstate.Button2,
    cstate.Button3,
    cstate.Button4,
    cstate.Button5,
    cstate.Button6,
    cstate.Button7,
    cstate.Button8,
    cstate.Button9
  };
  Wire1.write(toSend, 18);
  return;
}
