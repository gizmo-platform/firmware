#include "config.h"
#include <Wire.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <ArduinoJson.h>
#include <ArduinoMqttClient.h>
#include <CooperativeMultitasking.h>
#include "indicators.h"
#include "secrets.h"

const char WIFI_SSID[] = BRI_PRIVATE_WIFI_SSID;
const char WIFI_PSK[] = BRI_PRIVATE_WIFI_PSK;

const int SERIAL_SPEED = BRI_HW_SERIAL_SPEED;

String GAMEPAD_TOPIC  = String("robot/") + BRI_PUBLIC_TEAM_NUMBER + "/gamepad";
String LOCATION_TOPIC = String("robot/") + BRI_PUBLIC_TEAM_NUMBER + "/location";
String STATS_TOPIC    = String("robot/") + BRI_PUBLIC_TEAM_NUMBER + "/stats";
String DEFAULT_BROKER = BRI_PUBLIC_MQTT_BROKER;

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
Continuation doMQTTWatchdog;
Guard networkIsAvailable;

StatusIndicators status(BRI_HW_STATUS_NEOPIXELS_PIN, BRI_HW_STATUS_NEOPIXELS_CNT);

struct CState {
  byte Axis0;
  byte Axis1;
  byte Axis2;
  byte Axis3;
  byte Axis4;
  byte Axis5;

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
  byte Button10;
  byte Button11;
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

bool practiceModeEnabled;
unsigned long nextControlPacketDueBy;

void setup() {
  // Initialize the cstate axis data so that robots don't run away on
  // boot.
  cstate.Axis0 = 127;
  cstate.Axis1 = 127;
  cstate.Axis2 = 127;
  cstate.Axis3 = 127;
  cstate.Axis4 = 127;
  cstate.Axis5 = 127;

  pinMode(BRI_HW_PWR_BOARD, INPUT);
  pinMode(BRI_HW_PWR_PICO, INPUT);
  pinMode(BRI_HW_PWR_GPIO, INPUT);
  pinMode(BRI_HW_PWR_MAIN_A, INPUT);
  pinMode(BRI_HW_PWR_MAIN_B, INPUT);
  pinMode(BRI_HW_USER_RESET, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BRI_HW_PRACTICE_MODE, INPUT);

  Serial.begin(BRI_HW_SERIAL_SPEED);

  Serial.println();
  Serial.println();
  Serial.println("I'm Alive!");

  practiceModeEnabled = digitalRead(BRI_HW_PRACTICE_MODE);

  tasks.now(superviseWiFi);
  tasks.now(doFailsafeLED);
  tasks.now(doStatsReport);
  tasks.now(doMQTTWatchdog);
  status.Update();

  tasks.ifForThen(networkIsAvailable, 5000, superviseMQTT);
}

void doMQTTWatchdog() {
  if (nextControlPacketDueBy < millis()) {
    status.SetControlConnected(false);
  }
  tasks.after(250, doMQTTWatchdog);
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
  if (practiceModeEnabled) {
    Serial.println("Practice mode enabled, Configuring softAP");
    superviseWiFiAP();
  } else {
    superviseWiFiSTA();
  }
}

void superviseWiFiAP() {
  String ssid = String("robot") + BRI_PUBLIC_TEAM_NUMBER;
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid);
  DEFAULT_BROKER = "192.168.42.2";
  status.SetWifiConnected(WiFi.localIP().isSet());
}

void superviseWiFiSTA() {
  status.SetWifiConnected(WiFi.status() == WL_CONNECTED);
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
    status.SetControlConnected(false);
  }

  // Connect if required
  if (nextControlPacketDueBy < millis()) {
    status.SetControlConnected(false);
    Serial.println("MQTT is not connected, reconnecting (broker: " + DEFAULT_BROKER + ")...");
    Serial.println(WiFi.softAPIP());
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

    tasks.now(doMQTTPoll);
  }
  tasks.after(5000, superviseMQTT);
}

void loop() {
  tasks.run();
  status.Update();
}

void readBoardStatus() {
  boardState.VBat = analogRead(BRI_HW_ADC_BOARD_VOLTAGE);
  boardState.WatchdogRemaining = 0;
  boardState.WatchdogOK = false;
  boardState.RSSI = WiFi.RSSI();
  boardState.PwrBoard = digitalRead(BRI_HW_PWR_BOARD);
  boardState.PwrPico  = digitalRead(BRI_HW_PWR_PICO);
  boardState.PwrGPIO  = digitalRead(BRI_HW_PWR_GPIO);
  boardState.PwrMainA = digitalRead(BRI_HW_PWR_MAIN_A);
  boardState.PwrMainB = digitalRead(BRI_HW_PWR_MAIN_B);

  // This converts from what the ADC reads to a voltage that's pretty
  // close to reality.  There's some slop in this calculation because
  // we aren't really doing everything totally correct with all the
  // analog biasing on the input.  This number was calculated by a
  // linear regression over sampled values.
  float voltage = 0.008833 * boardState.VBat + 0.3017;

  if (voltage>=8) {
    status.SetBatteryLevel(BRI_BAT_FULL);
  } else if (voltage<8 && voltage>=7.5) {
    status.SetBatteryLevel(BRI_BAT_GOOD);
  } else if (voltage<7.5 && voltage>=7) {
    status.SetBatteryLevel(BRI_BAT_PASS);
  } else if (voltage<7) {
    status.SetBatteryLevel(BRI_BAT_DEAD);
  }
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
  nextControlPacketDueBy = millis() + 250;
  status.SetControlConnected(true);

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
  } else if (fstateJSON["Quadrant"] == "PRACTICE") {
    status.SetFieldQuadrant(BRI_QUAD_PRACTICE);
  }
  return;
}

void doParseControlData() {
  deserializeJson(cstateJSON, mqttClient);
  cstate.Button0  = cstateJSON["ButtonX"];
  cstate.Button1  = cstateJSON["ButtonA"];
  cstate.Button2  = cstateJSON["ButtonB"];
  cstate.Button3  = cstateJSON["ButtonY"];
  cstate.Button4  = cstateJSON["ButtonLShoulder"];
  cstate.Button5  = cstateJSON["ButtonRShoulder"];
  cstate.Button6  = cstateJSON["ButtonLT"];
  cstate.Button7  = cstateJSON["ButtonRT"];
  cstate.Button8  = cstateJSON["ButtonBack"];
  cstate.Button9  = cstateJSON["ButtonStart"];
  cstate.Button10 = cstateJSON["ButtonLeftStick"];
  cstate.Button11 = cstateJSON["ButtonRightStick"];

  cstate.Axis0 = cstateJSON["AxisLX"];
  cstate.Axis1 = cstateJSON["AxisLY"];
  cstate.Axis2 = cstateJSON["AxisRX"];
  cstate.Axis3 = cstateJSON["AxisRY"];
  cstate.Axis4 = cstateJSON["AxisDX"];
  cstate.Axis5 = cstateJSON["AxisDY"];
}

void setup1() {
  Wire1.setSDA(BRI_HW_I2C_SDA);
  Wire1.setSCL(BRI_HW_I2C_SCL);
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
    cstate.Button0,
    cstate.Button1,
    cstate.Button2,
    cstate.Button3,
    cstate.Button4,
    cstate.Button5,
    cstate.Button6,
    cstate.Button7,
    cstate.Button8,
    cstate.Button9,
    cstate.Button10,
    cstate.Button11,
  };
  Wire1.write(toSend, 18);
  return;
}
