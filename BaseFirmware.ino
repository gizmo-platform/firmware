#include "config.h"
#include <WiFi.h>
#include <WiFiMulti.h>
#include <ArduinoJson.h>
#include <ArduinoMqttClient.h>
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

unsigned long UserWatchdogBitesAt;
unsigned long UserWatchdogResetsAt;

String fieldLocation;
String messageTopic;

void setup() {
  // Let the world know we're alive.  Very useful during debugging.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  status.Update();

  setupSerial();
  setupWifi();
  setupMQTT();
  setupGPIO();

  UserWatchdogBitesAt = millis() + MILLIS_WATCHDOG;
}

void setupSerial() {
  Serial1.setTX(0);
  Serial1.setRX(1);
  Serial1.begin(SERIAL_SPEED);

  Serial.begin(SERIAL_SPEED);

  Serial.println();
  Serial.println();
  Serial.println("Connecting to WiFi...");
}

void setupWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PSK);

  // wait for WiFi connection
  while ((WiFi.status() != WL_CONNECTED)) {
    Serial.write('w');
    delay(500);
  }
  status.SetWifiConnected(true);
  status.Update();
  Serial.println(" connected to WiFi");
}

void setupMQTT() {
  while (!mqttClient.connect(BRI_PUBLIC_MQTT_BROKER, 1883)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    delay(500);
  }

  mqttClient.subscribe(GAMEPAD_TOPIC);
  mqttClient.subscribe(LOCATION_TOPIC);
}

void setupGPIO() {
  pinMode(PWR_BOARD, INPUT);
  pinMode(PWR_PICO, INPUT);
  pinMode(PWR_GPIO, INPUT);
  pinMode(PWR_MAIN_A, INPUT);
  pinMode(PWR_MAIN_B, INPUT);

  pinMode(USER_RESET, OUTPUT);
}

void loop() {
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  delay(20);

  int messageSize = mqttClient.parseMessage();
  if (messageSize) {
    if (mqttClient.messageTopic() == GAMEPAD_TOPIC) {
      doParseControlData();
    } else if (mqttClient.messageTopic() == LOCATION_TOPIC) {
      doParseLocation();
    }
  }
}

void loop1() {
  readBoardStatus();
  //doUserWatchdog();
  if (Serial1.available() > 0) {
    doCommands();
  }
  status.Update();

}

void readBoardStatus() {
  boardState.VBat = analogRead(BOARD_VOLTAGE);
  boardState.WatchdogRemaining = UserWatchdogBitesAt > millis() ? UserWatchdogBitesAt - millis() : 0;
  boardState.WatchdogOK = UserWatchdogBitesAt > millis() ? true: false;
  boardState.RSSI = WiFi.RSSI();
  boardState.PwrBoard = digitalRead(PWR_BOARD);
  boardState.PwrPico  = digitalRead(PWR_PICO);
  boardState.PwrGPIO  = digitalRead(PWR_GPIO);
  boardState.PwrMainA = digitalRead(PWR_MAIN_A);
  boardState.PwrMainB = digitalRead(PWR_MAIN_B);
}

void doDataReport() {
  http.begin(REPORT_URL);
  http.addHeader("Content-Type", "Content-Type: application/json");
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
  int httpCode = http.POST(output);
}

void doParseLocation() {
  deserializeJson(fstateJSON, mqttClient);
  status.SetFieldNumber(fstateJSON["Field"]);
  switch (fstateJSON["Quadrant"].as<const char*>()[0]) {
  case 82:
    status.SetFieldQuadrant(BRI_QUAD_RED);
    break;
  case 66:
    status.SetFieldQuadrant(BRI_QUAD_BLUE);
    break;
  case 71:
    status.SetFieldQuadrant(BRI_QUAD_GREEN);
    break;
  case 89:
    status.SetFieldQuadrant(BRI_QUAD_YELLOW);
    break;
  }
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

  status.SetControlConnected(true);
}

void doUserWatchdog() {
  if (UserWatchdogBitesAt < millis()) {
    Serial.print("WARNING: User watchdog timed out! (");
    Serial.print(millis() - UserWatchdogBitesAt);
    Serial.println("ms)");
    digitalWrite(USER_RESET, HIGH);
    UserWatchdogResetsAt = millis() + 100;
    UserWatchdogBitesAt = millis() + 2000;
  }
  if (digitalRead(USER_RESET) && (UserWatchdogResetsAt < millis())) {
    digitalWrite(USER_RESET, LOW);
    Serial.println("WARNING: User watchdog cleared!");
  }
}

void doCommands() {
  cmd = Serial1.readStringUntil('\n');
  cmd.trim();
  Serial.print("Got Command: " + cmd + " (");

  if (cmd == "BT") {
    Serial1.println("OK");
  } else if (cmd == "BTB0") {
    Serial1.write(cstate.Button0);
  } else if (cmd == "BTB1") {
    Serial1.write(cstate.Button1);
  } else if (cmd == "BTB2") {
    Serial1.write(cstate.Button2);
  } else if (cmd == "BTB3") {
    Serial1.write(cstate.Button3);
  } else if (cmd == "BTB4") {
    Serial1.write(cstate.Button4);
  } else if (cmd == "BTB5") {
    Serial1.write(cstate.Button5);
  } else if (cmd == "BTB6") {
    Serial1.write(cstate.Button6);
  } else if (cmd == "BTB7") {
    Serial1.write(cstate.Button7);
  } else if (cmd == "BTB8") {
    Serial1.write(cstate.Button8);
  } else if (cmd == "BTB9") {
    Serial1.write(cstate.Button9);
  } else if (cmd == "BTA0") {
    Serial1.write(cstate.Axis0);
  } else if (cmd == "BTA1") {
    Serial1.write(cstate.Axis1);
    Serial.print(cstate.Axis1);
  } else if (cmd == "BTA2") {
    Serial1.write(cstate.Axis2);
  } else if (cmd == "BTA3") {
    Serial1.write(cstate.Axis3);
    Serial.print(cstate.Axis3);
  } else if (cmd == "BTA4") {
    Serial1.write(cstate.Axis4);
  } else if (cmd == "BTA5") {
    Serial1.write(cstate.Axis5);
  } else if (cmd == "BTA6") {
    Serial1.write(cstate.Axis6);
  } else if (cmd == "BTFDWD" ) {
    UserWatchdogBitesAt = millis() + 1000;
    Serial1.write("OK");
  } else {
    Serial1.println("EPARSE");
  }

  Serial.println(")");
}
