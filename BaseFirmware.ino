#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "indicators.h"
#include "secrets.h"

#define PWR_BOARD 7
#define PWR_PICO 8
#define PWR_GPIO 9
#define PWR_MAIN_A 10
#define PWR_MAIN_B 11

#define STATUS_NEOPIXELS_PIN 15
#define STATUS_NEOPIXELS_CNT 3
#define USER_RESET 22

HTTPClient http;
StaticJsonDocument<384> cstateJSON;
StaticJsonDocument<64> fstateJSON;

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

struct BState {
  int VBat;
  int UserWatchdogMillis;

  bool PwrBoard;
  bool PwrPico;
  bool PwrGPIO;
  bool PwrMainA;
  bool PwrMainB;
};

String cmd;
CState cstate;
BState bstate;

bool ctrlFieldIdentified;

unsigned long UserWatchdogBitesAt;
unsigned long UserWatchdogResetsAt;

String fieldLocation;

void setup() {
  // Let the world know we're alive.  Very useful during debugging.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  status.Update();

  setupSerial();
  setupWifi();
  setupHTTP();
  setupGPIO();

  UserWatchdogBitesAt = millis() + 15000;
}

void setupSerial() {
  Serial1.setTX(0);
  Serial1.setRX(1);
  Serial1.begin(9600);

  Serial.begin(9600);

  Serial.println();
  Serial.println();
  Serial.println("Connecting to WiFi...");
}

void setupWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(STASSID, STAPSK);

  // wait for WiFi connection
  while ((WiFi.status() != WL_CONNECTED)) {
    Serial.write('.');
    delay(500);
  }
  status.SetWifiConnected(true);
  status.Update();
  Serial.println(" connected to WiFi");
}

void setupHTTP() {
  // allow reuse (if server supports it)
  http.setReuse(true);
  http.setInsecure();
  http.setTimeout(100);
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
  ensureFieldLocation();
  doFetchControlData();
}

void loop1() {
  readPowerBus();
  //doUserWatchdog();
  if (Serial1.available() > 0) {
    doCommands();
  }
  status.Update();
}

void readPowerBus() {
  bstate.PwrBoard = digitalRead(PWR_BOARD);
  bstate.PwrPico  = digitalRead(PWR_PICO);
  bstate.PwrGPIO  = digitalRead(PWR_GPIO);
  bstate.PwrMainA = digitalRead(PWR_MAIN_A);
  bstate.PwrMainB = digitalRead(PWR_MAIN_B);
}

void ensureFieldLocation() {
  if (!ctrlFieldIdentified) {
    http.end();
    http.begin("http://192.168.16.10:8080/robot/data/location/1234");
    int c = http.GET();
    if (c == HTTP_CODE_OK) {
      deserializeJson(fstateJSON, http.getStream());
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
      ctrlFieldIdentified = true;
    }
    return;
  }
}

void doFetchControlData() {
  http.begin("http://192.168.16.10:8080/robot/data/gamepad/1234");
  int httpCode = http.GET();
  if (httpCode > 0) {
    // file found at server
    if (httpCode == HTTP_CODE_OK) {
      deserializeJson(cstateJSON, http.getStream());
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
  } else {
    // Something went wrong with the connection, try to reconnect
    //Serial.printf("HTTP ERROR: %d\r\n", httpCode);
    status.SetControlConnected(false);
    ctrlFieldIdentified = false;
    http.end();
    http.begin("http://192.168.16.10:8080/robot/data/gamepad/1234");
  }
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
