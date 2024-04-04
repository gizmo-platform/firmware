#include "gizmo.h"

#include "Arduino.h"
#include "indicators.h"
#include <ArduinoJson.h>
#include <ArduinoMqttClient.h>
#include <WiFi.h>
#include <Wire.h>
#include "LEAmDNS.h"
#include "LittleFS.h"

Config cfg;
CfgState cfgState = CFG_INIT;

// Should really be const, but we don't know these until after
// initialization.
byte pinStatusPwrSupply;
byte pinStatusPwrBoard;
byte pinStatusPwrPico;
byte pinStatusPwrGPIO;
byte pinStatusPwrMainA;
byte pinStatusPwrMainB;

// This is the pin that can be toggled to force the User Processor
// into reset.
byte pinUserReset;

StatusIndicators status(15, 3);
WiFiClient network;
MqttClient mqtt(network);

CState cstate;
BoardState boardState;
NetState netState;

String fieldLocation;

JsonDocument cstateJSON;
JsonDocument fstateJSON;

unsigned long nextControlPacketDueBy;
unsigned long controlFrameAge;
unsigned long netStateConnectTimeout;
unsigned long nextStatusReportAt;

// function declarations for "private" functions.
bool loadConfig(String);
void loadConfigFromSerial();
void checkIfShouldConfig();

void netStateMachine();
void netStateLinkSearch();
void netStateWifiConnect();
void netStateConnectWait();
void netStateFMSDiscover();
void netStateMQTTConnect();
void netStateRun();

void mqttParseMessage(int);
void mqttParseLocation();
void mqttParseControl();

void wireRespond();

void statusUpdate();
void statusReport();

void ConfigureStatusIO(byte supply, byte board, byte pico, byte gpio, byte mainA, byte mainB) {
  pinStatusPwrSupply = supply;
  pinStatusPwrBoard = board;
  pinStatusPwrPico = pico;
  pinStatusPwrGPIO = gpio;
  pinStatusPwrMainA = mainA;
  pinStatusPwrMainB = mainB;
}

void ConfigureUserReset(byte rst) {
  pinUserReset = rst;
}

void GizmoSetup() {
  // Ensure that the cstate values for axis data start at a reasonable
  // zero point.  This prevents machines from running off into the
  // wild blue yonder.
  cstate.Axis0 = 127;
  cstate.Axis1 = 127;
  cstate.Axis2 = 127;
  cstate.Axis3 = 127;
  cstate.Axis4 = 127;
  cstate.Axis5 = 127;

  pinMode(pinStatusPwrBoard, INPUT);
  pinMode(pinStatusPwrPico, INPUT);
  pinMode(pinStatusPwrGPIO, INPUT);
  pinMode(pinStatusPwrMainA, INPUT);
  pinMode(pinStatusPwrMainB, INPUT);
  pinMode(pinUserReset, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);

  loadConfig("/gsscfg.json");

  WiFi.setHostname(cfg.hostname);
  WiFi.noLowPowerMode();
  WiFi.setTimeout(500);

  statusUpdate();
  status.Update();
  netState = NET_SEARCH;
}

// Tick runs all the functions that need to happen on every loop
// through.  Not every one of these functions will do something, but
// they do need to be ticked on each iteration.
void GizmoTick() {
  statusUpdate();
  status.Update();
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

  if (cfg.loaded) {
    checkIfShouldConfig();
    netStateMachine();
    if (mqtt.connected() && nextStatusReportAt < millis()) {
      statusReport();
      nextStatusReportAt = millis() + 2000;
    }
  } else {
    loadConfigFromSerial();
  }
}

// WireSetup binds the I2C bus for core 1 to be able to talk to the
// User Processor.
void WireSetup(int SDA, int SCL) {
  Wire1.setSDA(SDA);
  Wire1.setSCL(SCL);
  Wire1.begin(8);

  Wire1.onRequest(wireRespond);
}

bool loadConfig(String path) {
  delay(3000);
  Serial.println("GIZMO_LOAD_ATTEMPT");
  bool err = false;
  LittleFSConfig lcfg;
  lcfg.setAutoFormat(true);
  LittleFS.setConfig(lcfg);

  if (!LittleFS.begin()) {
    Serial.println("GIZMO_LOAD_FAIL_NOFILE");
    status.SetConfigStatus(CFG_NO_FILE);
    LittleFS.end();
    return false;
  }

  JsonDocument cfgDoc;
  auto f = LittleFS.open(path.c_str(), "r");
  auto error = deserializeJson(cfgDoc, f);
  if (error) {
    Serial.println("GIZMO_LOAD_FAIL_BADPARSE");
    status.SetConfigStatus(CFG_BAD_PARSE);
    f.close();
    LittleFS.end();
    return false;
  } else {
    Serial.println("GIZMO_LOAD_JSON_OK");
  }

  cfg.teamNumber = cfgDoc["Team"] | -1;
  cfg.mqttBroker = cfgDoc["ServerIP"] | "gizmo-ds";
  cfg.netSSID = cfgDoc["NetSSID"] | "";
  cfg.netPSK = cfgDoc["NetPSK"] | "";

  if (cfg.teamNumber == -1 || !cfg.netSSID.length() || !cfg.netPSK.length()) {
    status.SetConfigStatus(CFG_BAD);
    Serial.println("GIZMO_LOAD_FAIL_BADFILE");
    f.close();
    LittleFS.end();
    return false;
  }

  cfg.mqttTopicControl = String("robot/") + cfg.teamNumber + String("/control");
  cfg.mqttTopicLocation = String("robot/") + cfg.teamNumber + String("/location");
  cfg.mqttTopicStats = String("robot/") + cfg.teamNumber + String("/stats");

  status.SetConfigStatus(CFG_OK);
  cfg.loaded = true;
  Serial.println("GIZMO_LOAD_CFG_OK");
  f.close();
  LittleFS.end();
  return true;
}

void loadConfigFromSerial() {
  switch (cfgState) {
  case CFG_INIT:
    status.SetConfigStatus(CFG_NO_FILE);
    if (millis() > 2000) {
      cfgState = CFG_REQUEST;
    }
    break;
  case CFG_REQUEST:
    Serial.println("GIZMO_REQUEST_CONFIG");
    cfgState = CFG_LOAD;
    break;
  case CFG_LOAD:
    if (Serial.available()) {
      String c = Serial.readStringUntil('\n');
      if (c != NULL) {
        LittleFS.begin();
        File f = LittleFS.open("/gsscfg.json", "w");
        f.write(c.c_str());
        f.close();
        LittleFS.end();
        cfgState = CFG_REBOOT;
        Serial.println("GIZMO_CONFIG_UPDATED");
      }
    }
    break;
  case CFG_REBOOT:
    rp2040.wdt_begin(8);
    while (true);
    break;
  }
}

void checkIfShouldConfig() {
  static bool configWaiting;
  static unsigned long checkAt;

  if (!BOOTSEL) {
    configWaiting = false;
  } else {
    if (!configWaiting) {
      checkAt = millis() + 2000;
    }
    configWaiting = true;
    if (millis() > checkAt) {
      cfg.loaded = false;
    }
  }
}

// netStateMachine handles all the various state transitions of the
// network.  It is ticked on every loop in order to avoid needing to
// do any kind of timer management.
void netStateMachine() {
  switch(netState) {
  case NET_SEARCH:
    netStateLinkSearch();
    break;
  case NET_CONNECT_WIFI:
    netStateWifiConnect();
    break;
  case NET_CONNECT_WAIT:
    netStateConnectWait();
    break;
  case NET_FMS_DISCOVER:
    netStateFMSDiscover();
    break;
  case NET_CONNECT_MQTT:
    netStateMQTTConnect();
    break;
  case NET_RUNNING:
    netStateRun();
    break;
  default:
    netState = NET_SEARCH;
  }
}

void netStateLinkSearch() {
  // This function needs to check if we are in wifi or wired mode.
  // This is done by checking if the wired controller has a layer 1
  // link, because if it does it always wins.

  netState = NET_CONNECT_WIFI;
}

void netStateWifiConnect() {
  Serial.println("GIZMO_NET_WIFI_START");
  WiFi.mode(WIFI_STA);
  WiFi.begin(cfg.netSSID.c_str(), cfg.netPSK.c_str());

  netStateConnectTimeout = millis() + 30000;
  netState = NET_CONNECT_WAIT;
}

void netStateConnectWait() {
  if (!WiFi.connected()) {
    if (netStateConnectTimeout > millis()) {
      // We aren't connected yet, but we also aren't timed out.  Leave
      // and wait for more time to pass.
      return;
    } else {
      // If the IP isn't set and we're timed out, go back to searching
      // for a netdev.
      netState = NET_SEARCH;
      return;
    }
  }

  // If we've made it here, we're connected, so bump the counter and
  // flip the status forward.
  boardState.WifiReconnects++;
  status.SetWifiConnected(true);
  netState = NET_FMS_DISCOVER;
  Serial.println("GIZMO_NET_WIFI_RUN");
}

void netStateFMSDiscover() {
  IPAddress ip;
  if (hostByName("gizmo-fms.comp", ip, 2000)) {
    // This gets checked before we check any baked in values because
    // this is how we know if we're connected to a competition mode
    // field and we need to bail right here with a connection to that
    // endpoint.
    cfg.mqttBroker = ip.toString();
    netState = NET_CONNECT_MQTT;
    Serial.println("GIZMO_FMS_DISCOVERED_COMP");
    return;
  }

  if (ip.fromString(cfg.mqttBroker)) {
    // The broker address was an IP and so we can just connect
    // directly.
    netState = NET_CONNECT_MQTT;
    Serial.println("GIZMO_FMS_DISCOVERED_IP");
    return;
  }

  if (hostByName(cfg.mqttBroker.c_str(), ip, 2000)) {
    cfg.mqttBroker = ip.toString();
    // We can jump directly to connect since this is a terminal
    // address.
    netState = NET_CONNECT_MQTT;
    Serial.println("GIZMO_FMS_DISCOVERED_HOSTNAME");
    return;
  }

  // Welp, it wasn't a traditional IP or a hostname that the local DNS
  // knew about.  Lets chance it with mDNS.
  Serial.println("GIZMO_FMS_DISCOVERY_MDNS_START");
  status.SetmDNSRunning(true);
  MDNS.begin(cfg.hostname);

  String svcName = String("gizmo") + cfg.teamNumber;
  int res = MDNS.queryService(svcName.c_str(), "tcp", 10000);
  if (res == 0) {
    MDNS.end();
    Serial.println("GIZMO_FMS_DISCOVERY_MDNS_NOANSWER");
    status.SetmDNSRunning(false);
    return;
  }

  for (int i = 0; i<res ; i++) {
    if (MDNS.answerIP(i) == INADDR_ANY) {
      continue;
    }
    cfg.mqttBroker = MDNS.answerIP(i).toString();
    MDNS.end();
    status.SetmDNSRunning(false);
    netState = NET_CONNECT_MQTT;
    Serial.println("GIZMO_FMS_DISCOVERY_MDNS_STOP");
    return;
  }
}

void netStateMQTTConnect() {
  if (!mqtt.connect(cfg.mqttBroker.c_str(), 1883)) {
    Serial.println("GIZMO_MQTT_CONNECT_FAIL");
    return;
  }
  Serial.println("GIZMO_MQTT_CONNECT_OK");

  // Like and Subscribe
  mqtt.onMessage(mqttParseMessage);
  mqtt.subscribe(cfg.mqttTopicControl);
  mqtt.subscribe(cfg.mqttTopicLocation);
  nextControlPacketDueBy = millis() + 30000;
  netState = NET_RUNNING;
}

void netStateRun() {
  if (!WiFi.connected()) {
    netState = NET_SEARCH;
    status.SetControlConnected(false);
    Serial.println("GIZMO_NET_DISCONNECT");
    return;
  }

  if (nextControlPacketDueBy < millis()) {
    netState = NET_CONNECT_MQTT;
    status.SetControlConnected(false);
    Serial.println("GIZMO_MQTT_DISCONNECT");
    return;
  }

  // If we made it this far then the network is still running, so do
  // the maintenance tasks for the network.
  mqtt.poll();
}

void mqttParseMessage(int messageSize) {
  Serial.println("GIZMO_MQTT_MSG");

  nextControlPacketDueBy = millis() + 500;
  status.SetControlConnected(true);

  if (mqtt.messageTopic() == cfg.mqttTopicControl) {
    mqttParseControl();
  } else if (mqtt.messageTopic() == cfg.mqttTopicLocation) {
    mqttParseLocation();
  }
}

void mqttParseControl() {
  deserializeJson(cstateJSON, mqtt);
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

  controlFrameAge = millis();
  boardState.FramesReceived++;
}

void mqttParseLocation() {
  deserializeJson(fstateJSON, mqtt);
  status.SetFieldNumber(fstateJSON["Field"]);

  if (fstateJSON["Quadrant"] == "RED") {
    status.SetFieldQuadrant(GIZMO_QUAD_RED);
  } else if (fstateJSON["Quadrant"] == "BLUE") {
    status.SetFieldQuadrant(GIZMO_QUAD_BLUE);
  } else if (fstateJSON["Quadrant"] == "GREEN") {
    status.SetFieldQuadrant(GIZMO_QUAD_GREEN);
  } else if (fstateJSON["Quadrant"] == "YELLOW") {
    status.SetFieldQuadrant(GIZMO_QUAD_YELLOW);
  } else if (fstateJSON["Quadrant"] == "PRACTICE") {
    status.SetFieldQuadrant(GIZMO_QUAD_PRACTICE);
  }
  return;
}

// This has to be static to be used in the class level callback in the
// Wire1 object.  Its ugly but it means that the rest of the code can
// be nicely subdivided into member functions.
void wireRespond() {
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
}

void statusUpdate() {
  boardState.VBat = analogRead(pinStatusPwrSupply);
  boardState.WatchdogRemaining = 0;
  boardState.WatchdogOK = false;
  boardState.RSSI = WiFi.RSSI();
  boardState.PwrBoard = digitalRead(pinStatusPwrBoard);
  boardState.PwrPico  = digitalRead(pinStatusPwrPico);
  boardState.PwrGPIO  = digitalRead(pinStatusPwrGPIO);
  boardState.PwrMainA = digitalRead(pinStatusPwrMainA);
  boardState.PwrMainB = digitalRead(pinStatusPwrMainB);

  // This converts from what the ADC reads to a voltage that's pretty
  // close to reality.  There's some slop in this calculation because
  // we aren't really doing everything totally correct with all the
  // analog biasing on the input.  This number was calculated by a
  // linear regression over sampled values.
  float voltage = 0.008833 * boardState.VBat + 0.3017;

  if (voltage>=8) {
    status.SetBatteryLevel(GIZMO_BAT_FULL);
  } else if (voltage<8 && voltage>=7.5) {
    status.SetBatteryLevel(GIZMO_BAT_GOOD);
  } else if (voltage<7.5 && voltage>=7) {
    status.SetBatteryLevel(GIZMO_BAT_PASS);
  } else if (voltage<7) {
    status.SetBatteryLevel(GIZMO_BAT_DEAD);
  }
}

void statusReport() {
  String output;
  JsonDocument posting;
  posting["ControlFrameAge"] = millis() - controlFrameAge;
  posting["ControlFramesReceived"] = boardState.FramesReceived;
  posting["WifiReconnects"] = boardState.WifiReconnects;
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
  mqtt.beginMessage(cfg.mqttTopicStats);
  mqtt.print(output);
  mqtt.endMessage();
}
