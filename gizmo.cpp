#include "gizmo.h"
#include "config.h"

#include "Arduino.h"
#include "indicators.h"
#include <ArduinoJson.h>
#include <ArduinoMqttClient.h>
#include <WiFi.h>
#include <Wire.h>
#include <SPI.h>
#include "LittleFS.h"
#include <W5500lwIP.h>

Config cfg;
CfgState cfgState = CFG_INIT;

// This controls how long after the last control frame was receieved
// we should consider the control channel to have timed out.  This
// value is a balance between making sure that robots don't run away,
// but also making sure that there's enough slop in the system that a
// packet that is delayed doesn't cause a reset.  This value is in
// milliseconds, and is set as the time that would have to pass to
// miss 5 packets at 50Hz with a little slop.
//
// This value can be adjusted dynamically depending on where the
// broker is determined to be.
int controlTimeout = 1000;

// Should really be const, but we don't know these until after
// initialization.
byte pinStatusPwrSupply;
byte pinStatusPwrBoard;
byte pinStatusPwrPico;
byte pinStatusPwrGPIO;
byte pinStatusPwrServo;
byte pinStatusPwrMainA;
byte pinStatusPwrMainB;
byte pinStatusPwrPixels;

float pinStatusPwrSupplyTuneM;
float pinStatusPwrSupplyTuneB;

// This is the pin that can be toggled to force the User Processor
// into reset.
byte pinUserReset;

// This can be used to force the WizNet to reset
byte pinWiznetReset;

StatusIndicators status(15, 4);
Wiznet5500lwIP eth(GIZMO_HW_ENET_CS, SPI, GIZMO_HW_ENET_INT);
WiFiClient network;
MqttClient mqtt(network);
IPAddress mqttIP;

bool enetAvailable = false;

CState cstate;
BoardState boardState;
NetState netState;
NetLink netLink;

String fieldLocation;

JsonDocument cstateJSON;
JsonDocument fstateJSON;

unsigned long nextControlPacketDueBy;
unsigned long controlFrameAge;
unsigned long netStateConnectTimeout;
unsigned long nextStatusReportAt;
unsigned long nextMetaReportAt;

// function declarations for "private" functions.
bool loadConfig(String);
void loadConfigFromSerial();
void checkIfShouldConfig();
void zeroizeCState();

bool netLinkOk();
void netLinkResetWiznet();
void netStateMachine();
void netStateLinkSearch();
void netStateWifiConnect();
void netStateConnectWaitWifi();
void netStateConnectWaitEnet();
void netStateFMSDiscover();
void netStateMQTTConnect();
void netStateRun();

void mqttParseMessage(int);
void mqttParseLocation();
void mqttParseControl();

void wireRespond();

void statusUpdate();
void statusReport();
void metaReport();

void logMQTTError();

void ConfigureStatusIO(byte supply, byte board, byte pico, byte gpio, byte servo, byte mainA, byte mainB, byte pixels) {
  pinStatusPwrSupply = supply;
  pinStatusPwrBoard = board;
  pinStatusPwrPico = pico;
  pinStatusPwrGPIO = gpio;
  pinStatusPwrServo = servo;
  pinStatusPwrMainA = mainA;
  pinStatusPwrMainB = mainB;
  pinStatusPwrPixels = pixels;
}

void ConfigureBoardVoltageTuning(float m, float b) {
  pinStatusPwrSupplyTuneM = m;
  pinStatusPwrSupplyTuneB = b;
}

void ConfigureUserReset(byte rst) {
  pinUserReset = rst;
}

void ConfigureWiznetReset(byte rst) {
  pinWiznetReset = rst;
}


void zeroizeCState() {
  cstate.Button0 = false;
  cstate.Button1 = false;
  cstate.Button2 = false;
  cstate.Button3 = false;
  cstate.Button4 = false;
  cstate.Button5 = false;
  cstate.Button6 = false;
  cstate.Button7 = false;
  cstate.Button8 = false;
  cstate.Button9 = false;
  cstate.Button10 = false;
  cstate.Button11 = false;

  cstate.Axis0 = 127;
  cstate.Axis1 = 127;
  cstate.Axis2 = 127;
  cstate.Axis3 = 127;
  cstate.Axis4 = 127;
  cstate.Axis5 = 127;
}

void GizmoSetup() {
  // Ensure that the cstate values for axis data start at a reasonable
  // zero point.  This prevents machines from running off into the
  // wild blue yonder.
  zeroizeCState();

  pinMode(pinStatusPwrBoard, INPUT);
  pinMode(pinStatusPwrPico, INPUT);
  pinMode(pinStatusPwrGPIO, INPUT);
  pinMode(pinStatusPwrServo, INPUT);
  pinMode(pinStatusPwrMainA, INPUT);
  pinMode(pinStatusPwrMainB, INPUT);
  pinMode(pinStatusPwrPixels, INPUT);

  pinMode(pinUserReset, OUTPUT);
  pinMode(pinWiznetReset, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(pinWiznetReset, HIGH);
  Serial.begin(9600);

  // This slows down the boot long enough that the serial load
  // messages can be printed.  Its a tradeoff between boot speed and
  // debugability, and for the moment we're favoring debugability.
  delay(3000);

  loadConfig("/gsscfg.json");
  Serial.print("GIZMO_HARDWARE ");
  Serial.println(GIZMO_HW_VERSION);
  Serial.print("GIZMO_FIRMWARE ");
  Serial.println(GIZMO_FW_VERSION);

  // We set SPI here for the wiznet breakout
  SPI.setRX(0);
  SPI.setCS(1);
  SPI.setSCK(2);
  SPI.setTX(3);

  // We set SPI1 here to be correct for the only possible breakout on the board
  SPI1.setRX(12);
  SPI1.setCS(13);
  SPI1.setSCK(14);
  SPI1.setTX(11);

  WiFi.setHostname(cfg.hostname);
  WiFi.noLowPowerMode();
  WiFi.setTimeout(500);

  // Enable the hardware watchdog.  If we've gotten stuck for more
  // than a few seconds something has broken - badly.
  rp2040.wdt_begin(5000);

  // Administrative region
  byte d1 = (cfg.teamNumber / 1000) * 16;
  byte d2 = (cfg.teamNumber % 1000) / 100;
  byte d3 = (cfg.teamNumber % 1000) % 100 / 10 * 16;
  byte d4 = (cfg.teamNumber % 1000) % 100 % 10;

  byte mac[] = {0x02, 0x00, 0x00, byte(d1+d2), byte(d3+d4), 0x00};
  netLinkResetWiznet();
  // Necessary to wait here to allow the Wiznet time to come back.
  delay(50);
  eth.setSPISpeed(30000000);
  if (!eth.begin(mac)) {
    Serial.println("GIZMO_ENET_INIT_FAILED");
  } else {
    Serial.println("GIZMO_ENET_INIT");
    enetAvailable = true;
    lwipPollingPeriod(5);
  }

  statusUpdate();
  status.Update();
  netState = NET_SEARCH;
}

// Tick runs all the functions that need to happen on every loop
// through.  Not every one of these functions will do something, but
// they do need to be ticked on each iteration.
void GizmoTick() {
  rp2040.wdt_reset();
  statusUpdate();
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

  if (cfg.loaded) {
    checkIfShouldConfig();
    netStateMachine();
    if (mqtt.connected() && (nextStatusReportAt < millis()) && (nextControlPacketDueBy > millis())) {
      statusReport();
      nextStatusReportAt = millis() + 2000;
    }
    if (mqtt.connected() && (nextMetaReportAt < millis()) && (nextControlPacketDueBy > millis())) {
      metaReport();
      nextMetaReportAt = millis() + 10000;
    }
  } else {
    loadConfigFromSerial();
  }

  // This allows yielded threads to wake and execute.  It was
  // recommended by the MQTT library vendor for stability, though the
  // vendor did not articulate fully how it improved the stability.
  delay(10);
}

void GizmoTick1() {
  status.Update();
  delay(5);
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

  sprintf(cfg.hostname, "gizmo-%d", cfg.teamNumber);

  if (cfg.teamNumber == -1 || !cfg.netSSID.length() || !cfg.netPSK.length()) {
    status.SetConfigStatus(CFG_BAD);
    Serial.println("GIZMO_LOAD_FAIL_BADFILE");
    f.close();
    LittleFS.end();
    return false;
  }

  cfg.mqttTopicControl = String("robot/") + cfg.teamNumber + String("/gamepad");
  cfg.mqttTopicLocation = String("robot/") + cfg.teamNumber + String("/location");
  cfg.mqttTopicStats = String("robot/") + cfg.teamNumber + String("/stats");
  cfg.mqttTopicMeta = String("robot/") + cfg.teamNumber + String("/gizmo-meta");

  status.SetConfigStatus(CFG_OK);
  cfg.loaded = true;
  Serial.println("GIZMO_LOAD_CFG_OK");
  f.close();
  LittleFS.end();

  Serial.print("GIZMO_CFG_TEAM ");
  Serial.print(cfg.teamNumber);
  Serial.println();
  Serial.println("GIZMO_CFG_SERVER " + cfg.mqttBroker);
  Serial.printf("GIZMO_CFG_HOSTNAME %s\r\n", cfg.hostname);
  return true;
}

void loadConfigFromSerial() {
  static long _load_timeout = 0;
  static String c;

  switch (cfgState) {
  case CFG_INIT:
    status.SetConfigStatus(CFG_NO_FILE);
    if (millis() > 2000) {
      cfgState = CFG_REQUEST;
    }
    break;
  case CFG_REQUEST:
    Serial.println("GIZMO_REQUEST_CONFIG");
    _load_timeout = millis() + 15000;
    cfgState = CFG_WAIT;
    break;
  case CFG_WAIT:
    Serial.println("GIZMO_CONFIG_WAIT");
    if (Serial.available()) {
      cfgState = CFG_LOAD;
      break;
    }
    if (_load_timeout < millis()) {
      cfgState = CFG_REQUEST;
      break;
    }
  case CFG_LOAD:
    c = Serial.readStringUntil('\n');
    if (c != NULL) {
      LittleFS.begin();
      File f = LittleFS.open("/gsscfg.json", "w");
      f.write(c.c_str());
      f.close();
      LittleFS.end();
      cfgState = CFG_REBOOT;
      Serial.println("GIZMO_CONFIG_UPDATED");
    }
    break;
  case CFG_REBOOT:
    // Long enough to take your finger off so that you don't get stuck
    // in DFU mode.
    rp2040.wdt_begin(1000);
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

// netLinkOk gives us an easy way to change network state to NET_SEARCH
// in a consistent way that minimizes the different ways this could
// happen. A message gets printed, the neopixels are updated, and we go
// back to searching for networks.
bool netLinkOk() {
  if (netLink == NET_WIRELESS && enetAvailable && eth.isLinked() ||
      netLink == NET_WIRELESS && !WiFi.connected() ||
      netLink == NET_WIRED && enetAvailable && !eth.isLinked()) {
    netState = NET_SEARCH;
    Serial.println("GIZMO_NET_DISCONNECT");
    status.SetControlConnected(false);
    status.SetNetConnected(false);
    return false;
  }
  return true;
}

void netLinkResetWiznet() {
      digitalWrite(pinWiznetReset, LOW);
      delay(10);
      digitalWrite(pinWiznetReset, HIGH);
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
  case NET_CONNECT_WAIT_WIFI:
    netStateConnectWaitWifi();
    break;
  case NET_CONNECT_WAIT_ENET:
    netStateConnectWaitEnet();
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
  if (enetAvailable && eth.isLinked()) {
    netState = NET_CONNECT_WAIT_ENET;
    Serial.println("GIZMO_NET_ENET_ATTEMPT");
    netStateConnectTimeout = millis() + 5000;
    WiFi.disconnect();
    WiFi.end();
    return;
  }

  Serial.println("GIZMO_NET_WIFI_ATTEMPT");
  netState = NET_CONNECT_WIFI;
  netStateConnectTimeout = millis() + 30000;
}

void netStateConnectWaitEnet() {
  if (!eth.connected()) {
    if (netStateConnectTimeout > millis()) {
      // We aren't connected yet, but we also aren't timed out.  Leave
      // and wait for more time to pass.
      delay(20);
      return;
    } else {
      // If the IP isn't set and we're timed out, go back to searching
      // for a netdev.
      Serial.println("GIZMO_NET_ENET_TIMEOUT");
      netState = NET_SEARCH;
      return;
    }
  }
  status.SetNetConnected(true);
  Serial.println("GIZMO_NET_IP " + eth.localIP().toString());
  eth.setDNS(eth.gatewayIP());
  Serial.println("GIZMO_NET_DNS " + eth.dnsIP().toString());
  netState = NET_FMS_DISCOVER;
  netLink = NET_WIRED;
  Serial.println("GIZMO_NET_WIRED_RUN");
}

void netStateWifiConnect() {
  Serial.println("GIZMO_NET_WIFI_START");
  WiFi.mode(WIFI_STA);
  WiFi.beginNoBlock(cfg.netSSID.c_str(), cfg.netPSK.c_str());

  netState = NET_CONNECT_WAIT_WIFI;
}

void netStateConnectWaitWifi() {
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
  status.SetNetConnected(true);
  WiFi.setDNS(WiFi.dnsIP());

  Serial.println("GIZMO_NET_IP " + WiFi.localIP().toString());
  Serial.println("GIZMO_NET_DNS " + WiFi.dnsIP().toString());
  netState = NET_FMS_DISCOVER;
  netLink = NET_WIRELESS;
  Serial.println("GIZMO_NET_WIFI_RUN");
}

void netStateFMSDiscover() {
  Serial.println("GIZMO_FMS_DISCOVER_START");
  if (!netLinkOk()) {
    return;
  }

  // The DNS checks for our "special" names validate that nothing
  // returns for a name we can be certain doesn't exist.  This catches
  // several edge cases with misbehaving DNS servers when using
  // external network controllers.

  if (!hostByName("nxdomain.gizmo", mqttIP, 2000) && hostByName("fms.gizmo", mqttIP, 2000)) {
    // This gets checked before we check any baked in values because
    // this is how we know if we're connected to a competition mode
    // field and we need to bail right here with a connection to that
    // endpoint.
    cfg.mqttBroker = mqttIP.toString();
    netState = NET_CONNECT_MQTT;
    Serial.println("GIZMO_FMS_DISCOVERED_COMP " + mqttIP.toString());
    // We tighten this up to 500ms which provides really stable
    // operation on the FMS, while still handling the occasional lags
    // during radio boot up.
    controlTimeout = 1000;
    return;
  }

  if (!hostByName("nxdomain.gizmo", mqttIP, 2000) && hostByName("ds.gizmo", mqttIP, 2000)) {
    cfg.mqttBroker = mqttIP.toString();
    netState = NET_CONNECT_MQTT;
    Serial.println("GIZMO_FMS_DISCOVERED_DS " + mqttIP.toString());
    // We can massively tighten up the timeout if we're on the
    // driver's station since that's a point to point link with known
    // latency characteristics.
    controlTimeout = 500;
    return;
  }

  if (hostByName(cfg.mqttBroker.c_str(), mqttIP, 2000)) {
    cfg.mqttBroker = mqttIP.toString();
    // We can jump directly to connect since this is a terminal
    // address.
    netState = NET_CONNECT_MQTT;
    Serial.println("GIZMO_FMS_DISCOVERED_HOSTNAME " + mqttIP.toString());
    return;
  }

  if (mqttIP.fromString(cfg.mqttBroker)) {
    // The broker address was an IP and so we can just connect
    // directly.
    netState = NET_CONNECT_MQTT;
    Serial.println("GIZMO_FMS_DISCOVERED_IP");
    return;
  }
}

void netStateMQTTConnect() {
  Serial.printf("GIZMO_MQTT_CTRL_LIMIT %d\r\n", controlTimeout);
  Serial.println("GIZMO_MQTT_CONNECT_START");
  if (!netLinkOk()) {
    return;
  }

  Serial.println("GIZMO_MQTT_TARGET " + cfg.mqttBroker);
  if (!mqtt.connect(mqttIP)) {
    logMQTTError();
    return;
  }
  Serial.println("GIZMO_MQTT_CONNECT_OK");

  // Like and Subscribe
  mqtt.onMessage(mqttParseMessage);
  Serial.println("GIZMO_MQTT_SUBSCRIBE_TOPIC_CONTROL " + cfg.mqttTopicControl);
  int ret = mqtt.subscribe(cfg.mqttTopicControl, 0);
  if (ret != 1) {
    Serial.printf("GIZMO_MQTT_SUBSCRIBE_CONTROL_FAIL %d\r\n", ret);
  }
  Serial.println("GIZMO_MQTT_SUBSCRIBE_TOPIC_LOCATION " + cfg.mqttTopicLocation);
  ret = mqtt.subscribe(cfg.mqttTopicLocation, 0);
  if (ret != 1) {
    Serial.printf("GIZMO_MQTT_SUBSCRIBE_LOCATION_FAIL %d\r\n", ret);
  }
  Serial.println("GIZMO_MQTT_SUBSCRIBE_OK");
  nextControlPacketDueBy = millis() + 30000;
  nextStatusReportAt = millis() + 2000;
  nextMetaReportAt = millis() + 1500;
  netState = NET_RUNNING;
}

void netStateRun() {
  if (!netLinkOk()) {
    return;
  }

  if (nextControlPacketDueBy < millis()) {
    netState = NET_CONNECT_MQTT;
    status.SetControlConnected(false);
    Serial.printf("GIZMO_MQTT_CTRL_TIMEOUT %d\r\n", (millis() - nextControlPacketDueBy));
    zeroizeCState();
    return;
  }

  // If we made it this far then the network is still running, so do
  // the maintenance tasks for the network.
  mqtt.poll();

  // The interrupt needs us to be doing nothing so it can preempt,
  // this is only necessary when running the polling mode ethernet
  // lwIP interface.
  if (netLink == NET_WIRED) {
    delay(10);
  }
}

void mqttParseMessage(int messageSize) {
  if (mqtt.messageTopic() == cfg.mqttTopicControl) {
    nextControlPacketDueBy = millis() + controlTimeout;
    status.SetControlConnected(true);
    mqttParseControl();
  } else if (mqtt.messageTopic() == cfg.mqttTopicLocation) {
    mqttParseLocation();
  }
}

void mqttParseControl() {
  static int msgCounter = 0;
  if (msgCounter>50) {
    Serial.println("GIZMO_MQTT_MSG_CONTROL_X50");
    msgCounter = 0;
  };
  msgCounter++;
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
  Serial.println("GIZMO_MQTT_MSG_LOCATION");
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
  boardState.PwrBoard  = digitalRead(pinStatusPwrBoard);
  boardState.PwrPico   = digitalRead(pinStatusPwrPico);
  boardState.PwrGPIO   = digitalRead(pinStatusPwrGPIO);
  boardState.PwrServo  = digitalRead(pinStatusPwrServo);
  boardState.PwrMainA  = digitalRead(pinStatusPwrMainA);
  boardState.PwrMainB  = digitalRead(pinStatusPwrMainB);
  boardState.PwrPixels = digitalRead(pinStatusPwrPixels);

  // These are tuning constants that get defined on a per-hardware
  // level since they're part of the board stepping.  To generate new
  // ones set M to 1 and B to 0 and then collect a range of voltages
  // and values and plot a linear fit.  R^2 should be very high as the
  // ADC has near perfect linear response.
  float voltage = pinStatusPwrSupplyTuneM * boardState.VBat + pinStatusPwrSupplyTuneB;

  if (voltage>=7.2) {
    status.SetBatteryLevel(GIZMO_BAT_FULL);
  } else if (voltage<7.2 && voltage>=7) {
    status.SetBatteryLevel(GIZMO_BAT_GOOD);
  } else if (voltage<7 && voltage>=6.5) {
    status.SetBatteryLevel(GIZMO_BAT_PASS);
  } else if (voltage<6.5) {
    status.SetBatteryLevel(GIZMO_BAT_DEAD);
  }
}

void statusReport() {
  JsonDocument posting;
  posting["ControlFrameAge"] = millis() - controlFrameAge;
  posting["ControlFramesReceived"] = boardState.FramesReceived;
  posting["WifiReconnects"] = boardState.WifiReconnects;
  posting["VBat"] = boardState.VBat;
  posting["VBatM"] = int(pinStatusPwrSupplyTuneM * 100000);
  posting["VBatB"] = int(pinStatusPwrSupplyTuneB * 100000);
  posting["WatchdogRemaining"] = boardState.WatchdogRemaining;
  posting["RSSI"] = boardState.RSSI;
  posting["PwrBoard"] = boardState.PwrBoard;
  posting["PwrPico"] = boardState.PwrPico;
  posting["PwrGPIO"] = boardState.PwrGPIO;
  posting["PwrServo"] = boardState.PwrServo;
  posting["PwrMainA"] = boardState.PwrMainA;
  posting["PwrMainB"] = boardState.PwrMainB;
  posting["PwrPixels"] = boardState.PwrPixels;
  posting["WatchdogOK"] = boardState.WatchdogOK;

  // Push to the FMS
  mqtt.beginMessage(cfg.mqttTopicStats, (unsigned long)measureJson(posting));
  serializeJson(posting, mqtt);
  mqtt.endMessage();
}

void metaReport() {
  JsonDocument posting;
  posting["HardwareVersion"] = GIZMO_HW_VERSION;
  posting["FirmwareVersion"] = GIZMO_FW_VERSION;

  mqtt.beginMessage(cfg.mqttTopicMeta, (unsigned long)measureJson(posting));
  serializeJson(posting, mqtt);
  mqtt.endMessage();
}

void logMQTTError() {
  Serial.printf("GIZMO_MQTT_FAIL ");
  switch(mqtt.connectError()) {
  case MQTT_CONNECTION_REFUSED:
    Serial.println("MQTT_CONNECTION_REFUSED");
    break;
  case MQTT_CONNECTION_TIMEOUT:
    Serial.println("MQTT_CONNECTION_TIMEOUT");
    break;
  case MQTT_SUCCESS:
    Serial.println("MQTT_SUCCESS");
    break;
  case MQTT_UNACCEPTABLE_PROTOCOL_VERSION:
    Serial.println("MQTT_UNACCEPTABLE_PROTOCOL_VERSION");
    break;
  case MQTT_IDENTIFIER_REJECTED:
    Serial.println("MQTT_IDENTIFIER_REJECTED");
    break;
  case MQTT_SERVER_UNAVAILABLE:
    Serial.println("MQTT_SERVER_UNAVAILABLE");
    break;
  case MQTT_BAD_USER_NAME_OR_PASSWORD:
    Serial.println("MQTT_BAD_USER_NAME_OR_PASSWORD");
    break;
  case MQTT_NOT_AUTHORIZED:
    Serial.println("MQTT_NOT_AUTHORIZED");
    break;
  }
}
