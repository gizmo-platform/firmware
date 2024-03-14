#include "gizmo.h"

#include "Arduino.h"
#include "indicators.h"
#include <ArduinoJson.h>
#include <ArduinoMqttClient.h>
#include <WiFi.h>
#include <Wire.h>

// Team Number shouldn't be longer than 4 digits, but we'll use an int
// anyway.
int teamNum;

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
WiFiClient wifi;
MqttClient mqtt(wifi);

CState cstate;
BoardState boardState;
NetState netState;

String hostname;
String fieldLocation;
String mqttTopicControl;
String mqttTopicLocation;
String mqttTopicStats;
String mqttBroker;
String netSSID;
String netPSK;

JsonDocument cstateJSON;
JsonDocument fstateJSON;

unsigned long nextControlPacketDueBy;
unsigned long controlFrameAge;
unsigned long netStateConnectTimeout;

// function declarations for "private" functions.
void netStateMachine();
void netStateLinkSearch();
void netStateWifiConnect();
void netStateConnectWait();
void netStateMQTTConnect();
void netStateRun();

void mqttParseMessage(int);
void mqttParseLocation();
void mqttParseControl();

void wireRespond();

void statusUpdate();
void statusReport();

void ConfigureTeamNumber(int team) {
  teamNum = team;
  hostname = String("robot") + team;
}

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

void ConfigureStatusPixels(byte pin, byte count) {
  //status.begin(pin, count);
}

void ConfigureDefaultBroker(String broker) {
  mqttBroker = broker;
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

  WiFi.setHostname(hostname.c_str());
  WiFi.noLowPowerMode();
  WiFi.setTimeout(500);

  String teamNumStr = String(teamNum);

  mqttTopicControl  = "robot/" + teamNumStr + "/gamepad";
  mqttTopicLocation = "robot/" + teamNumStr + "/location";
  mqttTopicStats    = "robot/" + teamNumStr + "/stats";

  statusUpdate();
  status.Update();
  netState = NET_SEARCH;
}

// Tick runs all the functions that need to happen on every loop
// through.  Not every one of these functions will do something, but
// they do need to be ticked on each iteration.
void GizmoTick() {
  status.Update();
  netStateMachine();
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

// SetWifiNet configures the wireless network out of a location that
// is not contained in this class.
void SetWifiNet(String ssid, String psk) {
  netSSID = ssid;
  netPSK = psk;
}

// WireSetup binds the I2C bus for core 1 to be able to talk to the
// User Processor.
void WireSetup(int SDA, int SCL) {
  Wire1.setSDA(SDA);
  Wire1.setSCL(SCL);
  Wire1.begin(8);

  Wire1.onRequest(wireRespond);
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
  WiFi.mode(WIFI_STA);
  WiFi.begin(netSSID.c_str(), netPSK.c_str());

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
  netState = NET_CONNECT_MQTT;
}

void netStateMQTTConnect() {
  if (!mqtt.connect(mqttBroker.c_str(), 1883)) {
    return;
  }

  // Like and Subscribe
  mqtt.onMessage(mqttParseMessage);
  mqtt.subscribe(mqttTopicControl);
  mqtt.subscribe(mqttTopicLocation);
  nextControlPacketDueBy = millis() + 30000;
  netState = NET_RUNNING;
}

void netStateRun() {
  if (!WiFi.connected()) {
    netState = NET_SEARCH;
    status.SetControlConnected(false);
    return;
  }

  if (nextControlPacketDueBy < millis()) {
    netState = NET_CONNECT_MQTT;
    status.SetControlConnected(false);
    return;
  }

  // If we made it this far then the network is still running, so do
  // the maintenance tasks for the network.
  mqtt.poll();
}

void mqttParseMessage(int messageSize) {
  nextControlPacketDueBy = millis() + 500;
  status.SetControlConnected(true);

  if (mqtt.messageTopic() == mqttTopicControl) {
    mqttParseControl();
  } else if (mqtt.messageTopic() == mqttTopicLocation) {
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
  // Fetch updated information
  statusUpdate();

  // Serialize State
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
  mqtt.beginMessage(mqttTopicStats);
  mqtt.print(output);
  mqtt.endMessage();
}
