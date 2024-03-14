#ifndef GIZMO_INDICATORS_H
#define GIZMO_INDICATORS_H

#include <Adafruit_NeoPixel.h>

#define GIZMO_INDICATE_NET 0
#define GIZMO_INDICATE_FIELD 1
#define GIZMO_INDICATE_BAT 2

#define GIZMO_QUAD_RED 0
#define GIZMO_QUAD_BLUE 1
#define GIZMO_QUAD_GREEN 2
#define GIZMO_QUAD_YELLOW 3
#define GIZMO_QUAD_PRACTICE 4

#define GIZMO_BAT_FULL 4
#define GIZMO_BAT_GOOD 3
#define GIZMO_BAT_PASS 2
#define GIZMO_BAT_DEAD 1

const unsigned long blinkFieldNum[3][6] = { { 500, 100 }, { 500, 100, 100, 100 }, {500, 100, 100, 100, 100, 100}};
const byte blinkFieldLen[] = { 2, 4, 6 };
class StatusIndicators
{
public:
  StatusIndicators(int, int);
  void Update();
  void SetWifiConnected(bool);
  void SetControlConnected(bool);
  void SetFieldNumber(int);
  void SetFieldQuadrant(int);
  void SetBatteryLevel(byte);
private:
  Adafruit_NeoPixel pixels;
  uint32_t toggleNetworkAt;
  uint32_t toggleFieldAt;
  byte toggleFieldCnt;
  byte fieldPos;
  byte fieldNum;
  byte batLevel;
  bool wifiConnected;
  bool ctrlConnected;
  void doWifiSet();
  void doFieldSet();
  void doSetColorForFieldPos(byte);
  void doBatterySet();
  uint32_t wheel(byte);
};

#endif
