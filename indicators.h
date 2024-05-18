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

enum ConfigStatus {
  CFG_NO_FILE,
  CFG_BAD_PARSE,
  CFG_BAD,
  CFG_OK,
};

const unsigned long blinkFieldNum[3][6] = { { 500, 100 }, { 500, 100, 100, 100 }, {500, 100, 100, 100, 100, 100}};
const byte blinkFieldLen[] = { 2, 4, 6 };
class StatusIndicators
{
public:
  StatusIndicators(int, int);
  void Update();
  void SetNetConnected(bool);
  void SetControlConnected(bool);
  void SetFieldNumber(int);
  void SetFieldQuadrant(int);
  void SetBatteryLevel(byte);
  void SetConfigStatus(ConfigStatus);
private:
  Adafruit_NeoPixel pixels;
  uint32_t toggleFieldAt;
  uint32_t toggleCfgAt;
  ConfigStatus cfgStatus;
  byte toggleFieldCnt;
  byte toggleCfgCnt;
  byte fieldPos;
  byte fieldNum;
  byte batLevel;
  bool netConnected;
  bool ctrlConnected;
  void doNetSet();
  void doFieldSet();
  void doSetColorForFieldPos(byte);
  void doBatterySet();
  void doCfgSet();
  uint32_t wheel(byte);
};

#endif
