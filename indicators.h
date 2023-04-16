#ifndef BRI_INDICATORS_H
#define BRI_INDICATORS_H

#include <Adafruit_NeoPixel.h>

#define BRI_INDICATE_NET 0
#define BRI_INDICATE_FIELD 1

#define BRI_QUAD_RED 0
#define BRI_QUAD_BLUE 1
#define BRI_QUAD_GREEN 2
#define BRI_QUAD_YELLOW 3
#define BRI_QUAD_PRACTICE 4

const unsigned long blinkFieldNum[3][6] = { { 500, 300 }, { 500, 100, 100, 100 }, {500, 100, 100, 100, 100, 100}};
const byte blinkFieldLen[] = { 2, 4, 6 };
class StatusIndicators
{
public:
  StatusIndicators(int pin, int count);
  void Update();
  void SetWifiConnected(bool);
  void SetControlConnected(bool);
  void SetFieldNumber(int);
  void SetFieldQuadrant(int);
private:
  Adafruit_NeoPixel pixels;
  uint32_t toggleNetworkAt;
  uint32_t toggleFieldAt;
  byte toggleFieldCnt;
  byte fieldPos;
  byte fieldNum;
  bool wifiConnected;
  bool ctrlConnected;
  void doWifiSet();
  void doFieldSet();
  void doSetColorForFieldPos(byte);
  uint32_t wheel(byte);
};

#endif
