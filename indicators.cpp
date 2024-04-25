#include "indicators.h"
#include "Arduino.h"

StatusIndicators::StatusIndicators(int pin, int count) {
  pixels = Adafruit_NeoPixel(count, pin, NEO_GRB + NEO_KHZ800);

  pixels.begin();
  pixels.setBrightness(20);
  pixels.show();
}

void StatusIndicators::Update() {
  if (cfgStatus != CFG_OK) {
    doCfgSet();
  } else {
    doNetSet();
    doFieldSet();
    doBatterySet();
  }
  pixels.show();
}

void StatusIndicators::SetNetConnected(bool netState) {
  netConnected = netState;
}

void StatusIndicators::SetControlConnected(bool ctrlState) {
  ctrlConnected = ctrlState;
}

void StatusIndicators::SetFieldQuadrant(int pos) {
  fieldPos = pos;
}

void StatusIndicators::SetFieldNumber(int num) {
  fieldNum = num;
}

void StatusIndicators::SetBatteryLevel(byte lvl) {
  batLevel = lvl;
}

void StatusIndicators::SetConfigStatus(ConfigStatus c) {
  cfgStatus = c;
}

void StatusIndicators::doNetSet() {
  if (netConnected && ! ctrlConnected) {
    pixels.setPixelColor(GIZMO_INDICATE_NET, 255, 255, 0);
  } else if (netConnected && ctrlConnected) {
    pixels.setPixelColor(GIZMO_INDICATE_NET, 0, 255, 0);
  } else {
    pixels.setPixelColor(GIZMO_INDICATE_NET, 255, 0, 0);
  }
}

void StatusIndicators::doFieldSet() {
  if (! netConnected) {
    pixels.setPixelColor(GIZMO_INDICATE_FIELD, 0, 0, 0);
    return;
  } else if ( netConnected && ! ctrlConnected) {
    pixels.setPixelColor(GIZMO_INDICATE_FIELD, wheel((millis() / 5) % 255));
    return;
  }

  if (!fieldNum) return ;
  if (toggleFieldCnt > blinkFieldLen[fieldNum-1]-1) {
    toggleFieldCnt = 0;
  }
  if (toggleFieldAt < millis()) {
    if (toggleFieldCnt % 2 == 0) {
      pixels.setPixelColor(GIZMO_INDICATE_FIELD, 0, 0, 0);
    } else {
      doSetColorForFieldPos(fieldPos);
    }
    toggleFieldAt = millis() + blinkFieldNum[fieldNum-1][toggleFieldCnt];
    toggleFieldCnt++;
  }
}

void StatusIndicators::doSetColorForFieldPos(byte quad) {
  switch (quad) {
  case GIZMO_QUAD_RED:
    pixels.setPixelColor(GIZMO_INDICATE_FIELD, 255, 0, 0);
    break;
  case GIZMO_QUAD_BLUE:
    pixels.setPixelColor(GIZMO_INDICATE_FIELD, 0, 0, 255);
    break;
  case GIZMO_QUAD_GREEN:
    pixels.setPixelColor(GIZMO_INDICATE_FIELD, 0, 255, 0);
    break;
  case GIZMO_QUAD_YELLOW:
    pixels.setPixelColor(GIZMO_INDICATE_FIELD, 255, 255, 0);
    break;
  case GIZMO_QUAD_PRACTICE:
    pixels.setPixelColor(GIZMO_INDICATE_FIELD, 255, 255, 255);
    break;
  }
}

void StatusIndicators::doBatterySet() {
  switch (batLevel) {
  case GIZMO_BAT_FULL:
    pixels.setPixelColor(GIZMO_INDICATE_BAT, 0, 255, 0);
    break;
  case GIZMO_BAT_GOOD:
    pixels.setPixelColor(GIZMO_INDICATE_BAT, 255, 255, 0);
    break;
  case GIZMO_BAT_PASS:
    pixels.setPixelColor(GIZMO_INDICATE_BAT, 255, 165, 0);
    break;
  case GIZMO_BAT_DEAD:
    pixels.setPixelColor(GIZMO_INDICATE_BAT, 255, 0, 0);
    break;
  default:
    pixels.setPixelColor(GIZMO_INDICATE_BAT, 255, 0, 255);
    break;
  }
}

void StatusIndicators::doCfgSet() {
  if (toggleCfgAt < millis()) {
    toggleCfgAt = millis() + 125;
    toggleCfgCnt++;
    if (toggleCfgCnt % 2 == 0) {
      pixels.setPixelColor(GIZMO_INDICATE_NET, 0, 0, 0);
      pixels.setPixelColor(GIZMO_INDICATE_FIELD, 0, 0, 0);
    } else {
      switch (cfgStatus) {
      case CFG_NO_FILE:
        pixels.setPixelColor(GIZMO_INDICATE_NET, 255, 0, 0);
        pixels.setPixelColor(GIZMO_INDICATE_FIELD, 255, 0, 0);
        break;
      case CFG_BAD_PARSE:
        pixels.setPixelColor(GIZMO_INDICATE_NET, 255, 0, 255);
        pixels.setPixelColor(GIZMO_INDICATE_FIELD, 255, 0, 255);
        break;
      case CFG_BAD:
        pixels.setPixelColor(GIZMO_INDICATE_NET, 255, 255, 0);
        pixels.setPixelColor(GIZMO_INDICATE_FIELD, 255, 255, 0);
        break;
      }
    }
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.  Copied from Adafruit Pixel Library.
uint32_t StatusIndicators::wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return pixels.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return pixels.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return pixels.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
