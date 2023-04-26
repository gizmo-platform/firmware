#include "indicators.h"
#include "Arduino.h"

StatusIndicators::StatusIndicators(int pin, int count) {
  pixels = Adafruit_NeoPixel(count, pin, NEO_RGB + NEO_KHZ800);

  pixels.begin();
  pixels.setBrightness(20);
  pixels.show();
}

void StatusIndicators::Update() {
  doWifiSet();
  doFieldSet();
  pixels.show();
}

void StatusIndicators::SetWifiConnected(bool wifiState) {
  wifiConnected = wifiState;
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

void StatusIndicators::doWifiSet() {
  if (wifiConnected && ! ctrlConnected) {
    pixels.setPixelColor(BRI_INDICATE_NET, 255, 255, 0);
  } else if (wifiConnected && ctrlConnected) {
    pixels.setPixelColor(BRI_INDICATE_NET, 0, 255, 0);
  } else {
    pixels.setPixelColor(BRI_INDICATE_NET, 255, 0, 0);
  }
}

void StatusIndicators::doFieldSet() {
  if (! wifiConnected) {
    pixels.setPixelColor(BRI_INDICATE_FIELD, 0, 0, 0);
    return;
  } else if ( wifiConnected && ! ctrlConnected) {
    pixels.setPixelColor(BRI_INDICATE_FIELD, wheel((millis() / 5) % 255));
    return;
  }

  if (!fieldNum) return ;
  if (toggleFieldCnt > blinkFieldLen[fieldNum-1]-1) {
    toggleFieldCnt = 0;
  }
  if (toggleFieldAt < millis()) {
    if (toggleFieldCnt % 2 == 0) {
      pixels.setPixelColor(BRI_INDICATE_FIELD, 0, 0, 0);
    } else {
      doSetColorForFieldPos(fieldPos);
    }
    toggleFieldAt = millis() + blinkFieldNum[fieldNum-1][toggleFieldCnt];
    toggleFieldCnt++;
  }
}

void StatusIndicators::doSetColorForFieldPos(byte quad) {
  switch (quad) {
  case BRI_QUAD_RED:
    pixels.setPixelColor(BRI_INDICATE_FIELD, 255, 0, 0);
    break;
  case BRI_QUAD_BLUE:
    pixels.setPixelColor(BRI_INDICATE_FIELD, 0, 0, 255);
    break;
  case BRI_QUAD_GREEN:
    pixels.setPixelColor(BRI_INDICATE_FIELD, 0, 255, 0);
    break;
  case BRI_QUAD_YELLOW:
    pixels.setPixelColor(BRI_INDICATE_FIELD, 255, 255, 0);
    break;
  case BRI_QUAD_PRACTICE:
    pixels.setPixelColor(BRI_INDICATE_FIELD, 255, 255, 255);
    break;
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
