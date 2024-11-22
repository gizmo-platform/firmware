#ifndef GIZMO_H
#define GIZMO_H
#include "Arduino.h"

struct Config {
  bool loaded;

  int teamNumber;
  char hostname[32];

  String ds;
  String netSSID;
  String netPSK;
};

struct CState {
  byte Axis0;
  byte Axis1;
  byte Axis2;
  byte Axis3;
  byte Axis4;
  byte Axis5;

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
  byte Button10;
  byte Button11;
};

struct BoardState {
  int VBat;
  int WatchdogRemaining;
  int WifiReconnects;
  int RSSI;
  int FramesReceived;

  bool WatchdogOK;
  bool PwrBoard;
  bool PwrPico;
  bool PwrGPIO;
  bool PwrServo;
  bool PwrMainA;
  bool PwrMainB;
  bool PwrPixels;
};

enum CfgState {
  CFG_INIT,
  CFG_REQUEST,
  CFG_WAIT,
  CFG_LOAD,
  CFG_REBOOT,
};

enum NetState {
  NET_SEARCH,
  NET_CONNECT_WIFI,
  NET_CONNECT_WAIT_WIFI,
  NET_CONNECT_WAIT_ENET,
  NET_BIND,
  NET_RUNNING,
};

enum NetLink {
  NET_WIRED,
  NET_WIRELESS,
};

void ConfigureTeamNumber(int);
void ConfigureStatusIO(byte, byte, byte, byte, byte, byte, byte, byte);
void ConfigureBoardVoltageTuning(float, float);
void ConfigureUserReset(byte);
void ConfigureWiznetReset(byte);
void ConfigureDefaultBroker(String);
void GizmoSetup();
void GizmoTick();
void GizmoTick1();
void SetWifiNet(String, String);
void WireSetup(int, int);
#endif
