#include "config.h"
#include "secrets.h"
#include "gizmo.h"

void setup() {
  SetWifiNet(GIZMO_PRIVATE_WIFI_SSID, GIZMO_PRIVATE_WIFI_PSK);
  ConfigureStatusIO(
                    GIZMO_HW_ADC_BOARD_VOLTAGE,
                    GIZMO_HW_PWR_BOARD,
                    GIZMO_HW_PWR_PICO,
                    GIZMO_HW_PWR_GPIO,
                    GIZMO_HW_PWR_MAIN_A,
                    GIZMO_HW_PWR_MAIN_B
                    );
  ConfigureTeamNumber(GIZMO_PUBLIC_TEAM_NUMBER);
  ConfigureDefaultBroker(GIZMO_PUBLIC_MQTT_BROKER);
  GizmoSetup();
}

void loop() {
  GizmoTick();
}

void setup1() {
  WireSetup(GIZMO_HW_I2C_SDA, GIZMO_HW_I2C_SCL);
}
