#include "config.h"
#include "gizmo.h"

void setup() {
  ConfigureStatusIO(
                    GIZMO_HW_ADC_BOARD_VOLTAGE,
                    GIZMO_HW_PWR_BOARD,
                    GIZMO_HW_PWR_PICO,
                    GIZMO_HW_PWR_GPIO,
                    GIZMO_HW_PWR_SERVO,
                    GIZMO_HW_PWR_MAIN_A,
                    GIZMO_HW_PWR_MAIN_B,
                    GIZMO_HW_PWR_PIXELS
                    );
  ConfigureWiznetReset(GIZMO_HW_ENET_RST);
  ConfigureBoardVoltageTuning(GIZMO_HW_ADC_BOARD_VOLTAGE_M, GIZMO_HW_ADC_BOARD_VOLTAGE_B);
  GizmoSetup();
}

void loop() {
  GizmoTick();
}

void setup1() {
  WireSetup(GIZMO_HW_I2C_SDA, GIZMO_HW_I2C_SCL);
}

void loop1() {
  GizmoTick1();
  delay(20);
}
