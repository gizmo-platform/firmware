#include "Arduino.h"

// ETHERNET MAGIC PINS
#ifdef  GIZMO_ENET_OFFBOARD
//ON_HEADER
const byte GIZMO_HW_ENET_CS = 13;
const byte GIZMO_HW_ENET_RST = 17;
const byte GIZMO_HW_ENET_INT = -1;
#else
//BUILT_IN
const byte GIZMO_HW_ENET_CS = 1;
const byte GIZMO_HW_ENET_RST = 20;
const byte GIZMO_HW_ENET_INT = -1;
#endif
//

// Pin assignments that are hardware specific.
const byte GIZMO_HW_STATUS_NEOPIXELS_PIN = 15;
const byte GIZMO_HW_STATUS_NEOPIXELS_CNT = 3;
const byte GIZMO_HW_USER_RESET = 22;
const byte GIZMO_HW_I2C_SDA = 18;
const byte GIZMO_HW_I2C_SCL = 19;
const byte GIZMO_HW_ADC_BOARD_VOLTAGE = 28;

#ifdef GIZMO_VERSION_R3B
#define GIZMO_HW_VERSION "GIZMO_V00_R3B"
const float GIZMO_HW_ADC_BOARD_VOLTAGE_M = 0.008833;
const float GIZMO_HW_ADC_BOARD_VOLTAGE_B = 0.3017;
const byte GIZMO_HW_PWR_BOARD = 6;
const byte GIZMO_HW_PWR_PICO = 7;
const byte GIZMO_HW_PWR_GPIO = 8;
const byte GIZMO_HW_PWR_SERVO = -1;
const byte GIZMO_HW_PWR_MAIN_A = 9;
const byte GIZMO_HW_PWR_MAIN_B = 10;
const byte GIZMO_HW_PWR_PIXELS = -1;

#elifdef GIZMO_VERSION_R4B
#define GIZMO_HW_VERSION "GIZMO_V00_R4B"
const float GIZMO_HW_ADC_BOARD_VOLTAGE_M = 0; // Known non-functional in this version
const float GIZMO_HW_ADC_BOARD_VOLTAGE_B = 0;
const byte GIZMO_HW_PWR_BOARD = 3;
const byte GIZMO_HW_PWR_PICO = 6;
const byte GIZMO_HW_PWR_GPIO = 7;
const byte GIZMO_HW_PWR_SERVO = -1;
const byte GIZMO_HW_PWR_MAIN_A = 8;
const byte GIZMO_HW_PWR_MAIN_B = 9;
const byte GIZMO_HW_PWR_PIXELS = 10;

#elifdef GIZMO_VERSION_R6E
#define GIZMO_HW_VERSION "GIZMO_V00_R6E"
const float GIZMO_HW_ADC_BOARD_VOLTAGE_M = 0.011499;
const float GIZMO_HW_ADC_BOARD_VOLTAGE_B = 0.2722;
const byte GIZMO_HW_PWR_BOARD = 4;
const byte GIZMO_HW_PWR_PICO = 5;
const byte GIZMO_HW_PWR_GPIO = 6;
const byte GIZMO_HW_PWR_SERVO = 7;
const byte GIZMO_HW_PWR_MAIN_A = 8;
const byte GIZMO_HW_PWR_MAIN_B = 9;
const byte GIZMO_HW_PWR_PIXELS = 10;

#else
#define GIZMO_HW_VERSION "GIZMO_UNKNOWN"
const float GIZMO_HW_ADC_BOARD_VOLTAGE_M = 1;
const float GIZMO_HW_ADC_BOARD_VOLTAGE_B = 0;
const byte GIZMO_HW_PWR_BOARD = -1;
const byte GIZMO_HW_PWR_PICO = -1;
const byte GIZMO_HW_PWR_GPIO = -1;
const byte GIZMO_HW_PWR_SERVO = -1;
const byte GIZMO_HW_PWR_MAIN_A = -1;
const byte GIZMO_HW_PWR_MAIN_B = -1;
const byte GIZMO_HW_PWR_PIXELS = -1;
#endif

// Slow but standard serial speed
const int GIZMO_HW_SERIAL_SPEED = 9600;

// Watchdog bites at 15s without food.
const int GIZMO_PUBLIC_MILLIS_WATCHDOG = 15000;
