#include "Arduino.h"

#define BRI_PUBLIC_TEAM_NUMBER "1234"
#define BRI_PUBLIC_MQTT_BROKER "192.168.16.10"

// Pin assignments that are hardware specific.
const byte BRI_HW_ADC_BOARD_VOLTAGE = 28;
const byte BRI_HW_PWR_BOARD = 6;
const byte BRI_HW_PWR_PICO = 7;
const byte BRI_HW_PWR_GPIO = 8;
const byte BRI_HW_PWR_MAIN_A = 9;
const byte BRI_HW_PWR_MAIN_B = 10;
const byte BRI_HW_STATUS_NEOPIXELS_PIN = 15;
const byte BRI_HW_STATUS_NEOPIXELS_CNT = 3;
const byte BRI_HW_USER_RESET = 22;
const byte BRI_HW_I2C_SDA = 18;
const byte BRI_HW_I2C_SCL = 19;
const byte BRI_HW_PRACTICE_MODE = 0;

// Slow but standard serial speed
const int BRI_HW_SERIAL_SPEED = 9600;

// Watchdog bites at 15s without food.
const int BRI_PUBLIC_MILLIS_WATCHDOG = 15000;

// set the next two in secret.h if you like
#define BRI_PRIVATE_WIFI_SSID ""
#define BRI_PRIVATE_WIFI_PSK ""
