#pragma once

constexpr int RASP_EN = 2;
constexpr int RASP_UART_RX = 39; // P2E_UART: Raspberry Pi -> ESP32
constexpr int RASP_UART_TX = 40; // E2P_UART: ESP32 -> Raspberry Pi
constexpr int RASP_CAMERA_READY = 41;
constexpr int RASP_HEARTBEAT = 42;

constexpr int CAN_RX = 4;
constexpr int CAN_TX = 5;

constexpr int TWE_RX = 6;
constexpr int TWE_TX = 7;
constexpr int TWE_EN = 14;

constexpr int FLASH_CS = 10;
constexpr int FLASH_MOSI = 11;
constexpr int FLASH_MISO = 13;
constexpr int FLASH_SCLK = 12;

constexpr int BOOT_MANUAL = 8;
constexpr int BOOT_DEBUG = 9;

constexpr int LED_ERROR = 1;
constexpr int LED_FLASH = 38;
constexpr int LED_BOOT = 47;
constexpr int LED_STATE = 48;

constexpr int GPS_EN = 21;
constexpr int GPS_TX = 16;
constexpr int GPS_RX = 15;

constexpr int I2C_SCL = 17;
constexpr int I2C_SDA = 18;
