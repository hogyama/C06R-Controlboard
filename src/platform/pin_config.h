#pragma once

constexpr int SEQUENCE = 1;

constexpr int RASP_EN = 2;

constexpr int RASP_UART_TX = 41; // Raspberry Pi TX 10 -> ESP32 RX GPIO41
constexpr int RASP_CAMERA_READY = 39; // Raspberry Pi Camera Ready GPIO8 -> ESP32 GPIO39
constexpr int RASP_HANDSHAKE = 40; // Raspberry Pi Handshake GPIO9 <- ESP32 GPIO40

constexpr int RASP_IMAGE_MODE = 7; // ESP32 -> Raspberry Pi GPIO4 (active HIGH)

constexpr int CAN_TX = 4;
constexpr int CAN_RX = 9;

constexpr int TWE_EN = 5;
constexpr int TWE_TX = 17;
constexpr int TWE_RX = 18;

constexpr int FLASH_CS = 10;
constexpr int FLASH_MOSI = 11;
constexpr int FLASH_MISO = 13;
constexpr int FLASH_SCLK = 12;

constexpr int DEBUG_START = 8;

constexpr int LED_MAP = 47;
constexpr int LED_STATE = 48;

constexpr int GPS_EN = 21;
constexpr int GPS_TX = 16;
constexpr int GPS_RX = 15;
