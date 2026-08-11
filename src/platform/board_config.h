#pragma once
#include <Arduino.h>
#include <driver/uart.h>
constexpr int RASP_UART_NUM = UART_NUM_0;
constexpr int TWE_UART_NUM = UART_NUM_1;
constexpr int GPS_UART_NUM = UART_NUM_2;
constexpr int RASP_UART_BAUD = 115200;
constexpr int TWE_UART_BAUD = 115200;
constexpr int GPS_UART_BAUD = 9600;

constexpr uint32_t I2C_CLOCK_HZ = 400000;
constexpr uint8_t BMM350_I2C_ADDRESS = 0x14;
constexpr uint32_t IMU_TIMEOUT_MS = 100;
constexpr uint32_t MAGNETIC_TIMEOUT_MS = 250;

// 次期センサ基板の予定周期。
// IMUは姿勢予測に使うため100 Hz、地磁気はYawの低周波補正用に20 Hzとする。
// Local sensors are polled because their interrupt pins are not connected.
constexpr uint32_t SENSOR_POLL_PERIOD_MS = 1;
constexpr uint32_t MAGNETIC_POLL_PERIOD_MS = 5;
constexpr uint32_t SENSOR_STATS_PERIOD_MS = 100;

// Raspberry Pi camera parameters sent in the UART START command.
constexpr uint16_t CAMERA_MIN_TARGET_AREA_PX = 100;
constexpr uint16_t CAMERA_CONFIDENCE_FULL_AREA_PX = 5000;
constexpr uint16_t CAMERA_HORIZONTAL_FOV_DEG_X100 = 6220;
constexpr uint8_t CAMERA_HSV_LOW1[3] = {0, 120, 70};
constexpr uint8_t CAMERA_HSV_HIGH1[3] = {5, 255, 255};
constexpr uint8_t CAMERA_HSV_LOW2[3] = {175, 120, 70};
constexpr uint8_t CAMERA_HSV_HIGH2[3] = {180, 255, 255};
constexpr uint8_t CAMERA_BLUR_KERNEL_SIZE = 15;
constexpr uint8_t CAMERA_MORPH_KERNEL_SIZE = 5;
constexpr bool CAMERA_ROTATE_180 = true;
constexpr bool CAMERA_MIRROR_HORIZONTAL = false;
