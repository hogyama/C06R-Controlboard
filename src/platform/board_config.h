#pragma once
#include <Arduino.h>
#include <driver/uart.h>
constexpr int RASP_UART_NUM = UART_NUM_0;
constexpr int TWE_UART_NUM = UART_NUM_1;
constexpr int GPS_UART_NUM = UART_NUM_2;
constexpr int RASP_UART_BAUD = 1200;
constexpr int TWE_UART_BAUD = 115200;
constexpr int GPS_UART_BAUD = 9600;

// 次期センサ基板の予定周期。
// IMUは姿勢予測に使うため100 Hz、地磁気はYawの低周波補正用に20 Hzとする。
constexpr uint32_t IMU_PERIOD_MS = 10;
constexpr uint32_t MAGNETIC_PERIOD_MS = 50;
