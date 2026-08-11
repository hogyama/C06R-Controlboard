#pragma once
#include <Arduino.h>
#include <Freertos.h>
#include "service/Can/srv_can.h"
#include "service/Gps/srv_gps.h"
#include "service/Twe/srv_twe.h"
#include "service/Rasp/srv_rasp.h"
#include "service/Flash/srv_flash.h"
#include "domain/sensor/sensor_types.h"
#include "domain/localization/localization_types.h"

bool appInitQueues();
void pushGyroscopeRing(
    QueueHandle_t ring,
    const Sensor::GyroscopeData& sample);

// systemの状態を管理するQueue
extern QueueHandle_t mbx_system_data;

// can出力用のQueue
extern QueueHandle_t mbx_can_jog_cmd;
extern QueueHandle_t fifo_motion_command_request;
extern QueueHandle_t fifo_can_char_cmd;

// 内部状態遷移入力用のQueue
extern QueueHandle_t fifo_system_cmd;

// can入力用の最新データQueue
extern QueueHandle_t mbx_board_acceleration;
extern QueueHandle_t mbx_can_acceleration;
extern QueueHandle_t mbx_acceleration;
extern QueueHandle_t mbx_board_gyroscope;
extern QueueHandle_t mbx_can_gyroscope;
extern QueueHandle_t mbx_gyroscope;
extern QueueHandle_t fifo_board_gyroscope;
extern QueueHandle_t fifo_can_gyroscope;
extern QueueHandle_t mbx_board_magnetic;
extern QueueHandle_t mbx_can_magnetic;
extern QueueHandle_t mbx_magnetic;
extern QueueHandle_t mbx_pressure;
extern QueueHandle_t mbx_sensor_acquisition_stats;
extern QueueHandle_t mbx_can_encoder;
extern QueueHandle_t fifo_can_encoder;
// GPS driver raw observation and Domain-converted local observation.
extern QueueHandle_t mbx_gps_nav_pvt_observation;
extern QueueHandle_t mbx_gps_local_observation;

// coordinate出力用のQueue
extern QueueHandle_t mbx_coordinate;
extern QueueHandle_t mbx_localization_estimate;
extern QueueHandle_t fifo_localization_debug_command;
extern QueueHandle_t mbx_localization_debug_status;
extern QueueHandle_t mbx_navigation_progress;

// twe出力用のtelemetryQueue
extern QueueHandle_t mbx_twe_telemetry;

// camera出力用のQueue
extern QueueHandle_t mbx_camera_data;

// taskLogが作る最新Flashログと、taskStuckが作る最新判定結果
extern QueueHandle_t mbx_flash_log;
extern QueueHandle_t mbx_stuck_status;
extern QueueHandle_t mbx_stuck_diagnostics;
extern QueueHandle_t mbx_flip_status;
extern QueueHandle_t fifo_stuck_board_gyroscope;
extern QueueHandle_t fifo_stuck_can_gyroscope;

// Flash要求FIFO、要求元別の応答Mailbox、通常時を含むFlash状態共有
extern QueueHandle_t fifo_flash_debug_request;
extern QueueHandle_t mbx_flash_debug_response;
extern QueueHandle_t mbx_flash_nav_response;
extern QueueHandle_t mbx_flash_status;

// taskStuckからtaskNavへ地図更新を依頼するFIFO
extern QueueHandle_t fifo_map_update;

// taskNavの地図更新とtaskFlashのsnapshot取得を排他する
extern SemaphoreHandle_t mutex_grid_map;
