#include "app_queue.h"
#include "app_types.h"
#include "domain/localization/localization_types.h"
#include "domain/motion/flip_detector.h"
QueueHandle_t mbx_system_data = nullptr;
QueueHandle_t mbx_can_jog_cmd = nullptr;
QueueHandle_t fifo_motion_command_request = nullptr;
QueueHandle_t fifo_can_char_cmd = nullptr;
QueueHandle_t fifo_system_cmd = nullptr;
QueueHandle_t mbx_board_acceleration = nullptr;
QueueHandle_t mbx_can_acceleration = nullptr;
QueueHandle_t mbx_acceleration = nullptr;
QueueHandle_t mbx_board_gyroscope = nullptr;
QueueHandle_t mbx_can_gyroscope = nullptr;
QueueHandle_t mbx_gyroscope = nullptr;
QueueHandle_t fifo_board_gyroscope = nullptr;
QueueHandle_t fifo_can_gyroscope = nullptr;
QueueHandle_t mbx_board_magnetic = nullptr;
QueueHandle_t mbx_can_magnetic = nullptr;
QueueHandle_t mbx_magnetic = nullptr;
QueueHandle_t mbx_pressure = nullptr;
QueueHandle_t mbx_sensor_acquisition_stats = nullptr;
QueueHandle_t mbx_can_encoder = nullptr;
QueueHandle_t fifo_can_encoder = nullptr;
QueueHandle_t mbx_gps_nav_pvt_observation = nullptr;
QueueHandle_t mbx_gps_local_observation = nullptr;
QueueHandle_t mbx_coordinate = nullptr;
QueueHandle_t mbx_localization_estimate = nullptr;
QueueHandle_t fifo_localization_debug_command = nullptr;
QueueHandle_t mbx_localization_debug_status = nullptr;
QueueHandle_t mbx_navigation_progress = nullptr;
QueueHandle_t mbx_twe_telemetry = nullptr;
QueueHandle_t mbx_camera_data = nullptr;
QueueHandle_t mbx_flash_log = nullptr;
QueueHandle_t mbx_stuck_status = nullptr;
QueueHandle_t mbx_stuck_diagnostics = nullptr;
QueueHandle_t mbx_flip_status = nullptr;
QueueHandle_t fifo_stuck_board_gyroscope = nullptr;
QueueHandle_t fifo_stuck_can_gyroscope = nullptr;
QueueHandle_t fifo_flash_debug_request = nullptr;
QueueHandle_t mbx_flash_debug_response = nullptr;
QueueHandle_t mbx_flash_nav_response = nullptr;
QueueHandle_t mbx_flash_status = nullptr;
QueueHandle_t fifo_map_update = nullptr;
SemaphoreHandle_t mutex_grid_map = nullptr;

void pushGyroscopeRing(
    QueueHandle_t ring,
    const Sensor::GyroscopeData& sample)
{
    if (ring == nullptr || xQueueSend(ring, &sample, 0) == pdTRUE) return;
    Sensor::GyroscopeData discarded{};
    xQueueReceive(ring, &discarded, 0);
    xQueueSend(ring, &sample, 0);
}

bool appInitQueues() {
    mbx_system_data = xQueueCreate(1, sizeof(SystemData));
    mbx_can_jog_cmd = xQueueCreate(1, sizeof(JogData));
    fifo_motion_command_request =
        xQueueCreate(12, sizeof(MotionCommandRequest));

    fifo_can_char_cmd = xQueueCreate(10, sizeof(char));
    fifo_system_cmd = xQueueCreate(10, sizeof(SystemCmdType));

    mbx_board_acceleration =
        xQueueCreate(1, sizeof(Sensor::AccelerometerData));
    mbx_can_acceleration =
        xQueueCreate(1, sizeof(Sensor::AccelerometerData));
    mbx_acceleration = xQueueCreate(1, sizeof(Sensor::AccelerometerData));
    mbx_board_gyroscope =
        xQueueCreate(1, sizeof(Sensor::GyroscopeData));
    mbx_can_gyroscope =
        xQueueCreate(1, sizeof(Sensor::GyroscopeData));
    mbx_gyroscope = xQueueCreate(1, sizeof(Sensor::GyroscopeData));
    fifo_board_gyroscope = xQueueCreate(256, sizeof(Sensor::GyroscopeData));
    fifo_can_gyroscope = xQueueCreate(64, sizeof(Sensor::GyroscopeData));
    fifo_stuck_board_gyroscope =
        xQueueCreate(128, sizeof(Sensor::GyroscopeData));
    fifo_stuck_can_gyroscope =
        xQueueCreate(32, sizeof(Sensor::GyroscopeData));
    mbx_board_magnetic = xQueueCreate(1, sizeof(Sensor::MagneticData));
    mbx_can_magnetic = xQueueCreate(1, sizeof(Sensor::MagneticData));
    mbx_magnetic = xQueueCreate(1, sizeof(Sensor::MagneticData));
    mbx_pressure = xQueueCreate(1, sizeof(Sensor::PressureData));
    mbx_sensor_acquisition_stats =
        xQueueCreate(1, sizeof(Sensor::AcquisitionStats));
    mbx_can_encoder = xQueueCreate(1, sizeof(Can::Data::Encoder));
    fifo_can_encoder = xQueueCreate(16, sizeof(Can::Data::Encoder));
    mbx_gps_nav_pvt_observation =
        xQueueCreate(1, sizeof(Gps::NavPvtObservation));
    mbx_gps_local_observation =
        xQueueCreate(1, sizeof(Domain::Localization::GpsObservation));
    mbx_coordinate = xQueueCreate(1, sizeof(Coordinate));
    mbx_localization_estimate = xQueueCreate(
        1, sizeof(Domain::Localization::LocalizationEstimate));
    fifo_localization_debug_command = xQueueCreate(
        4, sizeof(LocalizationDebugCommand));
    mbx_localization_debug_status = xQueueCreate(
        1, sizeof(LocalizationDebugStatus));
    mbx_navigation_progress =
        xQueueCreate(1, sizeof(NavigationProgress));
    mbx_twe_telemetry = xQueueCreate(1, sizeof(Twe::TelemetrySnapshot));
    mbx_camera_data = xQueueCreate(1, sizeof(Rasp::CameraData));
    mbx_flash_log = xQueueCreate(1, sizeof(Flash::LogFrame));
    mbx_stuck_status = xQueueCreate(1, sizeof(StuckStatus));
    mbx_stuck_diagnostics = xQueueCreate(1, sizeof(StuckDiagnostics));
    mbx_flip_status = xQueueCreate(1, sizeof(Domain::Motion::FlipResult));
    fifo_flash_debug_request = xQueueCreate(2, sizeof(FlashDebugRequest));
    mbx_flash_debug_response = xQueueCreate(1, sizeof(FlashDebugResponse));
    mbx_flash_nav_response = xQueueCreate(1, sizeof(FlashDebugResponse));
    mbx_flash_status = xQueueCreate(1, sizeof(FlashStatus));
    fifo_map_update = xQueueCreate(16, sizeof(MapUpdate));
    mutex_grid_map = xSemaphoreCreateMutex();

    return
        mbx_system_data != nullptr &&
        mbx_can_jog_cmd != nullptr &&
        fifo_motion_command_request != nullptr &&
        fifo_can_char_cmd != nullptr &&
        fifo_system_cmd != nullptr &&
        mbx_board_acceleration != nullptr &&
        mbx_can_acceleration != nullptr &&
        mbx_acceleration != nullptr &&
        mbx_board_gyroscope != nullptr &&
        mbx_can_gyroscope != nullptr &&
        mbx_gyroscope != nullptr &&
        fifo_board_gyroscope != nullptr &&
        fifo_can_gyroscope != nullptr &&
        fifo_stuck_board_gyroscope != nullptr &&
        fifo_stuck_can_gyroscope != nullptr &&
        mbx_board_magnetic != nullptr &&
        mbx_can_magnetic != nullptr &&
        mbx_magnetic != nullptr &&
        mbx_pressure != nullptr &&
        mbx_sensor_acquisition_stats != nullptr &&
        mbx_can_encoder != nullptr &&
        fifo_can_encoder != nullptr &&
        mbx_gps_nav_pvt_observation != nullptr &&
        mbx_gps_local_observation != nullptr &&
        mbx_coordinate != nullptr &&
        mbx_localization_estimate != nullptr &&
        fifo_localization_debug_command != nullptr &&
        mbx_localization_debug_status != nullptr &&
        mbx_navigation_progress != nullptr &&
        mbx_twe_telemetry != nullptr &&
        mbx_camera_data != nullptr &&
        mbx_flash_log != nullptr &&
        mbx_stuck_status != nullptr &&
        mbx_stuck_diagnostics != nullptr &&
        mbx_flip_status != nullptr &&
        fifo_flash_debug_request != nullptr &&
        mbx_flash_debug_response != nullptr &&
        mbx_flash_nav_response != nullptr &&
        mbx_flash_status != nullptr &&
        fifo_map_update != nullptr &&
        mutex_grid_map != nullptr;
}
