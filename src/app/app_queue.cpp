#include "app_queue.h"
#include "app_types.h"
#include "domain/fusion/fusion_types.h"
QueueHandle_t mbx_system_data = nullptr;
QueueHandle_t mbx_can_jog_cmd = nullptr;
QueueHandle_t fifo_motion_command_request = nullptr;
QueueHandle_t fifo_can_char_cmd = nullptr;
QueueHandle_t fifo_system_cmd = nullptr;
QueueHandle_t mbx_can_heading = nullptr;
QueueHandle_t mbx_can_sensor = nullptr;
QueueHandle_t mbx_can_magnetic = nullptr;
QueueHandle_t mbx_can_angular_velocity = nullptr;
QueueHandle_t mbx_can_encoder = nullptr;
QueueHandle_t mbx_gps_nmea_observation = nullptr;
QueueHandle_t mbx_gps_nav_pvt_observation = nullptr;
QueueHandle_t mbx_gps_local_observation = nullptr;
QueueHandle_t mbx_coordinate = nullptr;
QueueHandle_t mbx_navigation_progress = nullptr;
QueueHandle_t mbx_twe_telemetry = nullptr;
QueueHandle_t mbx_camera_data = nullptr;
QueueHandle_t mbx_flash_log = nullptr;
QueueHandle_t mbx_stuck_status = nullptr;
QueueHandle_t mbx_stuck_diagnostics = nullptr;
QueueHandle_t fifo_flash_debug_request = nullptr;
QueueHandle_t mbx_flash_debug_response = nullptr;
QueueHandle_t mbx_flash_nav_response = nullptr;
QueueHandle_t mbx_flash_status = nullptr;
QueueHandle_t fifo_map_update = nullptr;
SemaphoreHandle_t mutex_grid_map = nullptr;

bool appInitQueues() {
    mbx_system_data = xQueueCreate(1, sizeof(SystemData));
    mbx_can_jog_cmd = xQueueCreate(1, sizeof(JogData));
    fifo_motion_command_request =
        xQueueCreate(12, sizeof(MotionCommandRequest));

    fifo_can_char_cmd = xQueueCreate(10, sizeof(char));
    fifo_system_cmd = xQueueCreate(10, sizeof(SystemCmdType));

    mbx_can_heading = xQueueCreate(1, sizeof(Can::Data::Heading));
    mbx_can_sensor = xQueueCreate(1, sizeof(Can::Data::Sensor));
    mbx_can_magnetic = xQueueCreate(1, sizeof(Can::Data::MagneticField));
    mbx_can_angular_velocity =
        xQueueCreate(1, sizeof(Can::Data::AngularVelocity));
    mbx_can_encoder = xQueueCreate(1, sizeof(Can::Data::Encoder));
    mbx_gps_nmea_observation =
        xQueueCreate(1, sizeof(Gps::NmeaObservation));
    mbx_gps_nav_pvt_observation =
        xQueueCreate(1, sizeof(Gps::NavPvtObservation));
    mbx_gps_local_observation =
        xQueueCreate(1, sizeof(Domain::Fusion::GpsUpdate));
    mbx_coordinate = xQueueCreate(1, sizeof(Coordinate));
    mbx_navigation_progress =
        xQueueCreate(1, sizeof(NavigationProgress));
    mbx_twe_telemetry = xQueueCreate(1, sizeof(Twe::TelemetryFrame));
    mbx_camera_data = xQueueCreate(1, sizeof(Rasp::CameraData));
    mbx_flash_log = xQueueCreate(1, sizeof(Flash::LogFrame));
    mbx_stuck_status = xQueueCreate(1, sizeof(StuckStatus));
    mbx_stuck_diagnostics = xQueueCreate(1, sizeof(StuckDiagnostics));
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
        mbx_can_heading != nullptr &&
        mbx_can_sensor != nullptr &&
        mbx_can_magnetic != nullptr &&
        mbx_can_angular_velocity != nullptr &&
        mbx_can_encoder != nullptr &&
        mbx_gps_nmea_observation != nullptr &&
        mbx_gps_nav_pvt_observation != nullptr &&
        mbx_gps_local_observation != nullptr &&
        mbx_coordinate != nullptr &&
        mbx_navigation_progress != nullptr &&
        mbx_twe_telemetry != nullptr &&
        mbx_camera_data != nullptr &&
        mbx_flash_log != nullptr &&
        mbx_stuck_status != nullptr &&
        mbx_stuck_diagnostics != nullptr &&
        fifo_flash_debug_request != nullptr &&
        mbx_flash_debug_response != nullptr &&
        mbx_flash_nav_response != nullptr &&
        mbx_flash_status != nullptr &&
        fifo_map_update != nullptr &&
        mutex_grid_map != nullptr;
}
