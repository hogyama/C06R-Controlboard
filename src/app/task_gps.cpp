#include "tasks.h"
#include "platform/board_config.h"
#include "platform/pin_config.h"
#include "app_types.h"
#include "app_queue.h"
#include "app_context.h"
#include "service/Gps/srv_gps.h"

namespace {

void holdGpsPowerOff()
{
    // DEBUGではUARTピンへ触れず、GPS電源だけを確実に切る。
    pinMode(GPS_EN, OUTPUT);
    digitalWrite(GPS_EN, LOW);
}

} // namespace

void taskGps(void *pvParameters) {
    const TickType_t period = pdMS_TO_TICKS(100);

    // BootMode確定前はUARTを開始せず、GPSを確実に無給電状態へ置く。
    holdGpsPowerOff();
    SystemData startup_status{};
    xQueuePeek(mbx_system_data, &startup_status, portMAX_DELAY);

    // BootModeは起動時固定。DEBUGではgps.init()を一度も呼ばない。
    if (startup_status.boot_mode == BootMode::DEBUG) {
        Gps::NmeaObservation nmea{};
        Gps::NavPvtObservation nav_pvt{};
        xQueueOverwrite(mbx_gps_nmea_observation, &nmea);
        xQueueOverwrite(mbx_gps_nav_pvt_observation, &nav_pvt);
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    const bool gps_initialized = gps.init(
        GPS_RX,
        GPS_TX,
        GPS_EN,
        GPS_UART_NUM,
        GPS_UART_BAUD
    );
    const bool gps_ready = gps_initialized;

    bool gps_power_on = false;
    bool gps_should_power_on = false;
    TickType_t last_wake = xTaskGetTickCount();
    while (true){
        xTaskDelayUntil(&last_wake, period);

        if(!gps_ready){
            Gps::NmeaObservation nmea{};
            Gps::NavPvtObservation nav_pvt{};
            xQueueOverwrite(mbx_gps_nmea_observation, &nmea);
            xQueueOverwrite(mbx_gps_nav_pvt_observation, &nav_pvt);
            continue;
        }

        SystemData status = {};
        if(xQueuePeek(mbx_system_data, &status, 0) != pdTRUE) continue;
        switch(status.boot_mode)
        {
            case BootMode::SEQUENCE:
                gps_should_power_on = (status.state != SystemState::STATE_GOAL) 
                                    && (status.state != SystemState::STATE_PRELAUNCH);
                break;
            case BootMode::MANUAL:
                // MANUALでは状態によらずGPSを動作させ、測位状態を常に確認できるようにする。
                gps_should_power_on = true;
                break;
            case BootMode::DEBUG:
                // DEBUGでは回り込み給電を防ぐため、UARTを初期化せずGPSも無給電に保つ。
                gps_should_power_on = false;
                break;
        }

        if (gps_should_power_on && !gps_power_on) {
            gps_power_on = gps.powerOn();
        } else if (!gps_should_power_on && gps_power_on) {
            gps.powerOff();
            gps_power_on = false;
        }
        if(gps_power_on){
            gps.poll();
            Gps::NmeaObservation nmea{};
            if (gps.getNmeaObservation(&nmea)) {
                xQueueOverwrite(mbx_gps_nmea_observation, &nmea);
            }
            Gps::NavPvtObservation nav_pvt{};
            if (gps.getNavPvtObservation(&nav_pvt)) {
                xQueueOverwrite(mbx_gps_nav_pvt_observation, &nav_pvt);
            }
        }else{
            Gps::NmeaObservation nmea{};
            Gps::NavPvtObservation nav_pvt{};
            xQueueOverwrite(mbx_gps_nmea_observation, &nmea);
            xQueueOverwrite(mbx_gps_nav_pvt_observation, &nav_pvt);
        }
    }
}
