#include "tasks.h"
#include "platform/board_config.h"
#include "platform/pin_config.h"
#include "app_types.h"
#include "app_queue.h"
#include "app_context.h"
#include "service/Can/srv_can.h"

SystemCmdType convertCanEvent(Can::Data::EventBytes event)
{
    switch (event) {
        case Can::Data::EventBytes::StuckResolved:
            return SystemCmdType::StuckResolved;

        case Can::Data::EventBytes::UprightRecoveryFailed:
            return SystemCmdType::UprightRecoveryFailed;

        case Can::Data::EventBytes  ::SeparationFinished:
            return SystemCmdType::SeparationFinished;

        case Can::Data::EventBytes::AscendDetected:
            return SystemCmdType::AscentDetected;

        case Can::Data::EventBytes::LandingDetected:
            return SystemCmdType::LandingDetected;

        default:
            return SystemCmdType::None;
    }
}

void taskCan(void *pvParameters) {
    TickType_t last_wake = xTaskGetTickCount();
    constexpr uint32_t CAN_RECEIVE_PERIOD_MS = 2;
    constexpr uint32_t CAN_SEND_PERIOD_MS = 10;
    const TickType_t period = pdMS_TO_TICKS(CAN_RECEIVE_PERIOD_MS);
    if (!can.begin(CAN_RX, CAN_TX)) {
        // 初期化失敗後に未初期化ドライバへアクセスしない。再起動までCAN処理を停止する。
        Serial.println("CAN initialization failed");
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    uint32_t last_send_ms = millis() - CAN_SEND_PERIOD_MS;
    while (true){
        vTaskDelayUntil(&last_wake, period);

        // 受信は2msごとに回収し、センサー・エンコーダの取りこぼしを抑える。
        can.poll();

        SystemData status = {};
        if (xQueuePeek(mbx_system_data, &status, 0) != pdTRUE) continue;
        BootMode boot_mode = status.boot_mode;
        SystemState state = status.state;

        const uint32_t now_ms = millis();
        const bool send_due =
            static_cast<uint32_t>(now_ms - last_send_ms) >=
            CAN_SEND_PERIOD_MS;
        if (send_due) {
            last_send_ms = now_ms;
            switch(boot_mode) {
            case BootMode::SEQUENCE:
                do{
                // jog送信するのは、GPS_NAV,CAMERA_NAVのとき
                if(state == SystemState::STATE_GPS_NAV || state == SystemState::STATE_CAMERA_NAV){
                    JogData jog = {};
                    if(xQueuePeek(mbx_can_jog_cmd, &jog, 0) == pdTRUE){
                        const uint32_t age_ms = static_cast<uint32_t>(millis() - jog.timestamp_ms);
                        if(age_ms >= jog.duration_ms){
                            xQueueReset(mbx_can_jog_cmd);
                            // Jog期限切れ時はモータ基板の監視タイムアウトを待たず即停止する。
                            Can::Command::Velocity stop_cmd{};
                            can.send(stop_cmd);
                        }else{
                            Can::Command::Velocity cmd{};
                            cmd.velocity_mm_s = jog.velocity_mm_s;
                            cmd.omega_rad_s = jog.omega_rad_s;
                            can.send(cmd);
                        }
                    }
                }
                // ActionはFIFOから取り出し、一度だけCAN送信する。
                Can::Command::Action action_cmd{};
                if(xQueuePeek(fifo_can_char_cmd, &action_cmd.type, 0) == pdTRUE &&
                   can.send(action_cmd)) {
                    // 送信成功後にだけ削除し、一時的な送信キュー満杯では次周期に再試行する。
                    xQueueReceive(fifo_can_char_cmd, &action_cmd.type, 0);
                }
                }while(0);
                break;
            case BootMode::MANUAL:
                // 全てのCANを送信
                do{
                JogData jog = {};
                if(xQueuePeek(mbx_can_jog_cmd, &jog, 0) == pdTRUE){
                        const uint32_t age_ms = static_cast<uint32_t>(millis() - jog.timestamp_ms);
                        if(age_ms >= jog.duration_ms){
                            xQueueReset(mbx_can_jog_cmd);
                            // MANUALでも指定時間が終わった周期に停止指令を送る。
                            Can::Command::Velocity stop_cmd{};
                            can.send(stop_cmd);
                        }else{
                        Can::Command::Velocity cmd{};
                        cmd.velocity_mm_s = jog.velocity_mm_s;
                        cmd.omega_rad_s = jog.omega_rad_s;
                        can.send(cmd);
                    }
                }
                Can::Command::Action action_cmd{};
                if(xQueuePeek(fifo_can_char_cmd, &action_cmd.type, 0) == pdTRUE &&
                   can.send(action_cmd)) {
                    xQueueReceive(fifo_can_char_cmd, &action_cmd.type, 0);
                }
                }while(0);
                break;
            case BootMode::DEBUG:
                do{
                // jog送信は行わない
                JogData jog = {};
                if(xQueuePeek(mbx_can_jog_cmd, &jog, 0) == pdTRUE){
                    xQueueReset(mbx_can_jog_cmd);
                }
                // DEBUGでは状態遷移FIFOを通さず、Reset/SequenceStartだけを一度送る。
                Can::Command::Action action_cmd{};
                if(xQueuePeek(fifo_can_char_cmd, &action_cmd.type, 0) == pdTRUE){
                    if(action_cmd.type == Can::Command::Reset){
                        if (can.send(action_cmd)) {
                            xQueueReceive(fifo_can_char_cmd, &action_cmd.type, 0);
                        }
                    } else if(action_cmd.type == Can::Command::SequenceStart){
                        if (can.send(action_cmd)) {
                            xQueueReceive(fifo_can_char_cmd, &action_cmd.type, 0);
                        }
                    } else {
                        // DEBUGで許可されない古いActionはFIFO先頭を塞がないよう破棄する。
                        xQueueReceive(fifo_can_char_cmd, &action_cmd.type, 0);
                    }
                }
                }while(0);
                break;
            }
        }

        Can::Data::Heading heading_data{};
        Can::Data::Sensor sensor_data{};
        Can::Data::MagneticField magnetic_data{};
        Can::Data::AngularVelocity angular_velocity_data{};
        Can::Data::Encoder encoder_data{};
        Can::Data::Event event_data{};

        if (boot_mode == BootMode::DEBUG) {
            // DEBUGではCANを動かすが、受信値を共有mailboxや状態遷移へ渡さない。
            while (can.read(&heading_data)) {}
            while (can.read(&sensor_data)) {}
            while (can.read(&magnetic_data)) {}
            while (can.read(&angular_velocity_data)) {}
            while (can.read(&encoder_data)) {}
            while (can.readEvent().bytes != Can::Data::EventBytes::None) {}
            continue;
        }

        if(can.read(&heading_data)){
            xQueueOverwrite(mbx_can_heading, &heading_data);
        }
        if(can.read(&sensor_data)){
            xQueueOverwrite(mbx_can_sensor, &sensor_data);
        }
        if(can.read(&magnetic_data)){
            xQueueOverwrite(mbx_can_magnetic, &magnetic_data);
        }
        if(can.read(&angular_velocity_data)){
            xQueueOverwrite(
                mbx_can_angular_velocity,
                &angular_velocity_data
            );
        }
        if(can.read(&encoder_data)){
            xQueueOverwrite(mbx_can_encoder, &encoder_data);
        }
        event_data = can.readEvent();
        if(event_data.bytes != Can::Data::EventBytes::None){
            SystemCmdType cmd = convertCanEvent(event_data.bytes);
            if(cmd != SystemCmdType::None){
                xQueueSend(fifo_system_cmd, &cmd, portMAX_DELAY);
            }
        }
    }
}
