#include <Arduino.h>
#include <WiFi.h>

#include "app/app_queue.h"
#include "app/tasks.h"

namespace {

constexpr uint32_t SERIAL_BAUD = 115200;

void stopOnStartupError(const char* message)
{
    Serial.println(message);
    while (true) {
        delay(1000);
    }
}

void createTaskChecked(
    TaskFunction_t task_function,
    const char* name,
    uint32_t stack_size_bytes,
    UBaseType_t priority,
    BaseType_t core)
{
    const BaseType_t result = xTaskCreatePinnedToCore(
        task_function,
        name,
        stack_size_bytes,
        nullptr,
        priority,
        nullptr,
        core);

    if (result != pdPASS) {
        Serial.print("Task create failed: ");
        stopOnStartupError(name);
    }
}

} // namespace

void setup()
{
    Serial.begin(SERIAL_BAUD);

    // この制御基板では無線LANを使用しないため、起動直後に明示的に停止する。
    WiFi.mode(WIFI_OFF);

    // 全タスクを開始する前に、共有mailbox・FIFO・mutexを生成する。
    if (!appInitQueues()) {
        stopOnStartupError("Queue or mutex create failed");
    }

    /*
     * Core 1: 状態判定と走行制御
     * Stateを最優先にし、BootModeと初期状態を他タスクへ公開する。
     */
    createTaskChecked(taskState,      "State",      4096, 5, 1);
    createTaskChecked(taskCan,        "CAN",        6144, 4, 1);
    createTaskChecked(taskSensor,     "Sensor",     6144, 4, 1);
    // 5-state Localization EKF. Gyro integration and delayed-GNSS replay use
    // fixed-size buffers and Joseph covariance updates.
    createTaskChecked(taskLocalization, "Localization", 12288, 4, 1);
    createTaskChecked(taskMotionArbiter, "MotionArb", 4096, 4, 1);
    createTaskChecked(taskNav,        "Nav",       12288, 3, 1);
    createTaskChecked(taskStuck,      "Stuck",      6144, 3, 1);

    /*
     * Core 0: 外部通信、Flash、ログ、表示
     * Flashの2KB packed-map snapshotとDEBUG応答を考慮してstackを確保する。
     */
    createTaskChecked(taskFlash, "Flash", 8192, 3, 0);
    createTaskChecked(taskGps,   "GPS",   6144, 2, 0);
    createTaskChecked(taskRasp,  "Rasp",  8192, 2, 0);
    createTaskChecked(taskTwe,   "TWE",   6144, 2, 0);
    createTaskChecked(taskLog,   "Log",   6144, 2, 0);
    createTaskChecked(taskDebug, "Debug", 4096, 1, 0);
    createTaskChecked(taskLed,   "LED",   3072, 1, 0);
}

void loop()
{
    // ArduinoのloopTaskは不要なので、自身を削除してCPU時間を各taskへ渡す。
    vTaskDelete(nullptr);
}
