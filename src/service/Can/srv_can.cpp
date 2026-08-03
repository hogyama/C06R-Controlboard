#include "srv_can.h"

namespace {

// ICMのX軸から機体前方軸への取付補正。実機校正時はこの値だけを調整する。
constexpr float HEADING_MOUNT_OFFSET_DEG = -90.0f;

float normalizeDegrees(float angle_deg)
{
    while (angle_deg < 0.0f) angle_deg += 360.0f;
    while (angle_deg >= 360.0f) angle_deg -= 360.0f;
    return angle_deg;
}

} // namespace

SrvCan::SrvCan() {
    heading_head = 0;
    sensor_head = 0;
    magnetic_head = 0;
    angular_velocity_head = 0;
    encoder_head = 0;
    event_head = 0;

    heading_tail = 0;
    sensor_tail = 0;
    magnetic_tail = 0;
    angular_velocity_tail = 0;
    encoder_tail = 0;
    event_tail = 0;

    heading_count = 0;
    sensor_count = 0;
    magnetic_count = 0;
    angular_velocity_count = 0;
    encoder_count = 0;
    event_count = 0;
}

bool SrvCan::begin(int rx, int tx) {
    can_setting_t setting;
    setting.baudRate = (long)500E3;
    setting.multiData_send  = true;
    setting.filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(); 
    if(can_create.begin(setting, rx, tx))
    {
        return false;
    }
    return true;
}

bool SrvCan::send(Can::Command::Velocity velocity) {
    uint32_t id = MAKE_CAN_ID(CAN_PRIO_HIGH, CAN_ADDR_MOTOR, CAN_SRC_CONTROL, CAN_TYPE_NORMAL);
    uint8_t data[8];
    // CAN仕様: 並進速度はm/s×100、旋回角速度はrad/s×100で格納する。
    int32_t velocity_m_s_1e2 = (int32_t)((velocity.velocity_mm_s / 1000.0f) * 100.0f);
    int32_t omega_rad_s_1e2 = (int32_t)(velocity.omega_rad_s * 100.0f);
    memcpy(data, &velocity_m_s_1e2, sizeof(int32_t));
    memcpy(data + sizeof(int32_t), &omega_rad_s_1e2, sizeof(int32_t));
    if(can_create.sendData(id, data, sizeof(data))) {
        return false;
    }
    return true;
}

bool SrvCan::send(Can::Command::Action cmd) {
    char c;
    switch(cmd.type) {
        case Can::Command::Reset:
            c = 'r';
            break;
        case Can::Command::SequenceStart:
            c = 'k';
            break;
        case Can::Command::NotifyGoal:
            c = 'g';
            break;
        case Can::Command::NotifySeparation:
            c = 't';
            break;
        case Can::Command::NotifyStuck:
            c = 's';
            break;
        case Can::Command::NotifyFlipped:
            c = 'S';
            break;
        case Can::Command::ConfirmUpright:
            c = 'U';
            break;
        case Can::Command::ServoUnlock:
            c = 'b';
            break;
        case Can::Command::ServoLock: 
            c = 'd';
            break;
        default:
            return false;
    }
    if(c == 'r' || c == 'k' || c == 'g') {
        uint32_t id = MAKE_CAN_ID(CAN_PRIO_CRITICAL, CAN_ADDR_BROADCAST, CAN_SRC_CONTROL, CAN_TYPE_NORMAL);
        uint8_t data[2];
        data[0] = (uint8_t)cmd.type_id;
        data[1] = (uint8_t)c;
        if(!can_create.sendData(id, data, sizeof(data))) {
            return true;
        }
    }else if(c == 't' || c == 'S' || c == 'U' || c == 'b' || c == 'd' || c == 's'){
        uint32_t id = MAKE_CAN_ID(CAN_PRIO_CRITICAL, CAN_ADDR_MOTOR, CAN_SRC_CONTROL, CAN_TYPE_NORMAL);
        uint8_t data[2];
        data[0] = (uint8_t)cmd.type_id;
        data[1] = (uint8_t)c;
        if(!can_create.sendData(id, data, sizeof(data))) {
            return true;
        }
    }
    return false;
}

void SrvCan::poll() {
    can_return_t ret;
    while(can_create.available()){
        if(!can_create.readWithDetail(&ret)){
            uint8_t prio, addr, src, type;
            PARSE_CAN_ID(ret.id, prio, addr, src, type);
            //センサー基板からの受信
            if(addr == CAN_ADDR_CONTROL && src == CAN_SRC_SENSOR){
                // 気圧は独立フレームなので、最新値を加速度と組み合わせる。
                static int32_t latest_pressure_pa = 0;
                static bool pressure_received = false;

                const uint32_t sensor_frame_ms = millis();

                switch(type){
                    case CAN_TYPE_HEADING:
                        if(ret.size == sizeof(float)){
                            if(heading_count >= HEADING_BUFFER_SIZE){
                                heading_head = (heading_head + 1) % HEADING_BUFFER_SIZE;
                                heading_count--;
                            }
                            float heading_theta;
                            memcpy(&heading_theta, ret.data, sizeof(float));
                            // 取付補正後、コンパス角から東0度・反時計回り正へ変換する。
                            heading_theta = normalizeDegrees(
                                90.0f -
                                normalizeDegrees(heading_theta + HEADING_MOUNT_OFFSET_DEG));
                            //radに変換
                            heading_theta = heading_theta * M_PI / 180.0f;
                            heading_buffer[heading_tail].heading = heading_theta;
                            heading_buffer[heading_tail].ts_ms = millis();
                            heading_tail = (heading_tail + 1) % HEADING_BUFFER_SIZE;
                            heading_count++;
                        }
                        break;
                    case CAN_TYPE_ACCELERATION_XYZ:
                        if(ret.size == sizeof(int16_t) * 3 &&
                           pressure_received){
                            int16_t acceleration_xyz_mg[3] = {};
                            memcpy(
                                acceleration_xyz_mg,
                                ret.data,
                                sizeof(acceleration_xyz_mg));
                            if(sensor_count >= SENSOR_BUFFER_SIZE){
                                sensor_head =
                                    (sensor_head + 1) % SENSOR_BUFFER_SIZE;
                                sensor_count--;
                            }
                            sensor_buffer[sensor_tail] = {
                                acceleration_xyz_mg[0] * 0.001f,
                                acceleration_xyz_mg[1] * 0.001f,
                                acceleration_xyz_mg[2] * 0.001f,
                                latest_pressure_pa,
                                sensor_frame_ms
                            };
                            sensor_tail =
                                (sensor_tail + 1) % SENSOR_BUFFER_SIZE;
                            sensor_count++;
                        }
                        break;
                    case CAN_TYPE_MAGNETIC_XYZ:
                        if(ret.size == sizeof(int16_t) * 3){
                            int16_t magnetic_xyz_uT_x10[3] = {};
                            memcpy(
                                magnetic_xyz_uT_x10,
                                ret.data,
                                sizeof(magnetic_xyz_uT_x10));
                            if(magnetic_count >= MAGNETIC_BUFFER_SIZE){
                                magnetic_head =
                                    (magnetic_head + 1) %
                                    MAGNETIC_BUFFER_SIZE;
                                magnetic_count--;
                            }
                            magnetic_buffer[magnetic_tail] = {
                                magnetic_xyz_uT_x10[0] * 0.1f,
                                magnetic_xyz_uT_x10[1] * 0.1f,
                                magnetic_xyz_uT_x10[2] * 0.1f,
                                millis()
                            };
                            magnetic_tail =
                                (magnetic_tail + 1) %
                                MAGNETIC_BUFFER_SIZE;
                            magnetic_count++;
                        }
                        break;
                    case CAN_TYPE_PRESSURE:
                        if(ret.size == sizeof(int32_t)){
                            memcpy(
                                &latest_pressure_pa,
                                ret.data,
                                sizeof(latest_pressure_pa));
                            pressure_received = true;
                        }
                        break;
                    case CAN_TYPE_ANGULAR_VELOCITY_XYZ:
                        if(ret.size == sizeof(int16_t) * 3){
                            int16_t angular_velocity_xyz_x500[3] = {};
                            memcpy(
                                angular_velocity_xyz_x500,
                                ret.data,
                                sizeof(angular_velocity_xyz_x500));
                            if(angular_velocity_count >=
                                    ANGULAR_VELOCITY_BUFFER_SIZE){
                                angular_velocity_head =
                                    (angular_velocity_head + 1) %
                                    ANGULAR_VELOCITY_BUFFER_SIZE;
                                angular_velocity_count--;
                            }
                            angular_velocity_buffer[
                                angular_velocity_tail] = {
                                    angular_velocity_xyz_x500[0] * 0.002f,
                                    angular_velocity_xyz_x500[1] * 0.002f,
                                    angular_velocity_xyz_x500[2] * 0.002f,
                                    sensor_frame_ms
                                };
                            angular_velocity_tail =
                                (angular_velocity_tail + 1) %
                                ANGULAR_VELOCITY_BUFFER_SIZE;
                            angular_velocity_count++;
                        }
                        break;
                    case CAN_TYPE_NORMAL:
                        if(ret.size == sizeof(uint8_t)){
                            Can::Data::Event event;
                            event.bytes = Can::Data::EventBytes::None;
                            if(ret.data[0] == Can::Data::AscendDetected)
                                event.bytes = Can::Data::EventBytes::AscendDetected;
                            else if(ret.data[0] == Can::Data::LandingDetected)
                                event.bytes = Can::Data::EventBytes::LandingDetected;
                            event.ts_ms = millis();
                            if(event_count >= EVENT_BUFFER_SIZE){
                                event_head = (event_head + 1) % EVENT_BUFFER_SIZE;
                                event_count--;
                            }
                            event_buffer[event_tail] = event;
                            event_tail = (event_tail + 1) % EVENT_BUFFER_SIZE;
                            event_count++;
                        }
                        break;
                }
            // モーター基板からの受信
            }else if(addr == CAN_ADDR_CONTROL && src == CAN_SRC_MOTOR){
                switch(type){
                    case CAN_TYPE_ENCODER:
                        if(ret.size == sizeof(int32_t) * 2){
                            if(encoder_count >= ENCODER_BUFFER_SIZE){
                                encoder_head = (encoder_head + 1) % ENCODER_BUFFER_SIZE;
                                encoder_count--;
                            }
                            memcpy(&encoder_buffer[encoder_tail].left_mm, ret.data, sizeof(int32_t));
                            memcpy(&encoder_buffer[encoder_tail].right_mm, ret.data + sizeof(int32_t), sizeof(int32_t));
                            encoder_buffer[encoder_tail].ts_ms = millis();
                            encoder_tail = (encoder_tail + 1) % ENCODER_BUFFER_SIZE;
                            encoder_count++;
                        }
                        break;
                    case CAN_TYPE_NORMAL:
                        if(ret.size == sizeof(uint8_t)){
                            Can::Data::Event event;
                            event.bytes = Can::Data::EventBytes::None;
                            if(ret.data[0] == Can::Data::StuckResolved)
                                event.bytes = Can::Data::EventBytes::StuckResolved;
                            else if(ret.data[0] == Can::Data::SeparationFinished)
                                event.bytes = Can::Data::EventBytes::SeparationFinished;
                            // wire値0x04は、モーター送信元の場合だけ反転復帰失敗を表す。
                            else if(ret.data[0] == 0x04)
                                event.bytes = Can::Data::EventBytes::UprightRecoveryFailed;
                            if(event.bytes == Can::Data::EventBytes::None) break;
                            event.ts_ms = millis();
                            if(event_count >= EVENT_BUFFER_SIZE){
                                event_head = (event_head + 1) % EVENT_BUFFER_SIZE;
                                event_count--;
                            }
                            event_buffer[event_tail] = event;
                            event_tail = (event_tail + 1) % EVENT_BUFFER_SIZE;
                            event_count++;
                        }
                        break;
                }
            }
        }
    }
}

bool SrvCan::read(Can::Data::Heading *heading) {
    if(heading_count == 0){
        return false;
    }
    heading->heading = heading_buffer[heading_head].heading;
    heading->ts_ms = heading_buffer[heading_head].ts_ms;
    heading_head = (heading_head + 1) % HEADING_BUFFER_SIZE;
    heading_count--;
    return true;
}

bool SrvCan::read(Can::Data::Sensor *sensor) {
    if(sensor_count == 0){
        return false;
    }
    sensor->acc_x = sensor_buffer[sensor_head].acc_x;
    sensor->acc_y = sensor_buffer[sensor_head].acc_y;
    sensor->acc_z = sensor_buffer[sensor_head].acc_z;
    sensor->atm = sensor_buffer[sensor_head].atm;
    sensor->ts_ms = sensor_buffer[sensor_head].ts_ms;
    sensor_head = (sensor_head + 1) % SENSOR_BUFFER_SIZE;
    sensor_count--;
    return true;
}

bool SrvCan::read(Can::Data::Encoder *encoder) {
    if(encoder_count == 0){
        return false;
    }
    encoder->left_mm = encoder_buffer[encoder_head].left_mm;
    encoder->right_mm = encoder_buffer[encoder_head].right_mm;
    encoder->ts_ms = encoder_buffer[encoder_head].ts_ms;
    encoder_head = (encoder_head + 1) % ENCODER_BUFFER_SIZE;
    encoder_count--;
    return true;
}

Can::Data::Event SrvCan::readEvent() {
    Can::Data::Event event;
    if(event_count == 0){
        event.bytes = Can::Data::EventBytes::None;
        event.ts_ms = 0;
        return event;
    }
    event = event_buffer[event_head];
    event_head = (event_head + 1) % EVENT_BUFFER_SIZE;
    event_count--;
    return event;
}

bool SrvCan::read(Can::Data::MagneticField *magnetic) {
    if(magnetic == nullptr || magnetic_count == 0){
        return false;
    }
    *magnetic = magnetic_buffer[magnetic_head];
    magnetic_head = (magnetic_head + 1) % MAGNETIC_BUFFER_SIZE;
    magnetic_count--;
    return true;
}

bool SrvCan::read(Can::Data::AngularVelocity *angular_velocity) {
    if(angular_velocity == nullptr || angular_velocity_count == 0){
        return false;
    }
    *angular_velocity = angular_velocity_buffer[angular_velocity_head];
    angular_velocity_head =
        (angular_velocity_head + 1) % ANGULAR_VELOCITY_BUFFER_SIZE;
    angular_velocity_count--;
    return true;
}
