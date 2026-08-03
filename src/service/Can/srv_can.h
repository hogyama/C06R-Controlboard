#pragma once
#include <CANCREATE.h>
#include <Arduino.h>

// 優先度
#define CAN_PRIO_CRITICAL 0b00
#define CAN_PRIO_HIGH 0b01
#define CAN_PRIO_NORMAL 0b11
// 送信先
#define CAN_ADDR_BROADCAST 0b011
#define CAN_ADDR_CONTROL 0b100
#define CAN_ADDR_SENSOR 0b010
#define CAN_ADDR_MOTOR 0b001
// 送信元
#define CAN_SRC_CONTROL 0b00
#define CAN_SRC_SENSOR 0b01
#define CAN_SRC_MOTOR 0b10
// コマンド
#define CAN_TYPE_NORMAL 0b0000
#define CAN_TYPE_HEADING 0b0001
#define CAN_TYPE_ACCELERATION_XYZ 0b0010
// 補正済み地磁気XYZをµT×10のint16_t×3で受信する。
#define CAN_TYPE_MAGNETIC_XYZ 0b0011
#define CAN_TYPE_PRESSURE 0b0100
#define CAN_TYPE_ANGULAR_VELOCITY_XYZ 0b0110
#define CAN_TYPE_ENCODER 0b0001

// buffer size
#define HEADING_BUFFER_SIZE 2
#define SENSOR_BUFFER_SIZE 2
#define MAGNETIC_BUFFER_SIZE 2
#define ANGULAR_VELOCITY_BUFFER_SIZE 2
#define ENCODER_BUFFER_SIZE 2
#define EVENT_BUFFER_SIZE 3

// prio:2bit, addr:3bit, src:2bit, type:4bit
#define MAKE_CAN_ID(prio, addr, src, type) \
    ( ((prio & 0b11)   << 9) | \
      ((addr & 0b111)  << 6) | \
      ((src  & 0b11)   << 4) | \
      (type  & 0b1111) )

#define PARSE_CAN_ID(id, prio, addr, src, type) \
    do { \
        prio = (id >> 9) & 0b11; \
        addr = (id >> 6) & 0b111; \
        src  = (id >> 4) & 0b11; \
        type = id & 0b1111; \
    } while(0)


namespace Can::Data{
    
    enum EventBytes : uint8_t {
        None = 0x00,
        StuckResolved       = 0x01,
        SeparationFinished  = 0x02,
        AscendDetected      = 0x04,
        LandingDetected     = 0x05,
        // 内部識別値。モーター基板からwire値0x04を受信した時だけ変換する。
        UprightRecoveryFailed = 0x06,
    };

    struct Heading {
        float heading;
        uint32_t ts_ms;
    };
    struct Sensor {
        float acc_x;
        float acc_y;
        float acc_z;
        int32_t atm;
        uint32_t ts_ms;
    };
    struct MagneticField {
        float x_uT;
        float y_uT;
        float z_uT;
        uint32_t ts_ms;
    };
    struct AngularVelocity {
        float x_rad_s;
        float y_rad_s;
        float z_rad_s;
        uint32_t ts_ms;
    };
    struct Encoder {
        int32_t left_mm;
        int32_t right_mm;
        uint32_t ts_ms;
    };
    struct Event {
        EventBytes bytes;
        uint32_t ts_ms;
    };
}

namespace Can::Command{
    enum ActionType : char {
        Reset             = 'r',
        SequenceStart     = 'k', 
        NotifyGoal        = 'g', 

        NotifySeparation  = 't', 

        NotifyStuck       = 's', 
        NotifyFlipped     = 'S', // 機体反転をモーター基板へ通知する
        ConfirmUpright    = 'U', // 正常姿勢を500ms確認後、反転復帰動作を停止する

        ServoUnlock = 'b', 
        ServoLock    = 'd' 
    };

    struct Action {
        static constexpr uint8_t TYPE_ID = 0x01;
        uint8_t type_id = TYPE_ID; 
        ActionType type;
    };

    struct Velocity {
        float velocity_mm_s;
        float omega_rad_s;
    };
}
class SrvCan {
    public: 

        SrvCan();
        bool begin(int rx, int tx);
        bool send(Can::Command::Velocity velocity);
        bool send(Can::Command::Action cmd);
        void poll();

        // Headingは東0度,反時計回り正の角度(rad)を取得する
        bool read(Can::Data::Heading *heading);
        bool read(Can::Data::Sensor *sensor);
        bool read(Can::Data::MagneticField *magnetic);
        bool read(Can::Data::AngularVelocity *angular_velocity);
        bool read(Can::Data::Encoder *encoder);

        Can::Data::Event readEvent();
    private:
        CAN_CREATE can_create{true};

        Can::Data::Heading heading_buffer[HEADING_BUFFER_SIZE];
        Can::Data::Sensor sensor_buffer[SENSOR_BUFFER_SIZE];
        Can::Data::MagneticField magnetic_buffer[MAGNETIC_BUFFER_SIZE];
        Can::Data::AngularVelocity angular_velocity_buffer[ANGULAR_VELOCITY_BUFFER_SIZE];
        Can::Data::Encoder encoder_buffer[ENCODER_BUFFER_SIZE];
        Can::Data::Event event_buffer[EVENT_BUFFER_SIZE];

        uint8_t heading_head;
        uint8_t sensor_head;
        uint8_t magnetic_head;
        uint8_t angular_velocity_head;
        uint8_t encoder_head;
        uint8_t event_head;

        uint8_t heading_tail;
        uint8_t sensor_tail;
        uint8_t magnetic_tail;
        uint8_t angular_velocity_tail;
        uint8_t encoder_tail;
        uint8_t event_tail;

        uint8_t heading_count;
        uint8_t sensor_count;
        uint8_t magnetic_count;
        uint8_t angular_velocity_count;
        uint8_t encoder_count;
        uint8_t event_count;
};
