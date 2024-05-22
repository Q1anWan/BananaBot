#pragma once
#ifndef BANANA_MSGS_H
#define BANANA_MSGS_H
#ifdef __cplusplus
extern "C" {
#endif

#include <cstdint>
#pragma pack(push, 1)
struct Msg_Battery_Voltage_t {
    bool isBattery;
    float voltage;
    uint64_t time_stamp;
};

struct Msg_INS_t {
    /*F-L-U*/
    float quaternion[4];
    float euler[3];     //RPY

    float accel[3];
    float gyro[3];
    uint64_t timestamp;
};

//                X
//            Y---|
//      1---a---4 |
//      |       | |
//   3  L       R  6
//      |       |
//      2---a---5
//

struct Msg_Motor_Ctr_t {
    bool enable;
    float torque[6];
};

struct Msg_Odometer_t {
    float x;
    float v;
};

struct Msg_Link_t {
    float angel_left[2];
    float angel_right[2];
};

enum class Msg_ErrorStatus: uint8_t {
    OK = 0,
    ERROR = 1,
    WARNING = 2,
};

enum class Msg_ThreadID: uint8_t {
    LED = 0,
    IMU = 1,
    IMU_HEAT = 2,
    MOTOR = 3,
    MAX_ID
};


struct Msg_Thread_Status_t {
    Msg_ThreadID thread_id;
    Msg_ErrorStatus status;
};

struct Msg_DBG_t {
    float dbg[6];
};

struct Msg_Remoter_t {
    bool online;
    float ch_0;
    float ch_1;
    float ch_2;
    float ch_3;
    float wheel;
    int8_t switch_left;
    int8_t switch_right;
    uint64_t timestamp;
};

#pragma pack(pop)
#ifdef __cplusplus
}
#endif
#endif