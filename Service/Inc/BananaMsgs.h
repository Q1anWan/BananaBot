#pragma once
#ifndef BANANA_MSGS_H
#define BANANA_MSGS_H
#ifdef __cplusplus
extern "C" {
#endif

#include <cstdint>
#pragma pack(push, 1)
struct msgBatteryVoltage_t {
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

#pragma pack(pop)
#ifdef __cplusplus
}
#endif
#endif