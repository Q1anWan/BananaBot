#pragma once
#ifndef FISHMESSAGE_H
#define FISHMESSAGE_H
#include <cstdint>
#include "mavlink.h"
#include "om.h"

extern uint32_t fishPrintf(uint8_t *buf, const char *str, ...);
#define MSG_MOTOR_EXTERN_FDB_LEN 4
#define MSG_MOTOR_EXTERN_CTRL_LEN 12

#pragma pack(push) //保存对齐状态
#pragma pack(1)

struct Msg_INS_t {
    float quaternion[4];
    /*R-F-U*/
    float accel[3];
    float gyro[3];
    uint64_t timestamp;
};

struct Msg_WheelControl_t {
    //meters per second
    bool enable;
    float mps[4];
    bool enable_extern;
    float extern_motor[2];
    uint64_t timestamp;
};

struct Msg_WheelFDB_t {
    //meters per second
    float mps[4];
    int16_t extern_rpm[2];
    uint64_t timestamp;
};

struct Msg_Servo_t {
    //Duty Cycle
    bool enable;
    uint16_t servo[7];
    uint64_t timestamp;
};

struct Msg_MotorExtern_t {
   int16_t motor[6];
};
struct Msg_MotorExternFDB_t {
    int16_t motor[2];
};



struct Msg_usb_rx_data_processed_t{
    mavlink_chs_ctrl_info_t chs_ctrl_info;
    mavlink_chs_motor_info_t chs_motor_info;
    mavlink_chs_servos_info_t chs_servos_info;
    mavlink_chs_manage_info_t chs_manage_info;
    bool update_list_in_order[4]; // 0:chs_ctrl_info 1:chs_motor_info 2:chs_servos_info 3:chs_manage_info
};

struct Msg_spi_rx_data_processed_t{
    Msg_MotorExtern_t chs_motor_info;
    mavlink_chs_servos_info_t chs_servos_info;
    mavlink_chs_manage_info_t chs_manage_info;
    bool update;
};

struct Msg_Remoter_t{
    uint8_t Online:2;
    uint8_t switch_left:3;
    uint8_t switch_right:3;
    int16_t ch_0;
    int16_t ch_1;
    int16_t ch_2;
    int16_t ch_3;
    int16_t wheel;
    uint64_t timestamp;
};

#pragma pack(pop) //恢复对齐状态
#endif