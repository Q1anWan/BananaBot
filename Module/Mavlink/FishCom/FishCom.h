/** @file
 *  @brief MAVLink comm protocol generated from FishCom.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_FISHCOM_H
#define MAVLINK_FISHCOM_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_FISHCOM.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#define MAVLINK_FISHCOM_XML_HASH 6214921265985737887

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{0, 136, 12, 12, 0, 0, 0}, {1, 8, 8, 8, 0, 0, 0}, {2, 68, 28, 28, 0, 0, 0}, {3, 134, 24, 24, 0, 0, 0}, {4, 79, 14, 14, 0, 0, 0}, {5, 255, 3, 3, 0, 0, 0}, {6, 250, 11, 11, 0, 0, 0}}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_FISHCOM

// ENUM DEFINITIONS


/** @brief Chassis system and ID. */
#ifndef HAVE_ENUM_CHS_SYSTEM_ID
#define HAVE_ENUM_CHS_SYSTEM_ID
typedef enum CHS_SYSTEM_ID
{
   CHS_ID_ORANGE=1, /* OrangePi. | */
   CHS_ID_ESP32=2, /* ESP32 controller. | */
   CHS_ID_CHASSIS=3, /* Chassis. | */
   CHS_SYSTEM_ID_ENUM_END=4, /*  | */
} CHS_SYSTEM_ID;
#endif

// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

// MESSAGE DEFINITIONS
#include "./mavlink_msg_chs_ctrl_info.h"
#include "./mavlink_msg_chs_motor_info.h"
#include "./mavlink_msg_chs_odom_info.h"
#include "./mavlink_msg_chs_imu_info.h"
#include "./mavlink_msg_chs_servos_info.h"
#include "./mavlink_msg_chs_manage_info.h"
#include "./mavlink_msg_chs_remoter_info.h"

// base include



#if MAVLINK_FISHCOM_XML_HASH == MAVLINK_PRIMARY_XML_HASH
# define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_CHS_CTRL_INFO, MAVLINK_MESSAGE_INFO_CHS_MOTOR_INFO, MAVLINK_MESSAGE_INFO_CHS_ODOM_INFO, MAVLINK_MESSAGE_INFO_CHS_IMU_INFO, MAVLINK_MESSAGE_INFO_CHS_SERVOS_INFO, MAVLINK_MESSAGE_INFO_CHS_MANAGE_INFO, MAVLINK_MESSAGE_INFO_CHS_REMOTER_INFO}
# define MAVLINK_MESSAGE_NAMES {{ "CHS_CTRL_INFO", 0 }, { "CHS_IMU_INFO", 3 }, { "CHS_MANAGE_INFO", 5 }, { "CHS_MOTOR_INFO", 1 }, { "CHS_ODOM_INFO", 2 }, { "CHS_REMOTER_INFO", 6 }, { "CHS_SERVOS_INFO", 4 }}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_FISHCOM_H
