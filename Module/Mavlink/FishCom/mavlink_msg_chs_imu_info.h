#pragma once
// MESSAGE CHS_IMU_INFO PACKING

#define MAVLINK_MSG_ID_CHS_IMU_INFO 3


typedef struct __mavlink_chs_imu_info_t {
 float accel[3]; /*< [m/s^2] Feedback accelmeter message.*/
 float gyro[3]; /*< [rad/s] Feedback gyrometer message.*/
} mavlink_chs_imu_info_t;

#define MAVLINK_MSG_ID_CHS_IMU_INFO_LEN 24
#define MAVLINK_MSG_ID_CHS_IMU_INFO_MIN_LEN 24
#define MAVLINK_MSG_ID_3_LEN 24
#define MAVLINK_MSG_ID_3_MIN_LEN 24

#define MAVLINK_MSG_ID_CHS_IMU_INFO_CRC 134
#define MAVLINK_MSG_ID_3_CRC 134

#define MAVLINK_MSG_CHS_IMU_INFO_FIELD_ACCEL_LEN 3
#define MAVLINK_MSG_CHS_IMU_INFO_FIELD_GYRO_LEN 3

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CHS_IMU_INFO { \
    3, \
    "CHS_IMU_INFO", \
    2, \
    {  { "accel", NULL, MAVLINK_TYPE_FLOAT, 3, 0, offsetof(mavlink_chs_imu_info_t, accel) }, \
         { "gyro", NULL, MAVLINK_TYPE_FLOAT, 3, 12, offsetof(mavlink_chs_imu_info_t, gyro) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CHS_IMU_INFO { \
    "CHS_IMU_INFO", \
    2, \
    {  { "accel", NULL, MAVLINK_TYPE_FLOAT, 3, 0, offsetof(mavlink_chs_imu_info_t, accel) }, \
         { "gyro", NULL, MAVLINK_TYPE_FLOAT, 3, 12, offsetof(mavlink_chs_imu_info_t, gyro) }, \
         } \
}
#endif

/**
 * @brief Pack a chs_imu_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param accel [m/s^2] Feedback accelmeter message.
 * @param gyro [rad/s] Feedback gyrometer message.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_chs_imu_info_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const float *accel, const float *gyro)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CHS_IMU_INFO_LEN];

    _mav_put_float_array(buf, 0, accel, 3);
    _mav_put_float_array(buf, 12, gyro, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CHS_IMU_INFO_LEN);
#else
    mavlink_chs_imu_info_t packet;

    mav_array_memcpy(packet.accel, accel, sizeof(float)*3);
    mav_array_memcpy(packet.gyro, gyro, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CHS_IMU_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CHS_IMU_INFO;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CHS_IMU_INFO_MIN_LEN, MAVLINK_MSG_ID_CHS_IMU_INFO_LEN, MAVLINK_MSG_ID_CHS_IMU_INFO_CRC);
}

/**
 * @brief Pack a chs_imu_info message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param accel [m/s^2] Feedback accelmeter message.
 * @param gyro [rad/s] Feedback gyrometer message.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_chs_imu_info_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const float *accel,const float *gyro)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CHS_IMU_INFO_LEN];

    _mav_put_float_array(buf, 0, accel, 3);
    _mav_put_float_array(buf, 12, gyro, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CHS_IMU_INFO_LEN);
#else
    mavlink_chs_imu_info_t packet;

    mav_array_memcpy(packet.accel, accel, sizeof(float)*3);
    mav_array_memcpy(packet.gyro, gyro, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CHS_IMU_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CHS_IMU_INFO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CHS_IMU_INFO_MIN_LEN, MAVLINK_MSG_ID_CHS_IMU_INFO_LEN, MAVLINK_MSG_ID_CHS_IMU_INFO_CRC);
}

/**
 * @brief Encode a chs_imu_info struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param chs_imu_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_chs_imu_info_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_chs_imu_info_t* chs_imu_info)
{
    return mavlink_msg_chs_imu_info_pack(system_id, component_id, msg, chs_imu_info->accel, chs_imu_info->gyro);
}

/**
 * @brief Encode a chs_imu_info struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param chs_imu_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_chs_imu_info_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_chs_imu_info_t* chs_imu_info)
{
    return mavlink_msg_chs_imu_info_pack_chan(system_id, component_id, chan, msg, chs_imu_info->accel, chs_imu_info->gyro);
}

/**
 * @brief Send a chs_imu_info message
 * @param chan MAVLink channel to send the message
 *
 * @param accel [m/s^2] Feedback accelmeter message.
 * @param gyro [rad/s] Feedback gyrometer message.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_chs_imu_info_send(mavlink_channel_t chan, const float *accel, const float *gyro)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CHS_IMU_INFO_LEN];

    _mav_put_float_array(buf, 0, accel, 3);
    _mav_put_float_array(buf, 12, gyro, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHS_IMU_INFO, buf, MAVLINK_MSG_ID_CHS_IMU_INFO_MIN_LEN, MAVLINK_MSG_ID_CHS_IMU_INFO_LEN, MAVLINK_MSG_ID_CHS_IMU_INFO_CRC);
#else
    mavlink_chs_imu_info_t packet;

    mav_array_memcpy(packet.accel, accel, sizeof(float)*3);
    mav_array_memcpy(packet.gyro, gyro, sizeof(float)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHS_IMU_INFO, (const char *)&packet, MAVLINK_MSG_ID_CHS_IMU_INFO_MIN_LEN, MAVLINK_MSG_ID_CHS_IMU_INFO_LEN, MAVLINK_MSG_ID_CHS_IMU_INFO_CRC);
#endif
}

/**
 * @brief Send a chs_imu_info message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_chs_imu_info_send_struct(mavlink_channel_t chan, const mavlink_chs_imu_info_t* chs_imu_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_chs_imu_info_send(chan, chs_imu_info->accel, chs_imu_info->gyro);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHS_IMU_INFO, (const char *)chs_imu_info, MAVLINK_MSG_ID_CHS_IMU_INFO_MIN_LEN, MAVLINK_MSG_ID_CHS_IMU_INFO_LEN, MAVLINK_MSG_ID_CHS_IMU_INFO_CRC);
#endif
}

#if MAVLINK_MSG_ID_CHS_IMU_INFO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_chs_imu_info_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const float *accel, const float *gyro)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _mav_put_float_array(buf, 0, accel, 3);
    _mav_put_float_array(buf, 12, gyro, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHS_IMU_INFO, buf, MAVLINK_MSG_ID_CHS_IMU_INFO_MIN_LEN, MAVLINK_MSG_ID_CHS_IMU_INFO_LEN, MAVLINK_MSG_ID_CHS_IMU_INFO_CRC);
#else
    mavlink_chs_imu_info_t *packet = (mavlink_chs_imu_info_t *)msgbuf;

    mav_array_memcpy(packet->accel, accel, sizeof(float)*3);
    mav_array_memcpy(packet->gyro, gyro, sizeof(float)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHS_IMU_INFO, (const char *)packet, MAVLINK_MSG_ID_CHS_IMU_INFO_MIN_LEN, MAVLINK_MSG_ID_CHS_IMU_INFO_LEN, MAVLINK_MSG_ID_CHS_IMU_INFO_CRC);
#endif
}
#endif

#endif

// MESSAGE CHS_IMU_INFO UNPACKING


/**
 * @brief Get field accel from chs_imu_info message
 *
 * @return [m/s^2] Feedback accelmeter message.
 */
static inline uint16_t mavlink_msg_chs_imu_info_get_accel(const mavlink_message_t* msg, float *accel)
{
    return _MAV_RETURN_float_array(msg, accel, 3,  0);
}

/**
 * @brief Get field gyro from chs_imu_info message
 *
 * @return [rad/s] Feedback gyrometer message.
 */
static inline uint16_t mavlink_msg_chs_imu_info_get_gyro(const mavlink_message_t* msg, float *gyro)
{
    return _MAV_RETURN_float_array(msg, gyro, 3,  12);
}

/**
 * @brief Decode a chs_imu_info message into a struct
 *
 * @param msg The message to decode
 * @param chs_imu_info C-struct to decode the message contents into
 */
static inline void mavlink_msg_chs_imu_info_decode(const mavlink_message_t* msg, mavlink_chs_imu_info_t* chs_imu_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_chs_imu_info_get_accel(msg, chs_imu_info->accel);
    mavlink_msg_chs_imu_info_get_gyro(msg, chs_imu_info->gyro);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CHS_IMU_INFO_LEN? msg->len : MAVLINK_MSG_ID_CHS_IMU_INFO_LEN;
        memset(chs_imu_info, 0, MAVLINK_MSG_ID_CHS_IMU_INFO_LEN);
    memcpy(chs_imu_info, _MAV_PAYLOAD(msg), len);
#endif
}
