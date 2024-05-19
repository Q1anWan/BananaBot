#pragma once
// MESSAGE CHS_MOTOR_INFO PACKING

#define MAVLINK_MSG_ID_CHS_MOTOR_INFO 1


typedef struct __mavlink_chs_motor_info_t {
 int16_t motor[4]; /*< [rpm] Indicate angular speed of wheel motors. Range [-6000, 6000]*/
} mavlink_chs_motor_info_t;

#define MAVLINK_MSG_ID_CHS_MOTOR_INFO_LEN 8
#define MAVLINK_MSG_ID_CHS_MOTOR_INFO_MIN_LEN 8
#define MAVLINK_MSG_ID_1_LEN 8
#define MAVLINK_MSG_ID_1_MIN_LEN 8

#define MAVLINK_MSG_ID_CHS_MOTOR_INFO_CRC 8
#define MAVLINK_MSG_ID_1_CRC 8

#define MAVLINK_MSG_CHS_MOTOR_INFO_FIELD_MOTOR_LEN 4

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CHS_MOTOR_INFO { \
    1, \
    "CHS_MOTOR_INFO", \
    1, \
    {  { "motor", NULL, MAVLINK_TYPE_INT16_T, 4, 0, offsetof(mavlink_chs_motor_info_t, motor) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CHS_MOTOR_INFO { \
    "CHS_MOTOR_INFO", \
    1, \
    {  { "motor", NULL, MAVLINK_TYPE_INT16_T, 4, 0, offsetof(mavlink_chs_motor_info_t, motor) }, \
         } \
}
#endif

/**
 * @brief Pack a chs_motor_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param motor [rpm] Indicate angular speed of wheel motors. Range [-6000, 6000]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_chs_motor_info_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const int16_t *motor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CHS_MOTOR_INFO_LEN];

    _mav_put_int16_t_array(buf, 0, motor, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CHS_MOTOR_INFO_LEN);
#else
    mavlink_chs_motor_info_t packet;

    mav_array_memcpy(packet.motor, motor, sizeof(int16_t)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CHS_MOTOR_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CHS_MOTOR_INFO;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CHS_MOTOR_INFO_MIN_LEN, MAVLINK_MSG_ID_CHS_MOTOR_INFO_LEN, MAVLINK_MSG_ID_CHS_MOTOR_INFO_CRC);
}

/**
 * @brief Pack a chs_motor_info message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param motor [rpm] Indicate angular speed of wheel motors. Range [-6000, 6000]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_chs_motor_info_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const int16_t *motor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CHS_MOTOR_INFO_LEN];

    _mav_put_int16_t_array(buf, 0, motor, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CHS_MOTOR_INFO_LEN);
#else
    mavlink_chs_motor_info_t packet;

    mav_array_memcpy(packet.motor, motor, sizeof(int16_t)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CHS_MOTOR_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CHS_MOTOR_INFO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CHS_MOTOR_INFO_MIN_LEN, MAVLINK_MSG_ID_CHS_MOTOR_INFO_LEN, MAVLINK_MSG_ID_CHS_MOTOR_INFO_CRC);
}

/**
 * @brief Encode a chs_motor_info struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param chs_motor_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_chs_motor_info_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_chs_motor_info_t* chs_motor_info)
{
    return mavlink_msg_chs_motor_info_pack(system_id, component_id, msg, chs_motor_info->motor);
}

/**
 * @brief Encode a chs_motor_info struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param chs_motor_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_chs_motor_info_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_chs_motor_info_t* chs_motor_info)
{
    return mavlink_msg_chs_motor_info_pack_chan(system_id, component_id, chan, msg, chs_motor_info->motor);
}

/**
 * @brief Send a chs_motor_info message
 * @param chan MAVLink channel to send the message
 *
 * @param motor [rpm] Indicate angular speed of wheel motors. Range [-6000, 6000]
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_chs_motor_info_send(mavlink_channel_t chan, const int16_t *motor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CHS_MOTOR_INFO_LEN];

    _mav_put_int16_t_array(buf, 0, motor, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHS_MOTOR_INFO, buf, MAVLINK_MSG_ID_CHS_MOTOR_INFO_MIN_LEN, MAVLINK_MSG_ID_CHS_MOTOR_INFO_LEN, MAVLINK_MSG_ID_CHS_MOTOR_INFO_CRC);
#else
    mavlink_chs_motor_info_t packet;

    mav_array_memcpy(packet.motor, motor, sizeof(int16_t)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHS_MOTOR_INFO, (const char *)&packet, MAVLINK_MSG_ID_CHS_MOTOR_INFO_MIN_LEN, MAVLINK_MSG_ID_CHS_MOTOR_INFO_LEN, MAVLINK_MSG_ID_CHS_MOTOR_INFO_CRC);
#endif
}

/**
 * @brief Send a chs_motor_info message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_chs_motor_info_send_struct(mavlink_channel_t chan, const mavlink_chs_motor_info_t* chs_motor_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_chs_motor_info_send(chan, chs_motor_info->motor);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHS_MOTOR_INFO, (const char *)chs_motor_info, MAVLINK_MSG_ID_CHS_MOTOR_INFO_MIN_LEN, MAVLINK_MSG_ID_CHS_MOTOR_INFO_LEN, MAVLINK_MSG_ID_CHS_MOTOR_INFO_CRC);
#endif
}

#if MAVLINK_MSG_ID_CHS_MOTOR_INFO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_chs_motor_info_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const int16_t *motor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _mav_put_int16_t_array(buf, 0, motor, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHS_MOTOR_INFO, buf, MAVLINK_MSG_ID_CHS_MOTOR_INFO_MIN_LEN, MAVLINK_MSG_ID_CHS_MOTOR_INFO_LEN, MAVLINK_MSG_ID_CHS_MOTOR_INFO_CRC);
#else
    mavlink_chs_motor_info_t *packet = (mavlink_chs_motor_info_t *)msgbuf;

    mav_array_memcpy(packet->motor, motor, sizeof(int16_t)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHS_MOTOR_INFO, (const char *)packet, MAVLINK_MSG_ID_CHS_MOTOR_INFO_MIN_LEN, MAVLINK_MSG_ID_CHS_MOTOR_INFO_LEN, MAVLINK_MSG_ID_CHS_MOTOR_INFO_CRC);
#endif
}
#endif

#endif

// MESSAGE CHS_MOTOR_INFO UNPACKING


/**
 * @brief Get field motor from chs_motor_info message
 *
 * @return [rpm] Indicate angular speed of wheel motors. Range [-6000, 6000]
 */
static inline uint16_t mavlink_msg_chs_motor_info_get_motor(const mavlink_message_t* msg, int16_t *motor)
{
    return _MAV_RETURN_int16_t_array(msg, motor, 4,  0);
}

/**
 * @brief Decode a chs_motor_info message into a struct
 *
 * @param msg The message to decode
 * @param chs_motor_info C-struct to decode the message contents into
 */
static inline void mavlink_msg_chs_motor_info_decode(const mavlink_message_t* msg, mavlink_chs_motor_info_t* chs_motor_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_chs_motor_info_get_motor(msg, chs_motor_info->motor);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CHS_MOTOR_INFO_LEN? msg->len : MAVLINK_MSG_ID_CHS_MOTOR_INFO_LEN;
        memset(chs_motor_info, 0, MAVLINK_MSG_ID_CHS_MOTOR_INFO_LEN);
    memcpy(chs_motor_info, _MAV_PAYLOAD(msg), len);
#endif
}
