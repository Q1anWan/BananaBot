#pragma once
// MESSAGE CHS_MANAGE_INFO PACKING

#define MAVLINK_MSG_ID_CHS_MANAGE_INFO 5


typedef struct __mavlink_chs_manage_info_t {
 uint8_t enable_chassis; /*<  Boolean. Set 1/0 to enable/disable chassis's close-loop control. Invalid with UINT8_MAX.*/
 uint8_t enable_servos; /*<  Boolean. Set true to enable/disable servo pwm generating. Invalid with UINT8_MAX.*/
 uint8_t reset_quaternion; /*<  Boolean. Set true to reset quaternion. Invalid with UINT8_MAX.*/
} mavlink_chs_manage_info_t;

#define MAVLINK_MSG_ID_CHS_MANAGE_INFO_LEN 3
#define MAVLINK_MSG_ID_CHS_MANAGE_INFO_MIN_LEN 3
#define MAVLINK_MSG_ID_5_LEN 3
#define MAVLINK_MSG_ID_5_MIN_LEN 3

#define MAVLINK_MSG_ID_CHS_MANAGE_INFO_CRC 255
#define MAVLINK_MSG_ID_5_CRC 255



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CHS_MANAGE_INFO { \
    5, \
    "CHS_MANAGE_INFO", \
    3, \
    {  { "enable_chassis", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_chs_manage_info_t, enable_chassis) }, \
         { "enable_servos", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_chs_manage_info_t, enable_servos) }, \
         { "reset_quaternion", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_chs_manage_info_t, reset_quaternion) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CHS_MANAGE_INFO { \
    "CHS_MANAGE_INFO", \
    3, \
    {  { "enable_chassis", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_chs_manage_info_t, enable_chassis) }, \
         { "enable_servos", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_chs_manage_info_t, enable_servos) }, \
         { "reset_quaternion", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_chs_manage_info_t, reset_quaternion) }, \
         } \
}
#endif

/**
 * @brief Pack a chs_manage_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param enable_chassis  Boolean. Set 1/0 to enable/disable chassis's close-loop control. Invalid with UINT8_MAX.
 * @param enable_servos  Boolean. Set true to enable/disable servo pwm generating. Invalid with UINT8_MAX.
 * @param reset_quaternion  Boolean. Set true to reset quaternion. Invalid with UINT8_MAX.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_chs_manage_info_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t enable_chassis, uint8_t enable_servos, uint8_t reset_quaternion)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CHS_MANAGE_INFO_LEN];
    _mav_put_uint8_t(buf, 0, enable_chassis);
    _mav_put_uint8_t(buf, 1, enable_servos);
    _mav_put_uint8_t(buf, 2, reset_quaternion);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CHS_MANAGE_INFO_LEN);
#else
    mavlink_chs_manage_info_t packet;
    packet.enable_chassis = enable_chassis;
    packet.enable_servos = enable_servos;
    packet.reset_quaternion = reset_quaternion;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CHS_MANAGE_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CHS_MANAGE_INFO;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CHS_MANAGE_INFO_MIN_LEN, MAVLINK_MSG_ID_CHS_MANAGE_INFO_LEN, MAVLINK_MSG_ID_CHS_MANAGE_INFO_CRC);
}

/**
 * @brief Pack a chs_manage_info message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param enable_chassis  Boolean. Set 1/0 to enable/disable chassis's close-loop control. Invalid with UINT8_MAX.
 * @param enable_servos  Boolean. Set true to enable/disable servo pwm generating. Invalid with UINT8_MAX.
 * @param reset_quaternion  Boolean. Set true to reset quaternion. Invalid with UINT8_MAX.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_chs_manage_info_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t enable_chassis,uint8_t enable_servos,uint8_t reset_quaternion)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CHS_MANAGE_INFO_LEN];
    _mav_put_uint8_t(buf, 0, enable_chassis);
    _mav_put_uint8_t(buf, 1, enable_servos);
    _mav_put_uint8_t(buf, 2, reset_quaternion);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CHS_MANAGE_INFO_LEN);
#else
    mavlink_chs_manage_info_t packet;
    packet.enable_chassis = enable_chassis;
    packet.enable_servos = enable_servos;
    packet.reset_quaternion = reset_quaternion;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CHS_MANAGE_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CHS_MANAGE_INFO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CHS_MANAGE_INFO_MIN_LEN, MAVLINK_MSG_ID_CHS_MANAGE_INFO_LEN, MAVLINK_MSG_ID_CHS_MANAGE_INFO_CRC);
}

/**
 * @brief Encode a chs_manage_info struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param chs_manage_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_chs_manage_info_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_chs_manage_info_t* chs_manage_info)
{
    return mavlink_msg_chs_manage_info_pack(system_id, component_id, msg, chs_manage_info->enable_chassis, chs_manage_info->enable_servos, chs_manage_info->reset_quaternion);
}

/**
 * @brief Encode a chs_manage_info struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param chs_manage_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_chs_manage_info_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_chs_manage_info_t* chs_manage_info)
{
    return mavlink_msg_chs_manage_info_pack_chan(system_id, component_id, chan, msg, chs_manage_info->enable_chassis, chs_manage_info->enable_servos, chs_manage_info->reset_quaternion);
}

/**
 * @brief Send a chs_manage_info message
 * @param chan MAVLink channel to send the message
 *
 * @param enable_chassis  Boolean. Set 1/0 to enable/disable chassis's close-loop control. Invalid with UINT8_MAX.
 * @param enable_servos  Boolean. Set true to enable/disable servo pwm generating. Invalid with UINT8_MAX.
 * @param reset_quaternion  Boolean. Set true to reset quaternion. Invalid with UINT8_MAX.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_chs_manage_info_send(mavlink_channel_t chan, uint8_t enable_chassis, uint8_t enable_servos, uint8_t reset_quaternion)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CHS_MANAGE_INFO_LEN];
    _mav_put_uint8_t(buf, 0, enable_chassis);
    _mav_put_uint8_t(buf, 1, enable_servos);
    _mav_put_uint8_t(buf, 2, reset_quaternion);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHS_MANAGE_INFO, buf, MAVLINK_MSG_ID_CHS_MANAGE_INFO_MIN_LEN, MAVLINK_MSG_ID_CHS_MANAGE_INFO_LEN, MAVLINK_MSG_ID_CHS_MANAGE_INFO_CRC);
#else
    mavlink_chs_manage_info_t packet;
    packet.enable_chassis = enable_chassis;
    packet.enable_servos = enable_servos;
    packet.reset_quaternion = reset_quaternion;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHS_MANAGE_INFO, (const char *)&packet, MAVLINK_MSG_ID_CHS_MANAGE_INFO_MIN_LEN, MAVLINK_MSG_ID_CHS_MANAGE_INFO_LEN, MAVLINK_MSG_ID_CHS_MANAGE_INFO_CRC);
#endif
}

/**
 * @brief Send a chs_manage_info message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_chs_manage_info_send_struct(mavlink_channel_t chan, const mavlink_chs_manage_info_t* chs_manage_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_chs_manage_info_send(chan, chs_manage_info->enable_chassis, chs_manage_info->enable_servos, chs_manage_info->reset_quaternion);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHS_MANAGE_INFO, (const char *)chs_manage_info, MAVLINK_MSG_ID_CHS_MANAGE_INFO_MIN_LEN, MAVLINK_MSG_ID_CHS_MANAGE_INFO_LEN, MAVLINK_MSG_ID_CHS_MANAGE_INFO_CRC);
#endif
}

#if MAVLINK_MSG_ID_CHS_MANAGE_INFO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_chs_manage_info_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t enable_chassis, uint8_t enable_servos, uint8_t reset_quaternion)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, enable_chassis);
    _mav_put_uint8_t(buf, 1, enable_servos);
    _mav_put_uint8_t(buf, 2, reset_quaternion);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHS_MANAGE_INFO, buf, MAVLINK_MSG_ID_CHS_MANAGE_INFO_MIN_LEN, MAVLINK_MSG_ID_CHS_MANAGE_INFO_LEN, MAVLINK_MSG_ID_CHS_MANAGE_INFO_CRC);
#else
    mavlink_chs_manage_info_t *packet = (mavlink_chs_manage_info_t *)msgbuf;
    packet->enable_chassis = enable_chassis;
    packet->enable_servos = enable_servos;
    packet->reset_quaternion = reset_quaternion;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHS_MANAGE_INFO, (const char *)packet, MAVLINK_MSG_ID_CHS_MANAGE_INFO_MIN_LEN, MAVLINK_MSG_ID_CHS_MANAGE_INFO_LEN, MAVLINK_MSG_ID_CHS_MANAGE_INFO_CRC);
#endif
}
#endif

#endif

// MESSAGE CHS_MANAGE_INFO UNPACKING


/**
 * @brief Get field enable_chassis from chs_manage_info message
 *
 * @return  Boolean. Set 1/0 to enable/disable chassis's close-loop control. Invalid with UINT8_MAX.
 */
static inline uint8_t mavlink_msg_chs_manage_info_get_enable_chassis(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field enable_servos from chs_manage_info message
 *
 * @return  Boolean. Set true to enable/disable servo pwm generating. Invalid with UINT8_MAX.
 */
static inline uint8_t mavlink_msg_chs_manage_info_get_enable_servos(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field reset_quaternion from chs_manage_info message
 *
 * @return  Boolean. Set true to reset quaternion. Invalid with UINT8_MAX.
 */
static inline uint8_t mavlink_msg_chs_manage_info_get_reset_quaternion(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Decode a chs_manage_info message into a struct
 *
 * @param msg The message to decode
 * @param chs_manage_info C-struct to decode the message contents into
 */
static inline void mavlink_msg_chs_manage_info_decode(const mavlink_message_t* msg, mavlink_chs_manage_info_t* chs_manage_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    chs_manage_info->enable_chassis = mavlink_msg_chs_manage_info_get_enable_chassis(msg);
    chs_manage_info->enable_servos = mavlink_msg_chs_manage_info_get_enable_servos(msg);
    chs_manage_info->reset_quaternion = mavlink_msg_chs_manage_info_get_reset_quaternion(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CHS_MANAGE_INFO_LEN? msg->len : MAVLINK_MSG_ID_CHS_MANAGE_INFO_LEN;
        memset(chs_manage_info, 0, MAVLINK_MSG_ID_CHS_MANAGE_INFO_LEN);
    memcpy(chs_manage_info, _MAV_PAYLOAD(msg), len);
#endif
}
