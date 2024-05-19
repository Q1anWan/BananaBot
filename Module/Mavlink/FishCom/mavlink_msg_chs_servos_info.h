#pragma once
// MESSAGE CHS_SERVOS_INFO PACKING

#define MAVLINK_MSG_ID_CHS_SERVOS_INFO 4


typedef struct __mavlink_chs_servos_info_t {
 uint16_t servos[7]; /*<  Indicate duty cycle of servo timer. Range from 499 to 24999 indicates 500/20000 to 2500/20000.*/
} mavlink_chs_servos_info_t;

#define MAVLINK_MSG_ID_CHS_SERVOS_INFO_LEN 14
#define MAVLINK_MSG_ID_CHS_SERVOS_INFO_MIN_LEN 14
#define MAVLINK_MSG_ID_4_LEN 14
#define MAVLINK_MSG_ID_4_MIN_LEN 14

#define MAVLINK_MSG_ID_CHS_SERVOS_INFO_CRC 79
#define MAVLINK_MSG_ID_4_CRC 79

#define MAVLINK_MSG_CHS_SERVOS_INFO_FIELD_SERVOS_LEN 7

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CHS_SERVOS_INFO { \
    4, \
    "CHS_SERVOS_INFO", \
    1, \
    {  { "servos", NULL, MAVLINK_TYPE_UINT16_T, 7, 0, offsetof(mavlink_chs_servos_info_t, servos) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CHS_SERVOS_INFO { \
    "CHS_SERVOS_INFO", \
    1, \
    {  { "servos", NULL, MAVLINK_TYPE_UINT16_T, 7, 0, offsetof(mavlink_chs_servos_info_t, servos) }, \
         } \
}
#endif

/**
 * @brief Pack a chs_servos_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param servos  Indicate duty cycle of servo timer. Range from 499 to 24999 indicates 500/20000 to 2500/20000.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_chs_servos_info_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const uint16_t *servos)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CHS_SERVOS_INFO_LEN];

    _mav_put_uint16_t_array(buf, 0, servos, 7);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CHS_SERVOS_INFO_LEN);
#else
    mavlink_chs_servos_info_t packet;

    mav_array_memcpy(packet.servos, servos, sizeof(uint16_t)*7);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CHS_SERVOS_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CHS_SERVOS_INFO;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CHS_SERVOS_INFO_MIN_LEN, MAVLINK_MSG_ID_CHS_SERVOS_INFO_LEN, MAVLINK_MSG_ID_CHS_SERVOS_INFO_CRC);
}

/**
 * @brief Pack a chs_servos_info message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param servos  Indicate duty cycle of servo timer. Range from 499 to 24999 indicates 500/20000 to 2500/20000.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_chs_servos_info_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const uint16_t *servos)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CHS_SERVOS_INFO_LEN];

    _mav_put_uint16_t_array(buf, 0, servos, 7);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CHS_SERVOS_INFO_LEN);
#else
    mavlink_chs_servos_info_t packet;

    mav_array_memcpy(packet.servos, servos, sizeof(uint16_t)*7);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CHS_SERVOS_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CHS_SERVOS_INFO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CHS_SERVOS_INFO_MIN_LEN, MAVLINK_MSG_ID_CHS_SERVOS_INFO_LEN, MAVLINK_MSG_ID_CHS_SERVOS_INFO_CRC);
}

/**
 * @brief Encode a chs_servos_info struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param chs_servos_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_chs_servos_info_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_chs_servos_info_t* chs_servos_info)
{
    return mavlink_msg_chs_servos_info_pack(system_id, component_id, msg, chs_servos_info->servos);
}

/**
 * @brief Encode a chs_servos_info struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param chs_servos_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_chs_servos_info_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_chs_servos_info_t* chs_servos_info)
{
    return mavlink_msg_chs_servos_info_pack_chan(system_id, component_id, chan, msg, chs_servos_info->servos);
}

/**
 * @brief Send a chs_servos_info message
 * @param chan MAVLink channel to send the message
 *
 * @param servos  Indicate duty cycle of servo timer. Range from 499 to 24999 indicates 500/20000 to 2500/20000.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_chs_servos_info_send(mavlink_channel_t chan, const uint16_t *servos)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CHS_SERVOS_INFO_LEN];

    _mav_put_uint16_t_array(buf, 0, servos, 7);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHS_SERVOS_INFO, buf, MAVLINK_MSG_ID_CHS_SERVOS_INFO_MIN_LEN, MAVLINK_MSG_ID_CHS_SERVOS_INFO_LEN, MAVLINK_MSG_ID_CHS_SERVOS_INFO_CRC);
#else
    mavlink_chs_servos_info_t packet;

    mav_array_memcpy(packet.servos, servos, sizeof(uint16_t)*7);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHS_SERVOS_INFO, (const char *)&packet, MAVLINK_MSG_ID_CHS_SERVOS_INFO_MIN_LEN, MAVLINK_MSG_ID_CHS_SERVOS_INFO_LEN, MAVLINK_MSG_ID_CHS_SERVOS_INFO_CRC);
#endif
}

/**
 * @brief Send a chs_servos_info message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_chs_servos_info_send_struct(mavlink_channel_t chan, const mavlink_chs_servos_info_t* chs_servos_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_chs_servos_info_send(chan, chs_servos_info->servos);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHS_SERVOS_INFO, (const char *)chs_servos_info, MAVLINK_MSG_ID_CHS_SERVOS_INFO_MIN_LEN, MAVLINK_MSG_ID_CHS_SERVOS_INFO_LEN, MAVLINK_MSG_ID_CHS_SERVOS_INFO_CRC);
#endif
}

#if MAVLINK_MSG_ID_CHS_SERVOS_INFO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_chs_servos_info_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const uint16_t *servos)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _mav_put_uint16_t_array(buf, 0, servos, 7);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHS_SERVOS_INFO, buf, MAVLINK_MSG_ID_CHS_SERVOS_INFO_MIN_LEN, MAVLINK_MSG_ID_CHS_SERVOS_INFO_LEN, MAVLINK_MSG_ID_CHS_SERVOS_INFO_CRC);
#else
    mavlink_chs_servos_info_t *packet = (mavlink_chs_servos_info_t *)msgbuf;

    mav_array_memcpy(packet->servos, servos, sizeof(uint16_t)*7);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHS_SERVOS_INFO, (const char *)packet, MAVLINK_MSG_ID_CHS_SERVOS_INFO_MIN_LEN, MAVLINK_MSG_ID_CHS_SERVOS_INFO_LEN, MAVLINK_MSG_ID_CHS_SERVOS_INFO_CRC);
#endif
}
#endif

#endif

// MESSAGE CHS_SERVOS_INFO UNPACKING


/**
 * @brief Get field servos from chs_servos_info message
 *
 * @return  Indicate duty cycle of servo timer. Range from 499 to 24999 indicates 500/20000 to 2500/20000.
 */
static inline uint16_t mavlink_msg_chs_servos_info_get_servos(const mavlink_message_t* msg, uint16_t *servos)
{
    return _MAV_RETURN_uint16_t_array(msg, servos, 7,  0);
}

/**
 * @brief Decode a chs_servos_info message into a struct
 *
 * @param msg The message to decode
 * @param chs_servos_info C-struct to decode the message contents into
 */
static inline void mavlink_msg_chs_servos_info_decode(const mavlink_message_t* msg, mavlink_chs_servos_info_t* chs_servos_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_chs_servos_info_get_servos(msg, chs_servos_info->servos);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CHS_SERVOS_INFO_LEN? msg->len : MAVLINK_MSG_ID_CHS_SERVOS_INFO_LEN;
        memset(chs_servos_info, 0, MAVLINK_MSG_ID_CHS_SERVOS_INFO_LEN);
    memcpy(chs_servos_info, _MAV_PAYLOAD(msg), len);
#endif
}
