#pragma once
// MESSAGE CHS_CTRL_INFO PACKING

#define MAVLINK_MSG_ID_CHS_CTRL_INFO 0


typedef struct __mavlink_chs_ctrl_info_t {
 float vx; /*< [m/s] Indicate velocity of x direction. Range [-2.0, 2.0]*/
 float vy; /*< [m/s] Indicate velocity of y direction. Range [-2.0, 2.0]*/
 float vw; /*< [rad/s] Indicate angular velocity of w direction. Range [-2*PI, 2*PI]*/
} mavlink_chs_ctrl_info_t;

#define MAVLINK_MSG_ID_CHS_CTRL_INFO_LEN 12
#define MAVLINK_MSG_ID_CHS_CTRL_INFO_MIN_LEN 12
#define MAVLINK_MSG_ID_0_LEN 12
#define MAVLINK_MSG_ID_0_MIN_LEN 12

#define MAVLINK_MSG_ID_CHS_CTRL_INFO_CRC 136
#define MAVLINK_MSG_ID_0_CRC 136



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CHS_CTRL_INFO { \
    0, \
    "CHS_CTRL_INFO", \
    3, \
    {  { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_chs_ctrl_info_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_chs_ctrl_info_t, vy) }, \
         { "vw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_chs_ctrl_info_t, vw) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CHS_CTRL_INFO { \
    "CHS_CTRL_INFO", \
    3, \
    {  { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_chs_ctrl_info_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_chs_ctrl_info_t, vy) }, \
         { "vw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_chs_ctrl_info_t, vw) }, \
         } \
}
#endif

/**
 * @brief Pack a chs_ctrl_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param vx [m/s] Indicate velocity of x direction. Range [-2.0, 2.0]
 * @param vy [m/s] Indicate velocity of y direction. Range [-2.0, 2.0]
 * @param vw [rad/s] Indicate angular velocity of w direction. Range [-2*PI, 2*PI]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_chs_ctrl_info_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float vx, float vy, float vw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CHS_CTRL_INFO_LEN];
    _mav_put_float(buf, 0, vx);
    _mav_put_float(buf, 4, vy);
    _mav_put_float(buf, 8, vw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CHS_CTRL_INFO_LEN);
#else
    mavlink_chs_ctrl_info_t packet;
    packet.vx = vx;
    packet.vy = vy;
    packet.vw = vw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CHS_CTRL_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CHS_CTRL_INFO;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CHS_CTRL_INFO_MIN_LEN, MAVLINK_MSG_ID_CHS_CTRL_INFO_LEN, MAVLINK_MSG_ID_CHS_CTRL_INFO_CRC);
}

/**
 * @brief Pack a chs_ctrl_info message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param vx [m/s] Indicate velocity of x direction. Range [-2.0, 2.0]
 * @param vy [m/s] Indicate velocity of y direction. Range [-2.0, 2.0]
 * @param vw [rad/s] Indicate angular velocity of w direction. Range [-2*PI, 2*PI]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_chs_ctrl_info_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float vx,float vy,float vw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CHS_CTRL_INFO_LEN];
    _mav_put_float(buf, 0, vx);
    _mav_put_float(buf, 4, vy);
    _mav_put_float(buf, 8, vw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CHS_CTRL_INFO_LEN);
#else
    mavlink_chs_ctrl_info_t packet;
    packet.vx = vx;
    packet.vy = vy;
    packet.vw = vw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CHS_CTRL_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CHS_CTRL_INFO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CHS_CTRL_INFO_MIN_LEN, MAVLINK_MSG_ID_CHS_CTRL_INFO_LEN, MAVLINK_MSG_ID_CHS_CTRL_INFO_CRC);
}

/**
 * @brief Encode a chs_ctrl_info struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param chs_ctrl_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_chs_ctrl_info_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_chs_ctrl_info_t* chs_ctrl_info)
{
    return mavlink_msg_chs_ctrl_info_pack(system_id, component_id, msg, chs_ctrl_info->vx, chs_ctrl_info->vy, chs_ctrl_info->vw);
}

/**
 * @brief Encode a chs_ctrl_info struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param chs_ctrl_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_chs_ctrl_info_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_chs_ctrl_info_t* chs_ctrl_info)
{
    return mavlink_msg_chs_ctrl_info_pack_chan(system_id, component_id, chan, msg, chs_ctrl_info->vx, chs_ctrl_info->vy, chs_ctrl_info->vw);
}

/**
 * @brief Send a chs_ctrl_info message
 * @param chan MAVLink channel to send the message
 *
 * @param vx [m/s] Indicate velocity of x direction. Range [-2.0, 2.0]
 * @param vy [m/s] Indicate velocity of y direction. Range [-2.0, 2.0]
 * @param vw [rad/s] Indicate angular velocity of w direction. Range [-2*PI, 2*PI]
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_chs_ctrl_info_send(mavlink_channel_t chan, float vx, float vy, float vw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CHS_CTRL_INFO_LEN];
    _mav_put_float(buf, 0, vx);
    _mav_put_float(buf, 4, vy);
    _mav_put_float(buf, 8, vw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHS_CTRL_INFO, buf, MAVLINK_MSG_ID_CHS_CTRL_INFO_MIN_LEN, MAVLINK_MSG_ID_CHS_CTRL_INFO_LEN, MAVLINK_MSG_ID_CHS_CTRL_INFO_CRC);
#else
    mavlink_chs_ctrl_info_t packet;
    packet.vx = vx;
    packet.vy = vy;
    packet.vw = vw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHS_CTRL_INFO, (const char *)&packet, MAVLINK_MSG_ID_CHS_CTRL_INFO_MIN_LEN, MAVLINK_MSG_ID_CHS_CTRL_INFO_LEN, MAVLINK_MSG_ID_CHS_CTRL_INFO_CRC);
#endif
}

/**
 * @brief Send a chs_ctrl_info message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_chs_ctrl_info_send_struct(mavlink_channel_t chan, const mavlink_chs_ctrl_info_t* chs_ctrl_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_chs_ctrl_info_send(chan, chs_ctrl_info->vx, chs_ctrl_info->vy, chs_ctrl_info->vw);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHS_CTRL_INFO, (const char *)chs_ctrl_info, MAVLINK_MSG_ID_CHS_CTRL_INFO_MIN_LEN, MAVLINK_MSG_ID_CHS_CTRL_INFO_LEN, MAVLINK_MSG_ID_CHS_CTRL_INFO_CRC);
#endif
}

#if MAVLINK_MSG_ID_CHS_CTRL_INFO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_chs_ctrl_info_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float vx, float vy, float vw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, vx);
    _mav_put_float(buf, 4, vy);
    _mav_put_float(buf, 8, vw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHS_CTRL_INFO, buf, MAVLINK_MSG_ID_CHS_CTRL_INFO_MIN_LEN, MAVLINK_MSG_ID_CHS_CTRL_INFO_LEN, MAVLINK_MSG_ID_CHS_CTRL_INFO_CRC);
#else
    mavlink_chs_ctrl_info_t *packet = (mavlink_chs_ctrl_info_t *)msgbuf;
    packet->vx = vx;
    packet->vy = vy;
    packet->vw = vw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHS_CTRL_INFO, (const char *)packet, MAVLINK_MSG_ID_CHS_CTRL_INFO_MIN_LEN, MAVLINK_MSG_ID_CHS_CTRL_INFO_LEN, MAVLINK_MSG_ID_CHS_CTRL_INFO_CRC);
#endif
}
#endif

#endif

// MESSAGE CHS_CTRL_INFO UNPACKING


/**
 * @brief Get field vx from chs_ctrl_info message
 *
 * @return [m/s] Indicate velocity of x direction. Range [-2.0, 2.0]
 */
static inline float mavlink_msg_chs_ctrl_info_get_vx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field vy from chs_ctrl_info message
 *
 * @return [m/s] Indicate velocity of y direction. Range [-2.0, 2.0]
 */
static inline float mavlink_msg_chs_ctrl_info_get_vy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field vw from chs_ctrl_info message
 *
 * @return [rad/s] Indicate angular velocity of w direction. Range [-2*PI, 2*PI]
 */
static inline float mavlink_msg_chs_ctrl_info_get_vw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a chs_ctrl_info message into a struct
 *
 * @param msg The message to decode
 * @param chs_ctrl_info C-struct to decode the message contents into
 */
static inline void mavlink_msg_chs_ctrl_info_decode(const mavlink_message_t* msg, mavlink_chs_ctrl_info_t* chs_ctrl_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    chs_ctrl_info->vx = mavlink_msg_chs_ctrl_info_get_vx(msg);
    chs_ctrl_info->vy = mavlink_msg_chs_ctrl_info_get_vy(msg);
    chs_ctrl_info->vw = mavlink_msg_chs_ctrl_info_get_vw(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CHS_CTRL_INFO_LEN? msg->len : MAVLINK_MSG_ID_CHS_CTRL_INFO_LEN;
        memset(chs_ctrl_info, 0, MAVLINK_MSG_ID_CHS_CTRL_INFO_LEN);
    memcpy(chs_ctrl_info, _MAV_PAYLOAD(msg), len);
#endif
}
