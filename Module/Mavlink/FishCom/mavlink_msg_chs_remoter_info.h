#pragma once
// MESSAGE CHS_REMOTER_INFO PACKING

#define MAVLINK_MSG_ID_CHS_REMOTER_INFO 6


typedef struct __mavlink_chs_remoter_info_t {
 int16_t channel_0; /*<  -660 to +660. Right-Upward is orientation.*/
 int16_t channel_1; /*<  -660 to +660. Right-Upward is orientation.*/
 int16_t channel_2; /*<  -660 to +660. Right-Upward is orientation.*/
 int16_t channel_3; /*<  -660 to +660. Right-Upward is orientation.*/
 int16_t wheel; /*<  -660 to +660. Counter-clockwise is orientation.*/
 uint8_t switch_messgae; /*<  Bit-0:isOnline(0-offline,1-online), Bit[1-2]:SwitchLeft(0-0b00,1-0b01,2-0x10), Bit[3-4]:SwitchRight(0-0b00,1-0b01,2-0x10)*/
} mavlink_chs_remoter_info_t;

#define MAVLINK_MSG_ID_CHS_REMOTER_INFO_LEN 11
#define MAVLINK_MSG_ID_CHS_REMOTER_INFO_MIN_LEN 11
#define MAVLINK_MSG_ID_6_LEN 11
#define MAVLINK_MSG_ID_6_MIN_LEN 11

#define MAVLINK_MSG_ID_CHS_REMOTER_INFO_CRC 250
#define MAVLINK_MSG_ID_6_CRC 250



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CHS_REMOTER_INFO { \
    6, \
    "CHS_REMOTER_INFO", \
    6, \
    {  { "switch_messgae", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_chs_remoter_info_t, switch_messgae) }, \
         { "channel_0", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_chs_remoter_info_t, channel_0) }, \
         { "channel_1", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_chs_remoter_info_t, channel_1) }, \
         { "channel_2", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_chs_remoter_info_t, channel_2) }, \
         { "channel_3", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_chs_remoter_info_t, channel_3) }, \
         { "wheel", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_chs_remoter_info_t, wheel) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CHS_REMOTER_INFO { \
    "CHS_REMOTER_INFO", \
    6, \
    {  { "switch_messgae", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_chs_remoter_info_t, switch_messgae) }, \
         { "channel_0", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_chs_remoter_info_t, channel_0) }, \
         { "channel_1", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_chs_remoter_info_t, channel_1) }, \
         { "channel_2", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_chs_remoter_info_t, channel_2) }, \
         { "channel_3", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_chs_remoter_info_t, channel_3) }, \
         { "wheel", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_chs_remoter_info_t, wheel) }, \
         } \
}
#endif

/**
 * @brief Pack a chs_remoter_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param switch_messgae  Bit-0:isOnline(0-offline,1-online), Bit[1-2]:SwitchLeft(0-0b00,1-0b01,2-0x10), Bit[3-4]:SwitchRight(0-0b00,1-0b01,2-0x10)
 * @param channel_0  -660 to +660. Right-Upward is orientation.
 * @param channel_1  -660 to +660. Right-Upward is orientation.
 * @param channel_2  -660 to +660. Right-Upward is orientation.
 * @param channel_3  -660 to +660. Right-Upward is orientation.
 * @param wheel  -660 to +660. Counter-clockwise is orientation.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_chs_remoter_info_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t switch_messgae, int16_t channel_0, int16_t channel_1, int16_t channel_2, int16_t channel_3, int16_t wheel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CHS_REMOTER_INFO_LEN];
    _mav_put_int16_t(buf, 0, channel_0);
    _mav_put_int16_t(buf, 2, channel_1);
    _mav_put_int16_t(buf, 4, channel_2);
    _mav_put_int16_t(buf, 6, channel_3);
    _mav_put_int16_t(buf, 8, wheel);
    _mav_put_uint8_t(buf, 10, switch_messgae);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CHS_REMOTER_INFO_LEN);
#else
    mavlink_chs_remoter_info_t packet;
    packet.channel_0 = channel_0;
    packet.channel_1 = channel_1;
    packet.channel_2 = channel_2;
    packet.channel_3 = channel_3;
    packet.wheel = wheel;
    packet.switch_messgae = switch_messgae;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CHS_REMOTER_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CHS_REMOTER_INFO;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CHS_REMOTER_INFO_MIN_LEN, MAVLINK_MSG_ID_CHS_REMOTER_INFO_LEN, MAVLINK_MSG_ID_CHS_REMOTER_INFO_CRC);
}

/**
 * @brief Pack a chs_remoter_info message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param switch_messgae  Bit-0:isOnline(0-offline,1-online), Bit[1-2]:SwitchLeft(0-0b00,1-0b01,2-0x10), Bit[3-4]:SwitchRight(0-0b00,1-0b01,2-0x10)
 * @param channel_0  -660 to +660. Right-Upward is orientation.
 * @param channel_1  -660 to +660. Right-Upward is orientation.
 * @param channel_2  -660 to +660. Right-Upward is orientation.
 * @param channel_3  -660 to +660. Right-Upward is orientation.
 * @param wheel  -660 to +660. Counter-clockwise is orientation.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_chs_remoter_info_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t switch_messgae,int16_t channel_0,int16_t channel_1,int16_t channel_2,int16_t channel_3,int16_t wheel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CHS_REMOTER_INFO_LEN];
    _mav_put_int16_t(buf, 0, channel_0);
    _mav_put_int16_t(buf, 2, channel_1);
    _mav_put_int16_t(buf, 4, channel_2);
    _mav_put_int16_t(buf, 6, channel_3);
    _mav_put_int16_t(buf, 8, wheel);
    _mav_put_uint8_t(buf, 10, switch_messgae);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CHS_REMOTER_INFO_LEN);
#else
    mavlink_chs_remoter_info_t packet;
    packet.channel_0 = channel_0;
    packet.channel_1 = channel_1;
    packet.channel_2 = channel_2;
    packet.channel_3 = channel_3;
    packet.wheel = wheel;
    packet.switch_messgae = switch_messgae;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CHS_REMOTER_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CHS_REMOTER_INFO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CHS_REMOTER_INFO_MIN_LEN, MAVLINK_MSG_ID_CHS_REMOTER_INFO_LEN, MAVLINK_MSG_ID_CHS_REMOTER_INFO_CRC);
}

/**
 * @brief Encode a chs_remoter_info struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param chs_remoter_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_chs_remoter_info_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_chs_remoter_info_t* chs_remoter_info)
{
    return mavlink_msg_chs_remoter_info_pack(system_id, component_id, msg, chs_remoter_info->switch_messgae, chs_remoter_info->channel_0, chs_remoter_info->channel_1, chs_remoter_info->channel_2, chs_remoter_info->channel_3, chs_remoter_info->wheel);
}

/**
 * @brief Encode a chs_remoter_info struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param chs_remoter_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_chs_remoter_info_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_chs_remoter_info_t* chs_remoter_info)
{
    return mavlink_msg_chs_remoter_info_pack_chan(system_id, component_id, chan, msg, chs_remoter_info->switch_messgae, chs_remoter_info->channel_0, chs_remoter_info->channel_1, chs_remoter_info->channel_2, chs_remoter_info->channel_3, chs_remoter_info->wheel);
}

/**
 * @brief Send a chs_remoter_info message
 * @param chan MAVLink channel to send the message
 *
 * @param switch_messgae  Bit-0:isOnline(0-offline,1-online), Bit[1-2]:SwitchLeft(0-0b00,1-0b01,2-0x10), Bit[3-4]:SwitchRight(0-0b00,1-0b01,2-0x10)
 * @param channel_0  -660 to +660. Right-Upward is orientation.
 * @param channel_1  -660 to +660. Right-Upward is orientation.
 * @param channel_2  -660 to +660. Right-Upward is orientation.
 * @param channel_3  -660 to +660. Right-Upward is orientation.
 * @param wheel  -660 to +660. Counter-clockwise is orientation.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_chs_remoter_info_send(mavlink_channel_t chan, uint8_t switch_messgae, int16_t channel_0, int16_t channel_1, int16_t channel_2, int16_t channel_3, int16_t wheel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CHS_REMOTER_INFO_LEN];
    _mav_put_int16_t(buf, 0, channel_0);
    _mav_put_int16_t(buf, 2, channel_1);
    _mav_put_int16_t(buf, 4, channel_2);
    _mav_put_int16_t(buf, 6, channel_3);
    _mav_put_int16_t(buf, 8, wheel);
    _mav_put_uint8_t(buf, 10, switch_messgae);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHS_REMOTER_INFO, buf, MAVLINK_MSG_ID_CHS_REMOTER_INFO_MIN_LEN, MAVLINK_MSG_ID_CHS_REMOTER_INFO_LEN, MAVLINK_MSG_ID_CHS_REMOTER_INFO_CRC);
#else
    mavlink_chs_remoter_info_t packet;
    packet.channel_0 = channel_0;
    packet.channel_1 = channel_1;
    packet.channel_2 = channel_2;
    packet.channel_3 = channel_3;
    packet.wheel = wheel;
    packet.switch_messgae = switch_messgae;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHS_REMOTER_INFO, (const char *)&packet, MAVLINK_MSG_ID_CHS_REMOTER_INFO_MIN_LEN, MAVLINK_MSG_ID_CHS_REMOTER_INFO_LEN, MAVLINK_MSG_ID_CHS_REMOTER_INFO_CRC);
#endif
}

/**
 * @brief Send a chs_remoter_info message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_chs_remoter_info_send_struct(mavlink_channel_t chan, const mavlink_chs_remoter_info_t* chs_remoter_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_chs_remoter_info_send(chan, chs_remoter_info->switch_messgae, chs_remoter_info->channel_0, chs_remoter_info->channel_1, chs_remoter_info->channel_2, chs_remoter_info->channel_3, chs_remoter_info->wheel);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHS_REMOTER_INFO, (const char *)chs_remoter_info, MAVLINK_MSG_ID_CHS_REMOTER_INFO_MIN_LEN, MAVLINK_MSG_ID_CHS_REMOTER_INFO_LEN, MAVLINK_MSG_ID_CHS_REMOTER_INFO_CRC);
#endif
}

#if MAVLINK_MSG_ID_CHS_REMOTER_INFO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_chs_remoter_info_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t switch_messgae, int16_t channel_0, int16_t channel_1, int16_t channel_2, int16_t channel_3, int16_t wheel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int16_t(buf, 0, channel_0);
    _mav_put_int16_t(buf, 2, channel_1);
    _mav_put_int16_t(buf, 4, channel_2);
    _mav_put_int16_t(buf, 6, channel_3);
    _mav_put_int16_t(buf, 8, wheel);
    _mav_put_uint8_t(buf, 10, switch_messgae);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHS_REMOTER_INFO, buf, MAVLINK_MSG_ID_CHS_REMOTER_INFO_MIN_LEN, MAVLINK_MSG_ID_CHS_REMOTER_INFO_LEN, MAVLINK_MSG_ID_CHS_REMOTER_INFO_CRC);
#else
    mavlink_chs_remoter_info_t *packet = (mavlink_chs_remoter_info_t *)msgbuf;
    packet->channel_0 = channel_0;
    packet->channel_1 = channel_1;
    packet->channel_2 = channel_2;
    packet->channel_3 = channel_3;
    packet->wheel = wheel;
    packet->switch_messgae = switch_messgae;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHS_REMOTER_INFO, (const char *)packet, MAVLINK_MSG_ID_CHS_REMOTER_INFO_MIN_LEN, MAVLINK_MSG_ID_CHS_REMOTER_INFO_LEN, MAVLINK_MSG_ID_CHS_REMOTER_INFO_CRC);
#endif
}
#endif

#endif

// MESSAGE CHS_REMOTER_INFO UNPACKING


/**
 * @brief Get field switch_messgae from chs_remoter_info message
 *
 * @return  Bit-0:isOnline(0-offline,1-online), Bit[1-2]:SwitchLeft(0-0b00,1-0b01,2-0x10), Bit[3-4]:SwitchRight(0-0b00,1-0b01,2-0x10)
 */
static inline uint8_t mavlink_msg_chs_remoter_info_get_switch_messgae(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field channel_0 from chs_remoter_info message
 *
 * @return  -660 to +660. Right-Upward is orientation.
 */
static inline int16_t mavlink_msg_chs_remoter_info_get_channel_0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field channel_1 from chs_remoter_info message
 *
 * @return  -660 to +660. Right-Upward is orientation.
 */
static inline int16_t mavlink_msg_chs_remoter_info_get_channel_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  2);
}

/**
 * @brief Get field channel_2 from chs_remoter_info message
 *
 * @return  -660 to +660. Right-Upward is orientation.
 */
static inline int16_t mavlink_msg_chs_remoter_info_get_channel_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  4);
}

/**
 * @brief Get field channel_3 from chs_remoter_info message
 *
 * @return  -660 to +660. Right-Upward is orientation.
 */
static inline int16_t mavlink_msg_chs_remoter_info_get_channel_3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  6);
}

/**
 * @brief Get field wheel from chs_remoter_info message
 *
 * @return  -660 to +660. Counter-clockwise is orientation.
 */
static inline int16_t mavlink_msg_chs_remoter_info_get_wheel(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Decode a chs_remoter_info message into a struct
 *
 * @param msg The message to decode
 * @param chs_remoter_info C-struct to decode the message contents into
 */
static inline void mavlink_msg_chs_remoter_info_decode(const mavlink_message_t* msg, mavlink_chs_remoter_info_t* chs_remoter_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    chs_remoter_info->channel_0 = mavlink_msg_chs_remoter_info_get_channel_0(msg);
    chs_remoter_info->channel_1 = mavlink_msg_chs_remoter_info_get_channel_1(msg);
    chs_remoter_info->channel_2 = mavlink_msg_chs_remoter_info_get_channel_2(msg);
    chs_remoter_info->channel_3 = mavlink_msg_chs_remoter_info_get_channel_3(msg);
    chs_remoter_info->wheel = mavlink_msg_chs_remoter_info_get_wheel(msg);
    chs_remoter_info->switch_messgae = mavlink_msg_chs_remoter_info_get_switch_messgae(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CHS_REMOTER_INFO_LEN? msg->len : MAVLINK_MSG_ID_CHS_REMOTER_INFO_LEN;
        memset(chs_remoter_info, 0, MAVLINK_MSG_ID_CHS_REMOTER_INFO_LEN);
    memcpy(chs_remoter_info, _MAV_PAYLOAD(msg), len);
#endif
}
