/** @file
 *    @brief MAVLink comm protocol testsuite generated from FishCom.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef FISHCOM_TESTSUITE_H
#define FISHCOM_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_FishCom(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

    mavlink_test_FishCom(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_chs_ctrl_info(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CHS_CTRL_INFO >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_chs_ctrl_info_t packet_in = {
        17.0,45.0,73.0
    };
    mavlink_chs_ctrl_info_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.vx = packet_in.vx;
        packet1.vy = packet_in.vy;
        packet1.vw = packet_in.vw;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CHS_CTRL_INFO_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CHS_CTRL_INFO_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_chs_ctrl_info_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_chs_ctrl_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_chs_ctrl_info_pack(system_id, component_id, &msg , packet1.vx , packet1.vy , packet1.vw );
    mavlink_msg_chs_ctrl_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_chs_ctrl_info_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.vx , packet1.vy , packet1.vw );
    mavlink_msg_chs_ctrl_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_chs_ctrl_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_chs_ctrl_info_send(MAVLINK_COMM_1 , packet1.vx , packet1.vy , packet1.vw );
    mavlink_msg_chs_ctrl_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("CHS_CTRL_INFO") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_CHS_CTRL_INFO) != NULL);
#endif
}

static void mavlink_test_chs_motor_info(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CHS_MOTOR_INFO >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_chs_motor_info_t packet_in = {
        { 17235, 17236, 17237, 17238 }
    };
    mavlink_chs_motor_info_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        mav_array_memcpy(packet1.motor, packet_in.motor, sizeof(int16_t)*4);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CHS_MOTOR_INFO_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CHS_MOTOR_INFO_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_chs_motor_info_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_chs_motor_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_chs_motor_info_pack(system_id, component_id, &msg , packet1.motor );
    mavlink_msg_chs_motor_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_chs_motor_info_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.motor );
    mavlink_msg_chs_motor_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_chs_motor_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_chs_motor_info_send(MAVLINK_COMM_1 , packet1.motor );
    mavlink_msg_chs_motor_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("CHS_MOTOR_INFO") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_CHS_MOTOR_INFO) != NULL);
#endif
}

static void mavlink_test_chs_odom_info(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CHS_ODOM_INFO >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_chs_odom_info_t packet_in = {
        17.0,45.0,73.0,{ 101.0, 102.0, 103.0, 104.0 }
    };
    mavlink_chs_odom_info_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.vx = packet_in.vx;
        packet1.vy = packet_in.vy;
        packet1.vw = packet_in.vw;
        
        mav_array_memcpy(packet1.quaternion, packet_in.quaternion, sizeof(float)*4);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CHS_ODOM_INFO_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CHS_ODOM_INFO_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_chs_odom_info_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_chs_odom_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_chs_odom_info_pack(system_id, component_id, &msg , packet1.vx , packet1.vy , packet1.vw , packet1.quaternion );
    mavlink_msg_chs_odom_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_chs_odom_info_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.vx , packet1.vy , packet1.vw , packet1.quaternion );
    mavlink_msg_chs_odom_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_chs_odom_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_chs_odom_info_send(MAVLINK_COMM_1 , packet1.vx , packet1.vy , packet1.vw , packet1.quaternion );
    mavlink_msg_chs_odom_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("CHS_ODOM_INFO") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_CHS_ODOM_INFO) != NULL);
#endif
}

static void mavlink_test_chs_imu_info(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CHS_IMU_INFO >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_chs_imu_info_t packet_in = {
        { 17.0, 18.0, 19.0 },{ 101.0, 102.0, 103.0 }
    };
    mavlink_chs_imu_info_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        mav_array_memcpy(packet1.accel, packet_in.accel, sizeof(float)*3);
        mav_array_memcpy(packet1.gyro, packet_in.gyro, sizeof(float)*3);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CHS_IMU_INFO_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CHS_IMU_INFO_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_chs_imu_info_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_chs_imu_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_chs_imu_info_pack(system_id, component_id, &msg , packet1.accel , packet1.gyro );
    mavlink_msg_chs_imu_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_chs_imu_info_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.accel , packet1.gyro );
    mavlink_msg_chs_imu_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_chs_imu_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_chs_imu_info_send(MAVLINK_COMM_1 , packet1.accel , packet1.gyro );
    mavlink_msg_chs_imu_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("CHS_IMU_INFO") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_CHS_IMU_INFO) != NULL);
#endif
}

static void mavlink_test_chs_servos_info(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CHS_SERVOS_INFO >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_chs_servos_info_t packet_in = {
        { 17235, 17236, 17237, 17238, 17239, 17240, 17241 }
    };
    mavlink_chs_servos_info_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        mav_array_memcpy(packet1.servos, packet_in.servos, sizeof(uint16_t)*7);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CHS_SERVOS_INFO_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CHS_SERVOS_INFO_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_chs_servos_info_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_chs_servos_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_chs_servos_info_pack(system_id, component_id, &msg , packet1.servos );
    mavlink_msg_chs_servos_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_chs_servos_info_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.servos );
    mavlink_msg_chs_servos_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_chs_servos_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_chs_servos_info_send(MAVLINK_COMM_1 , packet1.servos );
    mavlink_msg_chs_servos_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("CHS_SERVOS_INFO") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_CHS_SERVOS_INFO) != NULL);
#endif
}

static void mavlink_test_chs_manage_info(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CHS_MANAGE_INFO >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_chs_manage_info_t packet_in = {
        5,72,139
    };
    mavlink_chs_manage_info_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.enable_chassis = packet_in.enable_chassis;
        packet1.enable_servos = packet_in.enable_servos;
        packet1.reset_quaternion = packet_in.reset_quaternion;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CHS_MANAGE_INFO_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CHS_MANAGE_INFO_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_chs_manage_info_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_chs_manage_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_chs_manage_info_pack(system_id, component_id, &msg , packet1.enable_chassis , packet1.enable_servos , packet1.reset_quaternion );
    mavlink_msg_chs_manage_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_chs_manage_info_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.enable_chassis , packet1.enable_servos , packet1.reset_quaternion );
    mavlink_msg_chs_manage_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_chs_manage_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_chs_manage_info_send(MAVLINK_COMM_1 , packet1.enable_chassis , packet1.enable_servos , packet1.reset_quaternion );
    mavlink_msg_chs_manage_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("CHS_MANAGE_INFO") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_CHS_MANAGE_INFO) != NULL);
#endif
}

static void mavlink_test_chs_remoter_info(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CHS_REMOTER_INFO >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_chs_remoter_info_t packet_in = {
        17235,17339,17443,17547,17651,163
    };
    mavlink_chs_remoter_info_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.channel_0 = packet_in.channel_0;
        packet1.channel_1 = packet_in.channel_1;
        packet1.channel_2 = packet_in.channel_2;
        packet1.channel_3 = packet_in.channel_3;
        packet1.wheel = packet_in.wheel;
        packet1.switch_messgae = packet_in.switch_messgae;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CHS_REMOTER_INFO_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CHS_REMOTER_INFO_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_chs_remoter_info_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_chs_remoter_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_chs_remoter_info_pack(system_id, component_id, &msg , packet1.switch_messgae , packet1.channel_0 , packet1.channel_1 , packet1.channel_2 , packet1.channel_3 , packet1.wheel );
    mavlink_msg_chs_remoter_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_chs_remoter_info_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.switch_messgae , packet1.channel_0 , packet1.channel_1 , packet1.channel_2 , packet1.channel_3 , packet1.wheel );
    mavlink_msg_chs_remoter_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_chs_remoter_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_chs_remoter_info_send(MAVLINK_COMM_1 , packet1.switch_messgae , packet1.channel_0 , packet1.channel_1 , packet1.channel_2 , packet1.channel_3 , packet1.wheel );
    mavlink_msg_chs_remoter_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("CHS_REMOTER_INFO") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_CHS_REMOTER_INFO) != NULL);
#endif
}

static void mavlink_test_FishCom(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_chs_ctrl_info(system_id, component_id, last_msg);
    mavlink_test_chs_motor_info(system_id, component_id, last_msg);
    mavlink_test_chs_odom_info(system_id, component_id, last_msg);
    mavlink_test_chs_imu_info(system_id, component_id, last_msg);
    mavlink_test_chs_servos_info(system_id, component_id, last_msg);
    mavlink_test_chs_manage_info(system_id, component_id, last_msg);
    mavlink_test_chs_remoter_info(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // FISHCOM_TESTSUITE_H
