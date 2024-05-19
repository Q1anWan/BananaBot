///*
// * @Description: API for Upper
// * @Author: qianwan
// * @Date: 2023-09-29 14:19:38
// * @LastEditTime: 2023-12-18 23:30:25
// * @LastEditors: qianwan
// */
//#include "main.h"
//#include "spi.h"
//#include "app_usbx_device.h"
//#include "ux_device_class_cdc_acm.h"
//#include "ServiceMsgAPI.h"
//#include "TaskWheel.h"
//#include "CRC8.h"
//
//#include "om.h"
//#include "FishMessage.h"
//#include "mavlink.h"
//
//#define MSG_SPI_FLAG 0xAB
//#define MSG_SPI_MAX_LOSE 2
//#define MSG_DECNT_TIM 25
//#define MSG_RECNT_TIM 30
//#define USB_MAX_LEN UX_SLAVE_REQUEST_DATA_MAX_LENGTH
//#define MSG_SPI_TOTAL_TX_LEN (MAVLINK_MSG_ID_CHS_ODOM_INFO_LEN + MSG_MOTOR_EXTERN_FDB_LEN + MAVLINK_MSG_ID_CHS_REMOTER_INFO_LEN )
//#define MSG_SPI_TOTAL_RX_LEN (MSG_MOTOR_EXTERN_CTRL_LEN + MAVLINK_MSG_ID_CHS_SERVOS_INFO_LEN + MAVLINK_MSG_ID_CHS_MANAGE_INFO_LEN)
//
//#if (MSG_SPI_TOTAL_TX_LEN > MSG_SPI_TOTAL_RX_LEN)
//#define MSG_SPI_LEN (MSG_SPI_TOTAL_TX_LEN + 1)
//#else
//#define MSG_SPI_LEN (MSG_SPI_TOTAL_RX_LEN + 1)
//#endif
//
////#define ASSIGN_CONTROL_RIGHT
//#ifdef ASSIGN_CONTROL_RIGHT
//#define ASSIGH_CHASSIS_RIGHT false //false:SPI true:USB
//#define ASSIGH_SERVO_RIGHT  false  //false:SPI true:USB
//#endif
//
//[[noreturn]] static void Msg_Fault();
//
//extern TX_BYTE_POOL ComPool;
//SRAM_SET_CCM_UNINT bool imu_rst = false;
//
//SRAM_SET_CCM_UNINT static mavlink_message_t msg_tx;
//SRAM_SET_CCM_UNINT static mavlink_message_t msg_rx;
//
//SRAM_SET_CCM_UNINT static mavlink_chs_odom_info_t *odom_buf_now = nullptr;
//SRAM_SET_CCM_UNINT static mavlink_chs_odom_info_t chs_odom_info[2];
//SRAM_SET_CCM_UNINT static mavlink_chs_imu_info_t chs_imu_info;
//SRAM_SET_CCM_UNINT static mavlink_chs_remoter_info_t chs_remoter_info;
//SRAM_SET_CCM_UNINT static Msg_MotorExternFDB_t chs_motor_extern_fdb_info;
//
//
//SRAM_SET_CCM_UNINT Msg_spi_rx_data_processed_t spi_rx_data_processed;
//
//SRAM_SET_CCM_UNINT static uint8_t *usb_buf_now = nullptr;
//SRAM_SET_CCM_UNINT static uint16_t usb_tx_len = 0;
//
//SRAM_SET_CCM_UNINT static uint8_t usb_tx_buf[2][384];
//SRAM_SET_CCM_UNINT static uint8_t usb_rx_buf[USB_MAX_LEN];
//SRAM_SET_CCM_UNINT Msg_usb_rx_data_processed_t usb_rx_data_processed;
//
//SRAM_SET_CCM_UNINT static bool control_right_chassis = false; //true: CDC false: SPI
//SRAM_SET_CCM_UNINT static bool control_right_servo = false; //true: CDC false: SPI
//
//SRAM_SET_CCM TX_THREAD MsgSchedulerThread;
//SRAM_SET_CCM TX_SEMAPHORE MsgCDCSem;
//SRAM_SET_CCM uint8_t MsgSchedulerStack[1536] = {0};
//
//[[noreturn]] void MsgSchedulerFun(ULONG initial_input) {
//    UNUSED(initial_input);
//    om_suber_t *sub_ins = om_subscribe(om_find_topic("INS", UINT32_MAX));
//    om_suber_t *sub_wheel = om_subscribe(om_find_topic("WheelFDB", UINT32_MAX));
//    om_suber_t *sub_remoter = om_subscribe(om_find_topic("Remoter", UINT32_MAX));
//    om_topic_t *pup_wheel = om_find_topic("WheelControl", UINT32_MAX);
//    om_topic_t *pub_servo = om_find_topic("Servo", UINT32_MAX);
//
//    //Receive Topic
//    Msg_INS_t msg_ins{};
//    Msg_WheelFDB_t msg_wheel_fdb{};
//    Msg_Remoter_t msg_remoter{};
//
//    //Transmit Topic
//    Msg_WheelControl_t msg_wheel_ctrl{};
//    Msg_Servo_t msg_servo{};
//
//    bool _control_right_chassis = false;
//    bool _control_right_servo = false;
//    bool _control_right_chassis_last = false;
//    bool _control_right_servo_last = false;
//
//
//    UX_SLAVE_DEVICE *device;
//
//    odom_buf_now = chs_odom_info;
//    usb_buf_now = usb_tx_buf[0];
//
//    while (true) {
//        device = &_ux_system_slave->ux_system_slave_device;
//
//        /*Message Transmission*/
//        om_suber_export(sub_ins, &msg_ins, false);
//        om_suber_export(sub_wheel, &msg_wheel_fdb, false);
//        om_suber_export(sub_remoter, &msg_remoter, false);
//
//        /*Set odometer information*/
//        odom_buf_now = (odom_buf_now == chs_odom_info) ? chs_odom_info + 1 : chs_odom_info;
//        memcpy(odom_buf_now->quaternion, msg_ins.quaternion, 4 * sizeof(float));
//        odom_buf_now->vx =
//                0.25f * (msg_wheel_fdb.mps[0] + msg_wheel_fdb.mps[1] - msg_wheel_fdb.mps[2] - msg_wheel_fdb.mps[3]);
//        odom_buf_now->vy =
//                0.25f * (-msg_wheel_fdb.mps[0] + msg_wheel_fdb.mps[1] + msg_wheel_fdb.mps[2] - msg_wheel_fdb.mps[3]);
//        odom_buf_now->vw = msg_ins.gyro[2];
//
//        chs_motor_extern_fdb_info.motor[0] = msg_wheel_fdb.extern_rpm[0];
//        chs_motor_extern_fdb_info.motor[1] = msg_wheel_fdb.extern_rpm[1];
//
//        /*Set imu information*/
//        chs_imu_info.accel[0]=msg_ins.accel[0];
//        chs_imu_info.accel[1]=msg_ins.accel[1];
//        chs_imu_info.accel[2]=msg_ins.accel[2];
//        chs_imu_info.gyro[0]=msg_ins.gyro[0];
//        chs_imu_info.gyro[1]=msg_ins.gyro[1];
//        chs_imu_info.gyro[2]=msg_ins.gyro[2];
//
//        /*Set remoter information*/
//        if(msg_remoter.Online){
//            chs_remoter_info.switch_messgae = (msg_remoter.Online&0x01) | ((msg_remoter.switch_left << 1)&0x06) | ((msg_remoter.switch_right << 3)&0x18);
//            chs_remoter_info.channel_0 = msg_remoter.ch_0;
//            chs_remoter_info.channel_1 = msg_remoter.ch_1;
//            chs_remoter_info.channel_2 = msg_remoter.ch_2;
//            chs_remoter_info.channel_3 = msg_remoter.ch_3;
//            chs_remoter_info.wheel = msg_remoter.wheel;
//        } else{
//            memset(&chs_remoter_info,0,sizeof(chs_remoter_info));
//        }
//
//        /*Prepare mavlink transmission while USB connected.*/
//        if (device->ux_slave_device_state == UX_DEVICE_CONFIGURED) {
//            usb_buf_now = (usb_buf_now == usb_tx_buf[0]) ? usb_tx_buf[1] : usb_tx_buf[0];
//
//            mavlink_msg_chs_odom_info_encode(CHS_SYSTEM_ID::CHS_ID_CHASSIS, CHS_SYSTEM_ID::CHS_ID_CHASSIS, &msg_tx,
//                                             odom_buf_now);
//            usb_tx_len = mavlink_msg_to_send_buffer(usb_buf_now, &msg_tx);
//            mavlink_msg_chs_imu_info_encode(CHS_SYSTEM_ID::CHS_ID_CHASSIS, CHS_SYSTEM_ID::CHS_ID_CHASSIS, &msg_tx,
//                                            &chs_imu_info);
//            usb_tx_len += mavlink_msg_to_send_buffer(usb_buf_now + usb_tx_len, &msg_tx);
//            mavlink_msg_chs_remoter_info_encode(CHS_SYSTEM_ID::CHS_ID_CHASSIS, CHS_SYSTEM_ID::CHS_ID_CHASSIS, &msg_tx,
//                                                &chs_remoter_info);
//            usb_tx_len += mavlink_msg_to_send_buffer(usb_buf_now + usb_tx_len, &msg_tx);
//            tx_semaphore_put(&MsgCDCSem);
//        }
//
//        /*Message Receive*/
//        //Update control right
//#ifdef ASSIGN_CONTROL_RIGHT
//        _control_right_chassis = ASSIGH_CHASSIS_RIGHT;
//        _control_right_servo = ASSIGH_SERVO_RIGHT;
//#else
//
//        if (_control_right_chassis_last != control_right_chassis) {
//            _control_right_chassis_last = control_right_chassis;
//            _control_right_chassis = control_right_chassis;
//            msg_wheel_ctrl.enable = false;
//        }
//        if (_control_right_servo_last != control_right_servo) {
//            _control_right_servo_last = control_right_servo;
//            _control_right_servo = control_right_servo;
//            msg_servo.enable = false;
//        }
//#endif
//        bool publish_wheel = false;
//        bool publish_servo = false;
//
//        //manage info
//        if (spi_rx_data_processed.update) {
//            msg_wheel_ctrl.enable = spi_rx_data_processed.chs_manage_info.enable_chassis;
//            msg_servo.enable = spi_rx_data_processed.chs_manage_info.enable_servos;
//            imu_rst = spi_rx_data_processed.chs_manage_info.reset_quaternion;
//        }
//        // Chassis and Servo Enable need control right
//        if (usb_rx_data_processed.update_list_in_order[3]) {
//            imu_rst = usb_rx_data_processed.chs_manage_info.reset_quaternion;
//        }
//
//
//        if (_control_right_chassis) {
//            if (usb_rx_data_processed.update_list_in_order[0]) {
//                msg_wheel_ctrl.mps[0] =
//                        usb_rx_data_processed.chs_ctrl_info.vx - usb_rx_data_processed.chs_ctrl_info.vw * CHS_A_PLUS_B
//                        - usb_rx_data_processed.chs_ctrl_info.vy;
//                msg_wheel_ctrl.mps[1] =
//                        usb_rx_data_processed.chs_ctrl_info.vx - usb_rx_data_processed.chs_ctrl_info.vw * CHS_A_PLUS_B
//                        + usb_rx_data_processed.chs_ctrl_info.vy;
//                msg_wheel_ctrl.mps[2] =
//                        -usb_rx_data_processed.chs_ctrl_info.vx - usb_rx_data_processed.chs_ctrl_info.vw * CHS_A_PLUS_B
//                        + usb_rx_data_processed.chs_ctrl_info.vy;
//                msg_wheel_ctrl.mps[3] =
//                        -usb_rx_data_processed.chs_ctrl_info.vx - usb_rx_data_processed.chs_ctrl_info.vw * CHS_A_PLUS_B
//                        - usb_rx_data_processed.chs_ctrl_info.vy;
//
//                publish_wheel = true;
//            } else if (usb_rx_data_processed.update_list_in_order[1]) {
//                msg_wheel_ctrl.mps[0] = (float) usb_rx_data_processed.chs_motor_info.motor[0] * RPM_CONST_REV;
//                msg_wheel_ctrl.mps[1] = (float) usb_rx_data_processed.chs_motor_info.motor[1] * RPM_CONST_REV;
//                msg_wheel_ctrl.mps[2] = (float) usb_rx_data_processed.chs_motor_info.motor[2] * RPM_CONST_REV;
//                msg_wheel_ctrl.mps[3] = (float) usb_rx_data_processed.chs_motor_info.motor[3] * RPM_CONST_REV;
//                msg_wheel_ctrl.enable_extern = false;
//                publish_wheel = true;
//            }
//            if (usb_rx_data_processed.update_list_in_order[3]) {
//                msg_wheel_ctrl.enable = usb_rx_data_processed.chs_manage_info.enable_chassis;
//            }
//        } else if (spi_rx_data_processed.update) {
//            msg_wheel_ctrl.mps[0] = (float) spi_rx_data_processed.chs_motor_info.motor[0] * RPM_CONST_REV;
//            msg_wheel_ctrl.mps[1] = (float) spi_rx_data_processed.chs_motor_info.motor[1] * RPM_CONST_REV;
//            msg_wheel_ctrl.mps[2] = (float) spi_rx_data_processed.chs_motor_info.motor[2] * RPM_CONST_REV;
//            msg_wheel_ctrl.mps[3] = (float) spi_rx_data_processed.chs_motor_info.motor[3] * RPM_CONST_REV;
//            msg_wheel_ctrl.extern_motor[0] = (float) spi_rx_data_processed.chs_motor_info.motor[4] * RPM_CONST_REV;
//            msg_wheel_ctrl.extern_motor[1] = (float) spi_rx_data_processed.chs_motor_info.motor[5] * RPM_CONST_REV;
//            msg_wheel_ctrl.enable_extern = true;
//
//            publish_wheel = true;
//        }
//
//        if (publish_wheel) {
//            msg_wheel_ctrl.timestamp = tx_time_get();
//            om_publish(pup_wheel, &msg_wheel_ctrl, sizeof(Msg_WheelControl_t), true, false);
//        }
//        tx_thread_sleep(1);
//
//        if (_control_right_servo) {
//            if (usb_rx_data_processed.update_list_in_order[2]) {
//                msg_servo.servo[0] = usb_rx_data_processed.chs_servos_info.servos[0];
//                msg_servo.servo[1] = usb_rx_data_processed.chs_servos_info.servos[1];
//                msg_servo.servo[2] = usb_rx_data_processed.chs_servos_info.servos[2];
//                msg_servo.servo[3] = usb_rx_data_processed.chs_servos_info.servos[3];
//                msg_servo.servo[4] = usb_rx_data_processed.chs_servos_info.servos[4];
//                msg_servo.servo[5] = usb_rx_data_processed.chs_servos_info.servos[5];
//                msg_servo.servo[6] = usb_rx_data_processed.chs_servos_info.servos[6];
//
//                publish_servo = true;
//            }
//            if (usb_rx_data_processed.update_list_in_order[3]) {
//                msg_servo.enable = usb_rx_data_processed.chs_manage_info.enable_servos;
//            }
//        } else if (spi_rx_data_processed.update) {
//            msg_servo.servo[0] = spi_rx_data_processed.chs_servos_info.servos[0];
//            msg_servo.servo[1] = spi_rx_data_processed.chs_servos_info.servos[1];
//            msg_servo.servo[2] = spi_rx_data_processed.chs_servos_info.servos[2];
//            msg_servo.servo[3] = spi_rx_data_processed.chs_servos_info.servos[3];
//            msg_servo.servo[4] = spi_rx_data_processed.chs_servos_info.servos[4];
//            msg_servo.servo[5] = spi_rx_data_processed.chs_servos_info.servos[5];
//            msg_servo.servo[6] = spi_rx_data_processed.chs_servos_info.servos[6];
//
//            publish_servo = true;
//        }
//
//        spi_rx_data_processed.update = false;
//        usb_rx_data_processed.update_list_in_order[0] = false;
//        usb_rx_data_processed.update_list_in_order[1] = false;
//        usb_rx_data_processed.update_list_in_order[2] = false;
//        usb_rx_data_processed.update_list_in_order[3] = false;
//
//
//        if (publish_servo) {
//            msg_servo.timestamp = tx_time_get();
//            om_publish(pub_servo, &msg_servo, sizeof(Msg_Servo_t), true, false);
//        }
//
//        //100Hz
//        tx_thread_sleep(9);
//    }
//}
//
//SRAM_SET_CCM TX_THREAD MsgSPIThread;
//SRAM_SET_CCM TX_SEMAPHORE MsgSPITCSem;
//SRAM_SET_CCM uint8_t MsgSPIStack[768] = {0};
//SRAM_SET_CCM static uint8_t *spi_rx_buf = nullptr;
//SRAM_SET_CCM static uint8_t *spi_tx_buf = nullptr;
//
//[[noreturn]] void MsgSPIFun(ULONG initial_input) {
//    UNUSED(initial_input);
//    bool spi_off = false;
//    uint8_t crc_val;
//    uint32_t off_cnt = 0;
//
//    if (tx_byte_allocate(&ComPool, (VOID **) &spi_rx_buf, MSG_SPI_LEN, TX_NO_WAIT)
//        || tx_byte_allocate(&ComPool, (VOID **) &spi_tx_buf, MSG_SPI_LEN, TX_NO_WAIT)) {
//        Msg_Fault();
//    }
//    tx_thread_sleep(10);
//    //Flag of SPI TX
//    for (;;) {
//        memcpy(spi_tx_buf, odom_buf_now, sizeof(mavlink_chs_odom_info_t));
//        memcpy(spi_tx_buf+sizeof(mavlink_chs_odom_info_t), &chs_motor_extern_fdb_info, sizeof(chs_motor_extern_fdb_info));
//        memcpy(spi_tx_buf+sizeof(mavlink_chs_odom_info_t)+sizeof(chs_motor_extern_fdb_info), &chs_remoter_info, sizeof(mavlink_chs_remoter_info_t));
//        spi_tx_buf[MSG_SPI_LEN - 1] = MSG_SPI_FLAG;
//        spi_tx_buf[MSG_SPI_LEN - 1] = cal_crc8_table(spi_tx_buf, MSG_SPI_LEN);
//        HAL_SPI_TransmitReceive_DMA(&hspi2, spi_tx_buf, spi_rx_buf, MSG_SPI_LEN);
//        if (spi_off) {
//            control_right_servo = true;
//            control_right_chassis = true;
//            if (tx_semaphore_get(&MsgSPITCSem, MSG_RECNT_TIM) != TX_SUCCESS) {
//                continue;
//            }
//        } else {
//            if (tx_semaphore_get(&MsgSPITCSem, MSG_DECNT_TIM) != TX_SUCCESS) {
//                spi_off = true;
//                HAL_SPI_Abort(&hspi2);
//                tx_thread_sleep(1);
//                continue;
//            }
//        }
//
//        crc_val = spi_rx_buf[MSG_SPI_LEN - 1];
//        spi_rx_buf[MSG_SPI_LEN - 1] = MSG_SPI_FLAG;
//        if (crc_val != cal_crc8_table(spi_rx_buf, MSG_SPI_LEN)) {
//            if (++off_cnt > MSG_SPI_MAX_LOSE) {
//                spi_off = true;
//                off_cnt = 0;
//            }
//            tx_thread_sleep(1);
//            continue;
//        }
//
//        off_cnt = 0;
//        spi_off = false;
//        control_right_servo = false;
//        control_right_chassis = false;
//
//        memcpy(&spi_rx_data_processed.chs_motor_info, spi_rx_buf, sizeof(Msg_MotorExtern_t));
//        memcpy(&spi_rx_data_processed.chs_servos_info, spi_rx_buf + sizeof(Msg_MotorExtern_t),
//               sizeof(mavlink_chs_servos_info_t));
//        memcpy(&spi_rx_data_processed.chs_manage_info,
//               spi_rx_buf + sizeof(Msg_MotorExtern_t) + sizeof(mavlink_chs_servos_info_t),
//               sizeof(mavlink_chs_manage_info_t));
//        spi_rx_data_processed.update = true;
//    }
//}
//
////SPI TC Callback
//void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
//    if (hspi == &hspi2) {
//        tx_semaphore_put(&MsgSPITCSem);
//    }
//}
//
///*USB CDC-ACM pointer*/
//extern UX_SLAVE_CLASS_CDC_ACM *cdc_acm;
//
//
///**
//  * @brief  Function implementing usbx_cdc_acm_thread_entry.
//  * @param arg: Not used
//  * @retval None
//  */
//[[noreturn]] VOID usbx_cdc_acm_read_thread_entry(ULONG arg) {
//    ULONG actual_length;
//    ULONG cnt;
//    UX_SLAVE_DEVICE *device;
//    int chan = MAVLINK_COMM_0;
//    mavlink_status_t r_mavlink_status;
//
//    UX_PARAMETER_NOT_USED(arg);
//    tx_thread_sleep(10);
//
//    while (true) {
//        device = &_ux_system_slave->ux_system_slave_device;
//        /* Check if device is configured */
//        if ((device->ux_slave_device_state == UX_DEVICE_CONFIGURED) && (cdc_acm != UX_NULL)) {
//
//#ifndef UX_DEVICE_CLASS_CDC_ACM_TRANSMISSION_DISABLE
//
//            /* Set transmission_status to UX_FALSE for the first time */
//            cdc_acm->ux_slave_class_cdc_acm_transmission_status = UX_FALSE;
//
//#endif /* UX_DEVICE_CLASS_CDC_ACM_TRANSMISSION_DISABLE */
//
//            /* Read the received data in blocking mode */
//            ux_device_class_cdc_acm_read(cdc_acm, (UCHAR *) usb_rx_buf, 512,
//                                         &actual_length);
//            cnt = 0;
//            /*Have read data*/
//            while (actual_length-- > 0) {
//                if (mavlink_parse_char(chan, usb_rx_buf[cnt++], &msg_rx, &r_mavlink_status)) {
//                    /*解析包成功 处理数据*/
//                    switch (msg_rx.msgid) {
//                        case MAVLINK_MSG_ID_CHS_CTRL_INFO: {
//                            mavlink_msg_chs_ctrl_info_decode(&msg_rx, &(usb_rx_data_processed.chs_ctrl_info));
//                            usb_rx_data_processed.update_list_in_order[0] = true;
//                            break;
//                        }
//                        case MAVLINK_MSG_ID_CHS_MOTOR_INFO: {
//                            mavlink_msg_chs_motor_info_decode(&msg_rx, &(usb_rx_data_processed.chs_motor_info));
//                            usb_rx_data_processed.update_list_in_order[1] = true;
//                            break;
//                        }
//                        case MAVLINK_MSG_ID_CHS_SERVOS_INFO: {
//                            mavlink_msg_chs_servos_info_decode(&msg_rx, &(usb_rx_data_processed.chs_servos_info));
//                            usb_rx_data_processed.update_list_in_order[2] = true;
//                            break;
//                        }
//                        case MAVLINK_MSG_ID_CHS_MANAGE_INFO: {
//                            mavlink_msg_chs_manage_info_decode(&msg_rx, &(usb_rx_data_processed.chs_manage_info));
//                            usb_rx_data_processed.update_list_in_order[3] = true;
//                            break;
//                        }
//                        default: {
//                            break;
//                        }
//                    }
//                }
//            }
//        } else {
//            /* Sleep thread for 10ms */
//            tx_thread_sleep(10);
//        }
//    }
//}
//
///**
//  * @brief  Function implementing usbx_cdc_acm_write_thread_entry.
//  * @param  thread_input: Not used
//  * @retval none
//  */
//[[noreturn]] VOID usbx_cdc_acm_write_thread_entry(ULONG thread_input) {
//    ULONG actual_length;
//    UX_SLAVE_DEVICE *device;
//
//    UX_PARAMETER_NOT_USED(thread_input);
//
//    /*Wait for mutex*/
//    tx_thread_sleep(10);
//    while (true) {
//        /* Check if device is configured */
//        device = &_ux_system_slave->ux_system_slave_device;
//        while ((device->ux_slave_device_state == UX_DEVICE_CONFIGURED) && (cdc_acm != UX_NULL)) {
//
//#ifndef UX_DEVICE_CLASS_CDC_ACM_TRANSMISSION_DISABLE
//            /* Set transmission_status to UX_FALSE for the first time */
//            cdc_acm->ux_slave_class_cdc_acm_transmission_status = UX_FALSE;
//#endif
//            if (tx_semaphore_get(&MsgCDCSem, 10) == TX_SUCCESS) {
//                ux_device_class_cdc_acm_write(cdc_acm, usb_buf_now, usb_tx_len, &actual_length);
//            }
//        }
//        /*Sleep 10ms*/
//        tx_thread_sleep(10);
//    }
//}
//
//[[noreturn]] static void Msg_Fault() {
//    __disable_irq();
//    while (true);
//}