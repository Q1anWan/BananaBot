///*
// * @Description: Task of wheel control
// * @Author: qianwan
// * @Date: 2023-12-25 11:44:40
// * @LastEditTime: 2023-12-25 13:15:11
// * @LastEditors: qianwan
// */
//#include "TaskWheel.h"
//#include "DL_H723.h"
//#include "fdcan.h"
//#include "tx_api.h"
//#include "FishMessage.h"
//#include "libpid-i-1.0.hpp"
//#include "om.h"
//#include "DWT.hpp"
//
//using namespace PID;
//using namespace Wheel;
//
//static bool can_filter_config();
//
//static M2006_t m2006[6];
//
//TX_THREAD WheelThread;
//uint8_t WheelThreadStack[1024] = {0};
//
///*Close-loop control wheels*/
//[[noreturn]] void WheelThreadFun(ULONG initial_input) {
//    /*Creat Wheel Topic*/
//    om_config_topic(nullptr, "CA", "WheelControl", sizeof(Msg_WheelControl_t));
//    om_topic_t *msg_fdb = om_config_topic(nullptr, "CA", "WheelFDB", sizeof(Msg_WheelFDB_t));
//    bool ctr_enable = false;
//    uint32_t los_cnt = 0;
//    Msg_WheelControl_t wheel_ctr{};
//    Msg_WheelFDB_t wheel_fdb{};
//
//    uint8_t can_data[8];
//    uint8_t can_data_extern[8];
//
//    uint32_t *can_mail_box = nullptr;
//    CAN_TxHeaderTypeDef tx_header = {.StdId=0x200, .IDE=CAN_ID_STD, .RTR=CAN_RTR_DATA, .DLC=8};
//    CAN_TxHeaderTypeDef tx_header_extern = {.StdId=0x1FF, .IDE=CAN_ID_STD, .RTR=CAN_RTR_DATA, .DLC=8};
//
//    cDWT dwt[6];
//    PID_Inc_f wheel_pid[6];
//    for (auto &i: wheel_pid) {
//        i.SetParam(
//                0.025, 0.001, 0.0, 0.0, 0.04, 10000.0, -10000.0, false, 0, false, 0
//        );
//    }
//
//    can_filter_config();
//    om_suber_t *wheel_suber = om_subscribe(om_find_topic("WheelControl", UINT32_MAX));
//
//    for (;;) {
//        if (om_suber_export(wheel_suber, &wheel_ctr, false)) {
//            if (++los_cnt == 50) {
//                ctr_enable = false;
//            }
//        } else {
//            los_cnt = 0;
//            ctr_enable = wheel_ctr.enable;
//        }
//
//        if (ctr_enable) {
//            for (int i = 0; i < 4; ++i) {
//                wheel_pid[i].SetRef(wheel_ctr.mps[i] * RPM_CONST);
//                wheel_pid[i].Calculate(m2006[i].rpm, dwt[i].dt_sec());
//                can_data[i * 2] = (uint8_t) ((int16_t) wheel_pid[i].Out() >> 8);
//                can_data[i * 2 + 1] = (uint8_t) ((int16_t) wheel_pid[i].Out() & 0xFF);
//            }
//        } else {
//            memset(can_data, 0, sizeof(can_data));
//        }
//        HAL_CAN_AddTxMessage(&hcan1, &tx_header, can_data, can_mail_box);
//
//        if (wheel_ctr.enable_extern) {
//            if (ctr_enable) {
//                for (int i = 0; i < 2; ++i) {
//                    wheel_pid[4 + i].SetRef(wheel_ctr.extern_motor[i] * RPM_CONST);
//                    wheel_pid[4 + i].Calculate(m2006[4 + i].rpm, dwt[4 + i].dt_sec());
//                    can_data_extern[i * 2] = (uint8_t) ((int16_t) wheel_pid[4 + i].Out() >> 8);
//                    can_data_extern[i * 2 + 1] = (uint8_t) ((int16_t) wheel_pid[4 + i].Out() & 0xFF);
//                }
//            } else {
//                memset(can_data_extern, 0, sizeof(can_data));
//            }
//            HAL_CAN_AddTxMessage(&hcan1, &tx_header_extern, can_data_extern, can_mail_box);
//        }
//
//
//        /*Publish feedback*/
//        for (int i = 0; i < 4; ++i) {
//            wheel_fdb.mps[i] = (float) m2006[i].rpm * RPM_CONST_REV;
//        }
//        for (int i = 0; i < 2; ++i) {
//            wheel_fdb.extern_rpm[i] = m2006[4 + i].rpm;
//        }
//
//        wheel_fdb.timestamp = tx_time_get();
//        om_publish(msg_fdb, (uint8_t *) &wheel_fdb, sizeof(Msg_WheelFDB_t), true, false);
//
//        /*250Hz close-loop*/
//        tx_thread_sleep(4);
//    }
//}
//
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
//    uint8_t rx_data[8];
//    CAN_RxHeaderTypeDef rx_header;
//    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
//    // Add code here to handle messages in FIFO0
//    // Set data
//    m2006[rx_header.StdId - 0x201].mechanical_degree = rx_data[0] << 8 | rx_data[1];
//    m2006[rx_header.StdId - 0x201].rpm = rx_data[2] << 8 | rx_data[3];
//    m2006[rx_header.StdId - 0x201].torque = rx_data[4] << 8 | rx_data[5];
//}
//
//void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
//    uint8_t rx_data[8];
//    CAN_RxHeaderTypeDef rx_header;
//    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data);
//    // Add code here to handle messages in FIFO1
//
//}
//
//
//static bool can_filter_config() {
//    bool ret = false;
//    CAN_FilterTypeDef filter;
//    filter.FilterBank = 0;
//    filter.FilterActivation = CAN_FILTER_ENABLE;
//    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
//    // Allow ID range: 0x201-0x208
//    // Saw file in ./Doc/calculate_CAN_mask.cpp
//    filter.FilterMode = CAN_FILTERMODE_IDMASK;
//    filter.FilterScale = CAN_FILTERSCALE_32BIT;
//    filter.FilterMaskIdHigh = 0x7F0 << 5;
//    filter.FilterMaskIdLow = 0x7F0 << 5;
//    filter.FilterIdHigh = 0x200 << 5;
//    filter.FilterIdHigh = 0x200 << 5;
//    ret |= HAL_CAN_ConfigFilter(&hcan1, &filter);
//
//    // Allow ID range: All
//    filter.FilterMode = CAN_FILTERMODE_IDMASK;
//    filter.FilterScale = CAN_FILTERSCALE_32BIT;
//    filter.FilterMaskIdHigh = 0;
//    filter.FilterMaskIdLow = 0;
//    filter.FilterIdHigh = 0;
//    filter.FilterIdHigh = 0;
//    filter.FilterFIFOAssignment = CAN_RX_FIFO1;
//    filter.FilterBank = 14;
//    filter.SlaveStartFilterBank = 14;
//    ret |= HAL_CAN_ConfigFilter(&hcan2, &filter);
//
//    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
//    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
//    HAL_CAN_Start(&hcan1);
//    HAL_CAN_Start(&hcan2);
//    return ret;
//}