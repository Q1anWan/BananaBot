///*
// * @Description: Task of servos control
// * @Author: qianwan
// * @Date: 2023-12-25 11:44:40
// * @LastEditTime: 2023-12-25 13:15:11
// * @LastEditors: qianwan
// */
//#include "TaskServo.h"
//#include "DL_H723.h"
//#include "can.h"
//#include "tx_api.h"
//#include "FishMessage.h"
//#include "om.h"
//
//TX_THREAD ServoThread;
//uint8_t ServoThreadStack[256] = {0};
//
///*Close-loop control wheels*/
//[[noreturn]] void ServoThreadFun(ULONG initial_input) {
//    /*Creat Wheel Topic*/
//    om_config_topic(nullptr, "CA", "Servo", sizeof(Msg_Servo_t));
//
//    Msg_Servo_t msg_servo{.enable=false, .servo={1499, 1499, 1499, 1499, 1499, 1499, 1499},};
//
//    bool enable = false;
//    uint32_t lose_cnt;
//    LL_TIM_EnableAllOutputs(TIM1);
//    LL_TIM_EnableAllOutputs(TIM8);
//    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
//    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
//    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
//    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH4);
//    LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH1);
//    LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH2);
//    LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH3);
//    LL_TIM_EnableCounter(TIM1);
//    LL_TIM_EnableCounter(TIM8);
//    om_suber_t *servo_suber = om_subscribe(om_find_topic("Servo", UINT32_MAX));
//    for (;;) {
//
//        if (om_suber_export(servo_suber, &msg_servo, false)) {
//            if (++lose_cnt == 50) {
//                enable = false;
//            }
//        } else {
//            lose_cnt = 0;
//            enable = msg_servo.enable;
//        }
//
//        if (enable) {
//            LL_TIM_OC_SetCompareCH1(TIM1, msg_servo.servo[0]);
//            LL_TIM_OC_SetCompareCH2(TIM1, msg_servo.servo[1]);
//            LL_TIM_OC_SetCompareCH3(TIM1, msg_servo.servo[2]);
//            LL_TIM_OC_SetCompareCH4(TIM1, msg_servo.servo[3]);
//
//            LL_TIM_OC_SetCompareCH1(TIM8, msg_servo.servo[4]);
//            LL_TIM_OC_SetCompareCH2(TIM8, msg_servo.servo[5]);
//            LL_TIM_OC_SetCompareCH3(TIM8, msg_servo.servo[6]);
//        } else {
//            LL_TIM_OC_SetCompareCH1(TIM1, 0);
//            LL_TIM_OC_SetCompareCH2(TIM1, 0);
//            LL_TIM_OC_SetCompareCH3(TIM1, 0);
//            LL_TIM_OC_SetCompareCH4(TIM1, 0);
//
//            LL_TIM_OC_SetCompareCH1(TIM8, 0);
//            LL_TIM_OC_SetCompareCH2(TIM8, 0);
//            LL_TIM_OC_SetCompareCH3(TIM8, 0);
//        }
//
//        /*50Hz close-loop*/
//        tx_thread_sleep(20);
//    }
//}
