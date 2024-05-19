//#include "main.h"
//#include "usart.h"
//#include "DL_H723.h"
//#include "FishMessage.h"
//#include "om.h"
//
//TX_THREAD RemoterThread;
//uint8_t RemoterThreadStack[1024] = {0};
//TX_SEMAPHORE RemoterThreadSem;
//
//uint8_t data_rx_buf[20];
//
//[[noreturn]] void RemoterThreadFun(ULONG initial_input) {
//    UNUSED(initial_input);
//    /* Remoter Topic */
//    om_topic_t *remoter_topic = om_config_topic(nullptr, "CA", "Remoter", sizeof(Msg_Remoter_t));
//    Msg_Remoter_t msg_remoter{};
//    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, data_rx_buf, 20);
//    for (;;) {
//        if (tx_semaphore_get(&RemoterThreadSem, 100) != TX_SUCCESS) {
//            memset(&msg_remoter, 0, sizeof(Msg_Remoter_t));
//            om_publish(remoter_topic, &msg_remoter, sizeof(msg_remoter), true, false);
//            tx_semaphore_get(&RemoterThreadSem, TX_WAIT_FOREVER);
//        } else {
//            msg_remoter.Online = 1;
//            msg_remoter.ch_0 = (((int16_t) data_rx_buf[0] | ((int16_t) data_rx_buf[1] << 8)) & 0x07FF) - 1024;
//            msg_remoter.ch_1 = ((((int16_t) data_rx_buf[1] >> 3) | ((int16_t) data_rx_buf[2] << 5))
//                                & 0x07FF) - 1024;
//            msg_remoter.ch_2 = ((((int16_t) data_rx_buf[2] >> 6) | ((int16_t) data_rx_buf[3] << 2) |
//                                 ((int16_t) data_rx_buf[4] << 10)) & 0x07FF) - 1024;
//            msg_remoter.ch_3 = ((((int16_t) data_rx_buf[4] >> 1) | ((int16_t) data_rx_buf[5] << 7)) &
//                                0x07FF) - 1024;
//            msg_remoter.wheel = (((int16_t) data_rx_buf[16] | ((int16_t) data_rx_buf[17] << 8)) & 0x07FF) - 1024;
//
//            uint8_t tmp = ((data_rx_buf[5] >> 4) & 0x000C) >> 2;
//            switch (tmp) {
//                case 1:
//                    msg_remoter.switch_left = 2;
//                    break;
//                case 2:
//                    msg_remoter.switch_left = 0;
//                    break;
//                case 3:
//                    msg_remoter.switch_left = 1;
//                    break;
//            }
//            tmp = ((data_rx_buf[5] >> 4) & 0x0003);
//            switch (tmp) {
//                case 1:
//                    msg_remoter.switch_right = 2;
//                    break;
//                case 2:
//                    msg_remoter.switch_right = 0;
//                    break;
//                case 3:
//                    msg_remoter.switch_right = 1;
//                    break;
//            }
//            msg_remoter.timestamp = tx_time_get();
//            om_publish(remoter_topic, &msg_remoter, sizeof(msg_remoter), true, false);
//        }
//    }
//}
//
//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
//    if (huart == &huart3) {
//        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, data_rx_buf, 20);
//        if (Size == 18) {
//            tx_semaphore_put(&RemoterThreadSem);
//        }
//    }
//}
