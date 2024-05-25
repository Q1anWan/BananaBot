#include "usart.h"
#include "DL_H723.h"
#include "BananaMsgs.h"
#include "om.h"

TX_THREAD RemoterThread;
uint8_t RemoterThreadStack[2048] = {0};
TX_SEMAPHORE RemoterThreadSem;

#define RCV_BUS_SIZE    25 //SBUS
#define BUS_MIN         200
#define BUS_MAX         1800
#define BUS_DEATH_ZONE  10

#define SWITCH_DEFAULT  3
uint8_t map_value(int16_t value) {
    if (value == 200) {
        return 3;
    } else if (value == 1000) {
        return 2;
    } else if (value == 1800) {
        return 1;
    }
    return SWITCH_DEFAULT;
}

static uint8_t size_test;
SRAM_SET_RAM_D3 uint8_t data_rx[32];

[[noreturn]] void RemoterThreadFun(ULONG initial_input) {
    UNUSED(initial_input);
    /* Remoter Topic */
    om_topic_t *remoter_topic = om_config_topic(nullptr, "CA", "REMOTER", sizeof(Msg_Remoter_t));
    om_topic_t *status_topic = om_find_topic("STATUS", UINT32_MAX);
    Msg_Remoter_t msg_remoter{};
    Msg_Thread_Status_t status_msg{};

    float rmt_half_rev = 2.0f / (float) (BUS_MAX - BUS_MIN);
    int16_t rmt_mid = (BUS_MAX + BUS_MIN) / 2;
    int16_t tmp_buf[7] = {0};
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, data_rx, RCV_BUS_SIZE);
    for (;;) {
        while (tx_semaphore_get(&RemoterThreadSem, 100) != TX_SUCCESS) {
            msg_remoter.ch_0 = 0.0f;
            msg_remoter.ch_1 = 0.0f;
            msg_remoter.ch_2 = 0.0f;
            msg_remoter.ch_3 = 0.0f;
            msg_remoter.wheel = 0.0f;
            msg_remoter.online = false;
            msg_remoter.switch_left = SWITCH_DEFAULT;
            msg_remoter.switch_right = SWITCH_DEFAULT;
            msg_remoter.timestamp = tx_time_get();
            HAL_UART_Abort(&huart5);
            om_publish(remoter_topic, &msg_remoter, sizeof(msg_remoter), true, false);
            tx_thread_sleep(3);
            status_msg.thread_id = Msg_ThreadID::REMOTER;
            status_msg.status = Msg_ErrorStatus::ERROR;
            om_publish(status_topic, &status_msg, sizeof(status_msg), true, false);
            HAL_UARTEx_ReceiveToIdle_DMA(&huart5, data_rx, RCV_BUS_SIZE);
        }

        if (size_test == RCV_BUS_SIZE) {
            SCB_InvalidateDCache_by_Addr((uint32_t *) data_rx, RCV_BUS_SIZE);
            /*缓冲赋值*/

            tmp_buf[0] = (((int16_t) data_rx[1] | ((int16_t) data_rx[2] << 8)) & 0x07FF) - rmt_mid;

            tmp_buf[1] = ((((int16_t) data_rx[2] >> 3) | ((int16_t) data_rx[3] << 5)) & 0x07FF) - rmt_mid;

            tmp_buf[2] = ((((int16_t) data_rx[3] >> 6) | ((int16_t) data_rx[4] << 2) |
                           ((int16_t) data_rx[5] << 10)) & 0x07FF) - rmt_mid;

            tmp_buf[3] = ((((int16_t) data_rx[5] >> 1) | ((int16_t) data_rx[6] << 7)) & 0x07FF) - rmt_mid;

            tmp_buf[4] = ((((int16_t) data_rx[6] >> 4) | ((int16_t) data_rx[7] << 4)) & 0x07FF);

            tmp_buf[5] = ((((int16_t) data_rx[7] >> 7) | ((int16_t) data_rx[8] << 1) |
                           ((int16_t) data_rx[9] << 9)) & 0x07FF);
            tmp_buf[6] = ((((int16_t) data_rx[9] >> 2) | ((int16_t) data_rx[10] << 6)) & 0x07FF) - rmt_mid;

            if (abs(tmp_buf[0]) < BUS_DEATH_ZONE) { tmp_buf[0] = 0; }
            if (abs(tmp_buf[1]) < BUS_DEATH_ZONE) { tmp_buf[1] = 0; }
            if (abs(tmp_buf[2]) < BUS_DEATH_ZONE) { tmp_buf[2] = 0; }
            if (abs(tmp_buf[3]) < BUS_DEATH_ZONE) { tmp_buf[3] = 0; }


            msg_remoter.ch_0 = rmt_half_rev * tmp_buf[0];
            msg_remoter.ch_3 = rmt_half_rev * tmp_buf[1];
            msg_remoter.ch_1 = rmt_half_rev * tmp_buf[2];
            msg_remoter.ch_2 = rmt_half_rev * tmp_buf[3];
            msg_remoter.switch_left = map_value(tmp_buf[4]);
            msg_remoter.switch_right = map_value(tmp_buf[5]);
            msg_remoter.wheel = rmt_half_rev * (float) (tmp_buf[6]);

            if(msg_remoter.ch_1<-1.1){
                msg_remoter.online = false;
                status_msg.thread_id = Msg_ThreadID::REMOTER;
                status_msg.status = Msg_ErrorStatus::ERROR;
                om_publish(status_topic, &status_msg, sizeof(status_msg), true, false);
            }
            else{
                msg_remoter.online = true;
            }

            msg_remoter.timestamp = tx_time_get();
            om_publish(remoter_topic, &msg_remoter, sizeof(msg_remoter), true, false);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, data_rx, RCV_BUS_SIZE);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart == &huart5) {
        size_test = Size;
        tx_semaphore_put(&RemoterThreadSem);
    }
}
