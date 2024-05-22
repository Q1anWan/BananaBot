#include "usart.h"
#include "DL_H723.h"
#include "BananaMsgs.h"
#include "om.h"

TX_THREAD RemoterThread;
uint8_t RemoterThreadStack[1024] = {0};
TX_SEMAPHORE RemoterThreadSem;

#define RCV_BUS_SIZE    25 //SBUS
#define BUS_MIN         200
#define BUS_MAX         1800
#define BUS_DEATH_ZONE  10

uint8_t map_value(int16_t value) {
    if (value == 200) {
        return 3;
    } else if (value == 1000) {
        return 2;
    } else if (value == 1800) {
        return 1;
    }
}

static uint8_t size_test;
SRAM_SET_RAM_D3 uint8_t data_rx[RCV_BUS_SIZE];

[[noreturn]] void RemoterThreadFun(ULONG initial_input) {
    UNUSED(initial_input);
    /* Remoter Topic */
    om_topic_t *remoter_topic = om_config_topic(nullptr, "CA", "REMOTER", sizeof(Msg_Remoter_t));
    Msg_Remoter_t msg_remoter{};


    float rmt_half_rev = 2.0f / (float) (BUS_MAX - BUS_MIN);
    int16_t rmt_mid = (BUS_MAX + BUS_MIN) / 2;
    int16_t tmp_buf[7] = {0};

    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, data_rx, RCV_BUS_SIZE);
    for (;;) {
        if (tx_semaphore_get(&RemoterThreadSem, 100) != TX_SUCCESS) {
            memset(&msg_remoter, 0, sizeof(Msg_Remoter_t));
            om_publish(remoter_topic, &msg_remoter, sizeof(msg_remoter), true, false);
            tx_semaphore_get(&RemoterThreadSem, TX_WAIT_FOREVER);
        }

        if (size_test == RCV_BUS_SIZE) {
            SCB_CleanInvalidateDCache_by_Addr((uint32_t *) data_rx, RCV_BUS_SIZE);
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
