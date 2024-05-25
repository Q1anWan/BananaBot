/*
 * @Description: 
 * @Author: qianwan
 * @Date: 2023-12-17 23:50:03
 * @LastEditTime: 2024-01-18 03:16:08
 * @LastEditors: qianwan
 */
#include "main.h"
#include "DL_H723.h"
#include "spi.h"
#include "adc.h"
#include "usart.h"
#include "app_threadx.h"

#include "om.h"
#include "BananaMsgs.h"

#include "libspi-i-hal-1.0.hpp"
#include "libws2812screen-1.0.hpp"


/*TraceX utilities*/
#define TRC_BUF_SIZE (2048 * 32) /* Buffer size */
#define TRC_MAX_OBJ_COUNT (40)   /* Max number of ThreadX objects */
UCHAR TraceXBuf[TRC_BUF_SIZE];

SRAM_SET_RAM_D3 uint8_t display_buf[9 + 2];

static SPI::cSPI spi_screen(&hspi6, nullptr, 0, UINT32_MAX);

class cLED : public Screen::Display {
public:
    void AsynchBuff() override {
        SCB_CleanInvalidateDCache_by_Addr((uint32_t *) _frame_buff, sizeof(_frame_buff));
    }
};

extern TX_BYTE_POOL ComPool;

extern uint32_t fishPrintf(uint8_t *buf, const char *str, ...);

TX_THREAD HeartBeatThread;
uint8_t HeartBeatThreadStack[2048] = {0};

[[noreturn]] void HeartBeatThreadFun(ULONG initial_input) {
    UNUSED(initial_input);
    om_topic_t *battery_topic = om_config_topic(nullptr, "CA", "BATTERY", sizeof(Msg_Battery_Voltage_t));
    om_config_topic(nullptr, "CA", "STATUS", sizeof(Msg_Thread_Status_t));
    om_config_topic(nullptr, "CA", "DBG", sizeof(Msg_DBG_t));

    float battery_voltage;
    uint16_t buzzer_value = 0;

    Msg_Battery_Voltage_t msg_battery = {};

    static cLED LED;
    LED.ResetScreen(&spi_screen, display_buf, 1, 1);

    LL_TIM_EnableAllOutputs(TIM12);
    LL_TIM_CC_EnableChannel(TIM12, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_SetCompareCH2(TIM12, 0);
    LL_TIM_EnableCounter(TIM12);

    HAL_ADC_Start(&hadc1);
    tx_thread_sleep(100);
    LL_TIM_OC_SetCompareCH2(TIM12, 49);
    tx_thread_sleep(100);
    LL_TIM_OC_SetCompareCH2(TIM12, 0);

    battery_voltage = 25.2f;

    Msg_INS_t ins;
    Msg_Thread_Status_t status;
    Msg_Link_t link;
    Msg_Odometer_t odometer;
    Msg_DBG_t dbg;
    Msg_Remoter_t remoter;

    om_suber_t *ins_suber = om_subscribe(om_find_topic("INS", UINT32_MAX));
    om_suber_t *odm_suber = om_subscribe(om_find_topic("ODOMETER", UINT32_MAX));
    om_suber_t *link_suber = om_subscribe(om_find_topic("LINK", UINT32_MAX));
    om_suber_t *status_suber = om_subscribe(om_find_topic("STATUS", UINT32_MAX));
    om_suber_t *remoter_suber = om_subscribe(om_find_topic("REMOTER", UINT32_MAX));
    om_suber_t *dbg_suber = om_subscribe(om_find_topic("DBG", UINT32_MAX));

    uint8_t *tx_buf;
    tx_byte_allocate(&ComPool, (void **) &tx_buf, 512, TX_NO_WAIT);

    for (;;) {
        HAL_ADC_Start(&hadc1);
        tx_thread_sleep(10);
        battery_voltage = 0.9f * battery_voltage + HAL_ADC_GetValue(&hadc1) * 0.000055389404296875f;

        //Enable Detection
        if (battery_voltage > 9.0f) {
            if (battery_voltage < 22.2) {
                LL_TIM_OC_SetCompareCH2(TIM12, buzzer_value);
                if (buzzer_value == 0) {
                    buzzer_value = 49;
                } else {
                    buzzer_value = 0;
                }
            }
            msg_battery.isBattery = true;
        } else {
            msg_battery.isBattery = false;
        }

        msg_battery.voltage = battery_voltage;
        msg_battery.time_stamp = tx_time_get();
        om_publish(battery_topic, &msg_battery, sizeof(msg_battery), true, false);

        uint16_t len_tx = fishPrintf(tx_buf, "Voltag=%f\n", battery_voltage);
        SCB_CleanInvalidateDCache_by_Addr((uint32_t *) tx_buf, len_tx);
        HAL_UART_Transmit_DMA(&huart10, tx_buf, len_tx);
        tx_thread_sleep(2);

//        if (om_suber_export(ins_suber, &ins, false) == OM_OK) {
//            uint16_t len = fishPrintf(tx_buf, "R=%f,P=%f,Y=%f\n", ins.euler[0] * 57.2957795130f,
//                                      ins.euler[1] * 57.2957795130f, ins.euler[2] * 57.2957795130f);
//            SCB_CleanInvalidateDCache_by_Addr((uint32_t *) tx_buf, len);
//            HAL_UART_Transmit_DMA(&huart10, tx_buf, len);
//            tx_thread_sleep(2);
//
////            len = fishPrintf(tx_buf, "A0=%f,A1=%f,A2=%f,G0=%f,G1=%f,G2=%f\n", ins.accel[0], ins.accel[1], ins.accel[2],ins.gyro[0], ins.gyro[1], ins.gyro[2]);
////            SCB_CleanInvalidateDCache_by_Addr((uint32_t *) tx_buf, len);
////            HAL_UART_Transmit_DMA(&huart10, tx_buf, len);
////            tx_thread_sleep(2);
//        }

//        if (om_suber_export(link_suber, &link, false) == OM_OK) {
//            uint16_t len = fishPrintf(tx_buf, "L0=%f,L1=%f,R0=%f,R1=%f\n", link.angel_left[0], link.angel_left[1], link.angel_right[0], link.angel_right[1]);
//            SCB_CleanInvalidateDCache_by_Addr((uint32_t *) tx_buf, len);
//            HAL_UART_Transmit_DMA(&huart10, tx_buf, len);
//            tx_thread_sleep(10);
//        }
//
//        if (om_suber_export(odm_suber, &odometer, false) == OM_OK) {
//            uint16_t len = fishPrintf(tx_buf, "odm_x=%f,odm_v=%f\n", odometer.x, odometer.v);
//            SCB_CleanInvalidateDCache_by_Addr((uint32_t *) tx_buf, len);
//            HAL_UART_Transmit_DMA(&huart10, tx_buf, len);
//            tx_thread_sleep(1);
//        }
//
        if (om_suber_export(dbg_suber, &dbg, false) == OM_OK) {
            uint16_t len = fishPrintf(tx_buf, "dbg0=%f,dbg1=%f,dbg2=%f,dbg3=%f,dbg4=%f,dbg5=%f\n", dbg.dbg[0],
                                      dbg.dbg[1], dbg.dbg[2], dbg.dbg[3], dbg.dbg[4], dbg.dbg[5]);
            SCB_CleanInvalidateDCache_by_Addr((uint32_t *) tx_buf, len);
            HAL_UART_Transmit_DMA(&huart10, tx_buf, len);
            tx_thread_sleep(5);
        }

        if (om_suber_export(remoter_suber, &remoter, false) == OM_OK) {
            uint16_t len = fishPrintf(tx_buf, "CH0=%f,CH1=%f,CH2=%f,CH3=%f,SWL=%d,SWR=%d,WHEEL=%f,Timestamp=%d\n",
                                      remoter.ch_0, remoter.ch_1, remoter.ch_2, remoter.ch_3, remoter.switch_left,
                                      remoter.switch_right, remoter.wheel, remoter.timestamp);
            SCB_CleanInvalidateDCache_by_Addr((uint32_t *) tx_buf, len);
            HAL_UART_Transmit_DMA(&huart10, tx_buf, len);
            tx_thread_sleep(5);
        }

        bool is_warning = false;
        bool is_error = false;
        uint8_t id = 0;
        while (om_suber_export(status_suber, &status, false) == OM_OK) {
            if (status.status == Msg_ErrorStatus::ERROR) {
                is_error = true;
                id = static_cast<uint8_t>(status.thread_id);
            } else if (status.status == Msg_ErrorStatus::WARNING) {
                is_warning = true;
                id = static_cast<uint8_t>(status.thread_id);
            }
        }

        if (is_error) {
            for (int i = 0; i < id; i++) {
                LED.SetAllPixel(Screen::RED, 0.01f);
                LED.Refresh();
                tx_thread_sleep(50);
                LED.SetAllPixel(Screen::BLACK, 0.01f);
                LED.Refresh();
                tx_thread_sleep(150);
            }
        } else if (is_warning) {
            for (int i = 0; i < id; i++) {
                LED.SetAllPixel(Screen::YELLOW, 0.01f);
                LED.Refresh();
                tx_thread_sleep(50);
                LED.SetAllPixel(Screen::BLACK);
                LED.Refresh();
                tx_thread_sleep(150);
            }
        } else {
            LED.SetAllPixel(Screen::GREEN, 0.01f);
            LED.Refresh();
            tx_thread_sleep(500);
            LED.SetAllPixel(Screen::BLACK, 0.01f);
            LED.Refresh();
        }
        tx_thread_sleep(500);

    }
}
