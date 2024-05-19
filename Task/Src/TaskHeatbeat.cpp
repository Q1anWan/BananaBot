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
        SCB_CleanDCache_by_Addr((uint32_t *) _frame_buff, sizeof(_frame_buff));
    }
};

extern TX_BYTE_POOL ComPool;
extern uint32_t fishPrintf(uint8_t *buf, const char *str, ...);

TX_THREAD HeartBeatThread;
uint8_t HeartBeatThreadStack[1024] = {0};

[[noreturn]] void HeartBeatThreadFun(ULONG initial_input) {
    UNUSED(initial_input);
    om_topic_t *battery_topic = om_config_topic(nullptr, "CA", "Battery", sizeof(msgBatteryVoltage_t));

    float battery_voltage;
    uint16_t buzzer_value = 0;
    msgBatteryVoltage_t msg_battery = {};

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

    battery_voltage = HAL_ADC_GetValue(&hadc1) * 0.00000457763671875f;

    Msg_INS_t ins;
    uint8_t *tx_buf;
    tx_byte_allocate(&ComPool, (void **) &tx_buf, 128, TX_NO_WAIT);
    om_suber_t *ins_suber = om_subscribe(om_find_topic("INS", UINT32_MAX));

    for (;;) {
        LED.SetAllPixel(Screen::GREEN, 0);
        LED.Refresh();
        HAL_ADC_Start(&hadc1);
        tx_thread_sleep(100);
        battery_voltage = 0.9f * battery_voltage + HAL_ADC_GetValue(&hadc1) * 0.000000457763671875f;

        LED.SetAllPixel(Screen::GREEN, 0.01f);
        LED.Refresh();
        HAL_ADC_Start(&hadc1);
        tx_thread_sleep(100);
        battery_voltage = 0.9f * battery_voltage + HAL_ADC_GetValue(&hadc1) * 0.000000457763671875f;
        //Enable Detection
        if (battery_voltage > 9.0f) {
            if (battery_voltage < 19.2) {
                LL_TIM_OC_SetCompareCH2(TIM12, buzzer_value);
                if (buzzer_value == 0) {
                    buzzer_value = 49;
                } else {
                    buzzer_value = 0;
                }
            }
            msg_battery.isBattery = true;
        }
        else{
            msg_battery.isBattery = false;
        }
        msg_battery.voltage = battery_voltage;
        msg_battery.time_stamp = tx_time_get();
        om_publish(battery_topic, &msg_battery, sizeof(msg_battery), true, false);

        if (om_suber_export(ins_suber, &ins, false)==OM_OK) {
            uint16_t len = fishPrintf(tx_buf, "R=%f,P=%f,Y=%f\n", ins.euler[0]*57.2957795130f, ins.euler[1]*57.2957795130f, ins.euler[2]*57.2957795130f);
            SCB_CleanInvalidateDCache_by_Addr((uint32_t *) tx_buf, len);
            HAL_UART_Transmit_DMA(&huart10, tx_buf, len);
        }

    }
}
