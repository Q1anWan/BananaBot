#include "ServiceIMU.h"

#include "spi.h"
#include "usart.h"

#include "DL_H723.h"
#include "DWT.hpp"

#include "libspi-i-hal-1.0.hpp"
#include "libpid-i-1.0.hpp"
#include "libbmi088-1.0.hpp"

#include "om.h"
#include "BananaMsgs.h"

#include "Flash.hpp"
#include "Filter.hpp"

#include "QuaternionEKF.h"
#include "CRC8.h"

using namespace SPI;
using namespace BMI088;
using namespace IMUA;
using namespace PID;

static uint8_t BMI088_Config(cBMI088 &bmi088);

//extern bool imu_rst;
static cIMUA *imu_handle = nullptr;

TX_THREAD IMUThread;
uint8_t IMUThreadStack[1024] = {0};

__PACKED_STRUCT imu_cal_t {
    float gyro[3];
    uint8_t crc;
};


float gyro_offset[3] = {0};

[[noreturn]] void IMUThreadFun(ULONG initial_input) {
    UNUSED(initial_input);
    /* INS Topic */
    om_topic_t *ins_topic = om_config_topic(nullptr, "CA", "INS", sizeof(Msg_INS_t));

    cDWT dwt;
    int16_t accel[3];
    int16_t gyro[3];
    float accel_f[3];
    float gyro_f[3];
//    float gyro_offset[3] = {0};
    float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    cFilterBTW2_100Hz filter[3];
    Msg_INS_t msg_ins{};
    imu_cal_t imu_cal{};

    Flash::cFlashCore::flash_memcpy((uint8_t *) &imu_cal, (uint8_t *) FLASH_STORAGE_BASE_ADDRESS, sizeof(imu_cal_t));
    if (imu_cal.crc == cal_crc8_table((uint8_t *) &imu_cal, sizeof(imu_cal_t) - 1)) {
        gyro_offset[0] = (float) imu_cal.gyro[0];
        gyro_offset[1] = (float) imu_cal.gyro[1];
        gyro_offset[2] = (float) imu_cal.gyro[2];
    } else {
        gyro_offset[0] = 0.0f;
        gyro_offset[1] = 0.0f;
        gyro_offset[2] = 0.0f;
    }

    cSPI spi_accel(&hspi2, SPI2_CS0_GPIO_Port, SPI2_CS0_Pin, UINT32_MAX);
    cSPI spi_gyro(&hspi2, SPI2_CS1_GPIO_Port, SPI2_CS1_Pin, UINT32_MAX);
    cBMI088 bmi088(&spi_accel, &spi_gyro);

    imu_handle = &bmi088;
    BMI088_Config(bmi088);

    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_10);
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_12);
    /* EXTI interrupt init*/
    NVIC_EnableIRQ(EXTI15_10_IRQn);

    tx_thread_sleep(120);
    if (!LL_GPIO_IsInputPinSet(KEY_GPIO_Port, KEY_Pin)) {
        if (!LL_TIM_IsEnabledCounter(TIM12)) {
            LL_TIM_CC_EnableChannel(TIM12, LL_TIM_CHANNEL_CH2);
            LL_TIM_EnableAllOutputs(TIM12);
            LL_TIM_EnableCounter(TIM12);
        }

        accel[0] = 19;
        accel[1] = 0;
        LL_TIM_OC_SetCompareCH2(TIM12, accel[0]);
        tx_thread_sleep(300);
        LL_TIM_OC_SetCompareCH2(TIM12, 0);

        gyro_offset[0] = 0.0f;
        gyro_offset[1] = 0.0f;
        gyro_offset[2] = 0.0f;

        while (bmi088.GetTem() < 49.0f) {
            tx_thread_sleep(100);
        }
        tx_thread_sleep(5000);
        /*Calibrate Gyro for 100 s*/
        for (uint16_t i = 0; i < 10000; i++) {
            bmi088.GetGyro((uint8_t *) gyro);
            gyro_offset[0] -= (float) gyro[0] / 10000.0f;
            gyro_offset[1] -= (float) gyro[1] / 10000.0f;
            gyro_offset[2] -= (float) gyro[2] / 10000.0f;
            if (++accel[1] == 100) {
                LL_TIM_OC_SetCompareCH2(TIM12, accel[0]);
                accel[0] = accel[0] ? 0 : 19;
                accel[1] = 0;
            }
            tx_thread_sleep(10);
        }
        LL_TIM_OC_SetCompareCH2(TIM12, 19);
        __disable_interrupts();
        FLASH_EraseInitTypeDef EraseInitStruct{.TypeErase=FLASH_TYPEERASE_SECTORS, .Banks=FLASH_BANK_1, .Sector=FLASH_STORAGE_SECTOR, .NbSectors=1, .VoltageRange=FLASH_VOLTAGE_RANGE_3};
        HAL_FLASH_Unlock();
        /*Erase Flash*/
        uint32_t error_msg;
        uint8_t write_buf[32];
        imu_cal.gyro[0] = gyro_offset[0];
        imu_cal.gyro[1] = gyro_offset[1];
        imu_cal.gyro[2] = gyro_offset[2];
        imu_cal.crc = cal_crc8_table((uint8_t *) &imu_cal, sizeof(imu_cal_t) - 1);
        memcpy(write_buf, &imu_cal, sizeof(imu_cal_t));
        if (HAL_FLASHEx_Erase(&EraseInitStruct, &error_msg) != HAL_OK) {
            HAL_FLASH_Lock();
        }
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, FLASH_STORAGE_BASE_ADDRESS, (uint32_t) write_buf) !=
            HAL_OK) {
            HAL_FLASH_Lock();
        }
        HAL_FLASH_Lock();
        NVIC_SystemReset();
    }

    IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1);
    dwt.update();
    for (;;) {

        /*Rst quaternion*/
//        if (imu_rst) {
//            imu_rst = false;
//            IMU_QuaternionEKF_Reset();
//        }

        tx_thread_sleep(1);
        bmi088.GetAccel((uint8_t *) accel);
        bmi088.GetGyro((uint8_t *) gyro);
        accel_f[0] = (float) accel[0] * LSB_ACC_16B_12G;
        accel_f[1] = (float) accel[1] * LSB_ACC_16B_12G;
        accel_f[2] = (float) accel[2] * LSB_ACC_16B_12G;

        /*100Hz LowPass BWT 2-Order*/
        /*Watch out! Orientation R-F-U*/
        accel_f[0] = filter[0].Update(accel_f[0]);
        accel_f[1] = filter[1].Update(accel_f[1]);
        accel_f[2] = filter[2].Update(accel_f[2]);

        gyro_f[0] = ((float) gyro[0] + gyro_offset[0]) * LSB_GYRO_16B_1000_R;
        gyro_f[1] = ((float) gyro[1] + gyro_offset[1]) * LSB_GYRO_16B_1000_R;
        gyro_f[2] = ((float) gyro[2] + gyro_offset[2]) * LSB_GYRO_16B_1000_R;

        IMU_QuaternionEKF_Update(quaternion, gyro_f[0], gyro_f[1], gyro_f[2], accel_f[0], accel_f[1], accel_f[2],
                                 dwt.dt_sec());

        /*Message*/
        msg_ins.timestamp = tx_time_get();
        memcpy(msg_ins.quaternion, quaternion, sizeof(quaternion));

        msg_ins.accel[0] = accel_f[0];
        msg_ins.accel[1] = accel_f[1];
        msg_ins.accel[2] = accel_f[2];
        msg_ins.gyro[0] = gyro_f[0];
        msg_ins.gyro[1] = gyro_f[1];
        msg_ins.gyro[2] = gyro_f[2];

        msg_ins.euler[0] = atan2f(2.0f * (quaternion[0] * quaternion[1] +
                                          quaternion[2] * quaternion[3]),
                                  2.0f * (quaternion[0] * quaternion[0] +
                                          quaternion[3] * quaternion[3]) - 1.0f);
        msg_ins.euler[1] =
                asinf(-2.0f * (quaternion[1] * quaternion[3] -
                               quaternion[0] * quaternion[2]));
        msg_ins.euler[2] = atan2f(2.0f * (quaternion[0] * quaternion[3] +
                                          quaternion[1] * quaternion[2]),
                                  2.0f * (quaternion[0] * quaternion[0] +
                                          quaternion[1] * quaternion[1]) - 1.0f);

        om_publish(ins_topic, &msg_ins, sizeof(msg_ins), true, false);
    }
}

TX_THREAD IMUTemThread;
uint8_t IMUTemThreadStack[512] = {0};

extern uint32_t fishPrintf(uint8_t *buf, const char *str, ...);

extern TX_BYTE_POOL ComPool;

[[noreturn]] void IMUTemThreadFun(ULONG initial_input) {
    UNUSED(initial_input);
    float tmp_last;
//    uint8_t *tx_buf;
//    tx_byte_allocate(&ComPool, (void **) &tx_buf, 128, TX_NO_WAIT);

    cDWT dwt;
    PID_Inc_f pid(8.0f, 0.5f, 3.0f, 0.0f, 0.32, 49, 0, false, 0, true, 1.5f);
    pid.SetRef(50.0f);

    LL_TIM_EnableAllOutputs(TIM3);
    LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
    LL_TIM_OC_SetCompareCH4(TIM3, 99);
    LL_TIM_EnableCounter(TIM3);
    tx_thread_sleep(2000);

    /*Assert*/
    if (imu_handle == nullptr) {
        LL_TIM_OC_SetCompareCH4(TIM3, 0);
        tx_thread_suspend(&IMUTemThread);
    }
    tmp_last = imu_handle->GetTem();
    tx_thread_sleep(1000);
    if (tmp_last == imu_handle->GetTem()) {
        //error in temp
        LL_TIM_OC_SetCompareCH4(TIM3, 0);
        tx_thread_suspend(&IMUTemThread);
    }

    while (imu_handle->GetTem() < 47.5f) {
        tx_thread_sleep(320);
    }

    dwt.update();
    for (;;) {
        tx_thread_sleep(320);
        LL_TIM_OC_SetCompareCH4(TIM3, (uint32_t) pid.Calculate(imu_handle->GetTem(), dwt.dt_sec()));
//        fishPrintf(tx_buf, "tem=%f,cmp=%f\n", imu_handle->GetTem(), pid.Out());
//        SCB_CleanInvalidateDCache_by_Addr((uint32_t *) tx_buf, strlen((char *) tx_buf));
//        HAL_UART_Transmit_DMA(&huart10, tx_buf, strlen((char *) tx_buf));
    }
}

void EXTI15_10_IRQHandler() {
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_10)) {
        static uint16_t cnt;
        if (cnt++ == 320) {
            cnt = 0;
            imu_handle->UpdateTem();
        }
        imu_handle->UpdateAccel();
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_10);
    }
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_12)) {
        imu_handle->UpdateGyro();
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_12);
    }
}

static uint8_t BMI088_Config(cBMI088 &bmi088) {
    /*Begin ACC SPI Communication*/
    bmi088.ReadReg(0x00, BMI088_CS::CS_ACCEL);
    tx_thread_sleep(1);
    /*Reset Sensor*/
    bmi088.WriteReg(0x14, 0xB6, BMI088_CS::CS_GYRO);
    tx_thread_sleep(100);
    bmi088.WriteReg(0x7E, 0xB6, BMI088_CS::CS_ACCEL);
    tx_thread_sleep(100);
    /*Begin ACC SPI Communication*/
    bmi088.ReadReg(0x00, BMI088_CS::CS_ACCEL);
    tx_thread_sleep(1);
    bmi088.ReadReg(0x00, BMI088_CS::CS_ACCEL);
    tx_thread_sleep(1);

    /*Check Chip Connection*/
    while (bmi088.ReadReg(0x00, BMI088_CS::CS_GYRO) != 0x0F) {
        return 0x01;
        //ERROR
    }
    /*Check Chip Connection*/
    while (bmi088.ReadReg(0x00, BMI088_CS::CS_ACCEL) != 0x1E) {
        return 0x02;
        //ERROR
    }
    tx_thread_sleep(10);

    /*Start to config IMU*/
    /*Enable accelerometer */
    bmi088.WriteReg(0x7D, 0x04, BMI088_CS::CS_ACCEL);
    tx_thread_sleep(10);
    /*Normal Mode*/
    bmi088.WriteReg(0x7C, 0x00, BMI088_CS::CS_ACCEL);
    tx_thread_sleep(10);
    /*Accel range +-12G */
    bmi088.WriteReg(0x41, 0x02, BMI088_CS::CS_ACCEL);
    tx_thread_sleep(5);
    /*Accel ODR 1.6KHz BW 280Hz*/
    bmi088.WriteReg(0x40, 0xAC, BMI088_CS::CS_ACCEL);
    tx_thread_sleep(5);
    /*Accel INT1 PP AL*/
    bmi088.WriteReg(0x53, 0x08, BMI088_CS::CS_ACCEL);
    tx_thread_sleep(5);
    /*Accel DRY pin to INT1*/
    bmi088.WriteReg(0x58, 0x04, BMI088_CS::CS_ACCEL);
    tx_thread_sleep(5);

    /*Gyro range 1000dps*/
    bmi088.WriteReg(0x0F, 0x01, BMI088_CS::CS_GYRO);
    tx_thread_sleep(5);
    /*Gyro ODR 1KHz BW 116Hz*/
    bmi088.WriteReg(0x10, 0x02, BMI088_CS::CS_GYRO);
    tx_thread_sleep(5);
    /*Gyro Normal Mode*/
    bmi088.WriteReg(0x11, 0x00, BMI088_CS::CS_GYRO);
    tx_thread_sleep(5);
    /*Gyro INT DRY*/
    bmi088.WriteReg(0x15, 0x80, BMI088_CS::CS_GYRO);
    tx_thread_sleep(5);
    /*Gyro INT3 PP AL*/
    bmi088.WriteReg(0x16, 0x0C, BMI088_CS::CS_GYRO);
    tx_thread_sleep(5);
    /*Gyro DRY pin to INT3*/
    bmi088.WriteReg(0x18, 0x01, BMI088_CS::CS_GYRO);
    tx_thread_sleep(5);

    /*Enable IDLE*/
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_4);
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_5);
    NVIC_EnableIRQ(EXTI4_IRQn);//ACC
    NVIC_EnableIRQ(EXTI9_5_IRQn);//GYRO
    return 0;
}