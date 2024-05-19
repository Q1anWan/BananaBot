/*
 * @Description: A instantiation of IMU-BMI088
 * @Author: qianwan
 * @Date: 2023-12-22 12:00:00
 * @LastEditTime: 2023-12-23 00:28:21
 * @LastEditors: qianwan
 */

/******************************************************************************
 * @attention
 * BSD 3-Clause License
 * Copyright (c) 2023, Qianwan.Jin
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************/
/*Version 1.0*/
/*Stepper 0.0*/
#include "libbmi088-1.0.hpp"
#include <cstring>


using namespace BMI088;

uint8_t cBMI088::ReadReg(uint8_t reg, BMI088_CS cs) {
  _data_buf[0] = reg | 0x80;
  _spi[cs]->CS_Enable();
  _spi[cs]->ExchangeByte(_data_buf, _data_buf + 10, 2 + cs);
  _spi[cs]->CS_Disable();
  return _data_buf[11 + cs];
}

uint8_t cBMI088::WriteReg(uint8_t reg, uint8_t data, BMI088_CS cs) {
  _data_buf[0] = reg;
  _data_buf[1] = data;
  _spi[cs]->CS_Enable();
  _spi[cs]->ExchangeByte(_data_buf, _data_buf + 10, 2);
  _spi[cs]->CS_Disable();
  return 0;
}

uint8_t cBMI088::UpdateTem() {
  _data_buf[0] = 0x22 | 0x80;
  _spi[CS_ACCEL]->CS_Enable();
  _data_buf[0] = _spi[CS_ACCEL]->ExchangeByte(_data_buf, _data_buf + 10, 4);
  _spi[CS_ACCEL]->CS_Disable();

  /*Check spi communication*/
  if (_data_buf[0] != 0) {
    return 0x01;
  }
  uint16_t tmp = (_data_buf[12] << 3) | (_data_buf[13] >> 5);
  if (tmp > 1023) {
    tmp -= 2048;
  }
  _temperature = tmp * 0.125f + 23.0f;
  return 0;
}

uint8_t cBMI088::UpdateAccel() {
  _data_buf[0] = 0x12 | 0x80;
  _spi[CS_ACCEL]->CS_Enable();
  _data_buf[0] = _spi[CS_ACCEL]->ExchangeByte(_data_buf, _data_buf + 10, 8);
  _spi[CS_ACCEL]->CS_Disable();

  /*Check spi communication*/
  if (_data_buf[0] != 0) {
    return 0x01;
  }

  _accel[0] = (int16_t)(_data_buf[13] << 8) | _data_buf[12];
  _accel[1] = (int16_t)(_data_buf[15] << 8) | _data_buf[14];
  _accel[2] = (int16_t)(_data_buf[17] << 8) | _data_buf[16];
  return 0;
}

uint8_t cBMI088::UpdateGyro() {
  _data_buf[0] = 0x02 | 0x80;
  _spi[CS_GYRO]->CS_Enable();
  _data_buf[0] = _spi[CS_GYRO]->ExchangeByte(_data_buf, _data_buf + 10, 7);
  _spi[CS_GYRO]->CS_Disable();

  /*Check spi communication*/
  if (_data_buf[0] != 0) {
    return 0x01;
  }

  _gyro[0] = (int16_t)(_data_buf[12] << 8) | _data_buf[11];
  _gyro[1] = (int16_t)(_data_buf[14] << 8) | _data_buf[13];
  _gyro[2] = (int16_t)(_data_buf[16] << 8) | _data_buf[15];
  return 0;
}

uint8_t cBMI088::UpdateAll(void) {
  UpdateAccel();
  UpdateGyro();
  UpdateTem();
  return 0;
}

void cBMI088::GetAccel(uint8_t *pdata) { memcpy(pdata, _accel, 6); }

void cBMI088::GetGyro(uint8_t *pdata) { memcpy(pdata, _gyro, 6); }

float cBMI088::GetTem() { return _temperature; }

/*A example of configuration*/
// static uint8_t BMI088_Config(cBMI088 &bmi088) {
//     /*Begin ACC SPI Communication*/
//     bmi088.ReadReg(0x00, BMI088_CS::CS_ACCEL);
//     tx_thread_sleep(1);
//     /*Reset Sensor*/
//     bmi088.WriteReg(0x14, 0xB6, BMI088_CS::CS_GYRO);
//     bmi088.WriteReg(0x7E, 0xB6, BMI088_CS::CS_ACCEL);
//     tx_thread_sleep(100);
//     /*Begin ACC SPI Communication*/
//     bmi088.ReadReg(0x00, BMI088_CS::CS_ACCEL);
//     tx_thread_sleep(1);
//     bmi088.ReadReg(0x00, BMI088_CS::CS_ACCEL);
//     tx_thread_sleep(1);
//
//
//     /*Check Chip Connection*/
//     while (bmi088.ReadReg(0x00, BMI088_CS::CS_GYRO) != 0x0F) {
//         return 0x01;
//         //ERROR
//     }
//     /*Check Chip Connection*/
//     while (bmi088.ReadReg(0x00, BMI088_CS::CS_ACCEL) != 0x1E) {
//         return 0x02;
//         //ERROR
//     }
//     tx_thread_sleep(10);
//
//     /*Start to config IMU*/
//     /*Enable accelmemter */
//     bmi088.WriteReg(0x7D, 0x04, BMI088_CS::CS_ACCEL);
//     tx_thread_sleep(10);
//     /*Normal Mode*/
//     bmi088.WriteReg(0x7C, 0x00, BMI088_CS::CS_ACCEL);
//     tx_thread_sleep(10);
//     /*Gyro range 1000dps*/
//     bmi088.WriteReg(0x0F, 0x01, BMI088_CS::CS_GYRO);
//     /*Accel range +-12G */
//     bmi088.WriteReg(0x41, 0x02, BMI088_CS::CS_ACCEL);
//     tx_thread_sleep(1);
//     /*Gyro ODR 1KHz BW 116Hz*/
//     bmi088.WriteReg(0x10, 0x02, BMI088_CS::CS_GYRO);
//     /*Accel ODR 1.6KHz BW 280Hz*/
//     bmi088.WriteReg(0x40, 0xAC, BMI088_CS::CS_ACCEL);
//     tx_thread_sleep(1);
//     /*Gyro INT DRY*/
//     bmi088.WriteReg(0x15, 0x80, BMI088_CS::CS_GYRO);
//     tx_thread_sleep(1);
//     /*Gyro INT3 PP AL*/
//     bmi088.WriteReg(0x16, 0x0C, BMI088_CS::CS_GYRO);
//     /*Accel INT1 PP AL*/
//     bmi088.WriteReg(0x53, 0x08, BMI088_CS::CS_ACCEL);
//     tx_thread_sleep(1);
//     /*Gyro DRY pin to INT3*/
//     bmi088.WriteReg(0x18, 0x01, BMI088_CS::CS_GYRO);
//     /*Accel DRY pin to INT1*/
//     bmi088.WriteReg(0x58, 0x04, BMI088_CS::CS_ACCEL);
//     tx_thread_sleep(1);
//
//     /*Enable IDLE*/
//     LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_4);
//     LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_5);
//     NVIC_EnableIRQ(EXTI4_IRQn);//ACC
//     NVIC_EnableIRQ(EXTI9_5_IRQn);//GYRO
//     return 0;
// }