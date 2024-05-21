/*
* @Description: A instantiation of IMU-BMI088
* @Author: qianwan
* @Date: 2023-12-22 12:00:00
* @LastEditTime: 2023-12-22 17:39:26
* @LastEditors: qianwan
*/
/******************************************************************************
 * @attention
 * BSD 3-Clause License
 * Copyright (c) 2023, Qianwan.Jin
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************/
/*Version 1.0*/
/*Stepper 0.0*/
#pragma once
#ifndef LIB_BMI088_
#define LIB_BMI088_

#include "libspi-a-1.0.h"
#include "libimu-a-1.1.h"

namespace BMI088 {
    enum BMI088_CS {
        CS_GYRO = 0,
        CS_ACCEL = 1,
    };

    class cBMI088 : public IMUA::cIMUA {
    protected:
        /*BMI088 has two independent cs pin, so we use two spi class*/
        SPIA::cSPIA *_spi[2];               //SPI handle

        uint8_t _data_buf[20] = {0};        //Buffer used to communicate with IMU
        int16_t _accel[3];               //Raw acceleration
        int16_t _gyro[3];                //Raw angular velocity
        float _temperature = 0.0f;         //Temperature of IMU


    public:
        cBMI088(SPIA::cSPIA *spi_accel, SPIA::cSPIA *spi_gyro) {
            _spi[CS_ACCEL] = spi_accel;
            _spi[CS_GYRO] = spi_gyro;
        }

        cBMI088() {}

        uint8_t ReadReg(uint8_t reg, BMI088_CS cs);                   //Read a register from IMU
        uint8_t WriteReg(uint8_t reg, uint8_t data, BMI088_CS cs);    //Write a register from IMU

        uint8_t UpdateAccel() override; //Read raw acceleration
        uint8_t UpdateGyro() override;  //Read raw angular velocity
        uint8_t UpdateTem() override;   //Read temperature
        uint8_t UpdateAll() override;   //Read accel,gyro,tem

        void GetAccel(uint8_t *pdata) override; //Get raw data
        void GetGyro(uint8_t *pdata) override;  //Get raw data
        float GetTem() override;                //Get raw data
    };
}  // namespace BMI088

#endif