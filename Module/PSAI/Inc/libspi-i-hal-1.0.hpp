/*
 * @Description: An instance of SPIA
 * @Author: qianwan
 * @Date: 2023-10-30 18:42:55
 * @LastEditTime: 2023-11-10 10:57:12
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
/*Stepper 0.3*/

#pragma once
#ifndef LIB_SPI_I_
#define LIB_SPI_I_

#include <cstdint>
#include "libspi-a-1.0.h"

/***********Config library***********/

//#include "stm32f1xx_hal_spi.h"
//#include "stm32f1xx_ll_gpio.h"

//#include "stm32f4xx_hal_spi.h"
//#include "stm32f4xx_ll_gpio.h"


#include "stm32h7xx_hal_spi.h"
#include "stm32h7xx_ll_gpio.h"

/***********************************/

namespace SPI {
    /**
     * @brief  Basic spi driver instance
     */
    class cSPI : public SPIA::cSPIA {
    protected:
        SPI_HandleTypeDef *_pspi;
        uint32_t _maxwaitcnt = 1000;
        GPIO_TypeDef *_gpio_port_cs = nullptr;
        uint32_t _gpio_pin_cs;
        bool _active_cs = false;
    public:
        cSPI(
                SPI_HandleTypeDef *pSPI,                                  /*SPI handel*/
                GPIO_TypeDef *GPIO_PORT_CS,
                uint32_t GPIO_PIN_CS,
                uint32_t maxBytetime
        )
                : _maxwaitcnt(maxBytetime) {
            bool check_pointer = (pSPI == nullptr);
            /*Check pointers*/
            if (check_pointer) {
                SPIA::cSPIA::_init_status = 0x00;
                return;
            }
            _pspi = pSPI;
            _gpio_port_cs = GPIO_PORT_CS;
            _gpio_pin_cs = GPIO_PIN_CS;
            if (GPIO_PORT_CS == nullptr) {
                _active_cs = false;
            } else {
                _active_cs = true;
            }

            SPIA::cSPIA::_init_status = 0x01;
        }

        inline void CS_Enable() override { if (_active_cs)LL_GPIO_ResetOutputPin(_gpio_port_cs, _gpio_pin_cs); }

        inline void CS_Disable() override { if (_active_cs)LL_GPIO_SetOutputPin(_gpio_port_cs, _gpio_pin_cs); }

        uint8_t ExchangeByte(uint8_t data) override {
            uint8_t rx_data;
            HAL_SPI_TransmitReceive(_pspi, &data, &rx_data, 1, _maxwaitcnt);
            return rx_data;
        }

        uint8_t ExchangeByte(uint8_t *data_tx, uint8_t *data_rx, uint32_t num) override {
            if (HAL_SPI_TransmitReceive(_pspi, data_tx, data_rx, num, _maxwaitcnt) != HAL_OK) {
                return 0x01;
            }
            return 0;
        }

        uint8_t WriteByte(uint8_t *data_tx, uint32_t num) override {
            if (HAL_SPI_Transmit(_pspi, data_tx, num, _maxwaitcnt) != HAL_OK) {
                return 0x01;
            }
            return 0;
        }

        uint8_t ReadByte(uint8_t *data_rx, uint32_t num) override {
            if (HAL_SPI_Receive(_pspi, data_rx, num, _maxwaitcnt) != HAL_OK) {
                return 0x01;
            }
            return 0;
        }

        uint8_t TransmitDMA(uint8_t *data_tx, uint32_t num) override {
            if (HAL_SPI_Transmit_DMA(_pspi, data_tx, num) != HAL_OK) {
                return 0x01;
            }
            return 0;
        }

        uint8_t ReceiveDMA(uint8_t *data_rx, uint32_t num) override {
            if (HAL_SPI_Receive_DMA(_pspi, data_rx, num) != HAL_OK) {
                return 0x01;
            }
            return 0;
        }

        uint8_t TransmitReceiveDMA(uint8_t *data_tx, uint8_t *data_rx, uint32_t num) override {
            if (HAL_SPI_TransmitReceive_DMA(_pspi, data_tx, data_rx, num) != HAL_OK) {
                return 0x01;
            }
            return 0;
        }
    };

}
#endif