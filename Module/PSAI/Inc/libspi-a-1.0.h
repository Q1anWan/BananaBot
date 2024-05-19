/*
 * @Description: Abstract layer of SPI driver
 * @Author: qianwan
 * @Date: 2023-10-30 18:51:08
 * @LastEditTime: 2023-11-10 10:59:43
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
/*Stepper 0.2*/

#pragma once
#ifndef LIB_SPI_A_
#define LIB_SPI_A_
#include <cstdint>

namespace SPIA
{
    class cSPIA
    {
    protected:
        uint8_t _init_status = 0;    /*0:Not-inited 1:SPI-OK 2:SPI+DMA-OK*/

    public:
        /**
        * @brief  Check init status
        * @param  None
        * @retval State of initial
            * @arg @ref 0 Not inited
            * @arg @ref 1 SPI basic is inited
            * @arg @ref 2 SPI and DMA are inited
        */
        virtual uint8_t CheckInitializationStatus()
        {
            return _init_status;
        }

        /**
        * @brief  Enable CS# GPIO
        * @note   CS normally low is enable
        * @param  None
        * @retval None
        */
        virtual void CS_Enable() = 0;

        /**
        * @brief  Disable CS# GPIO
        * @note   CS normally high is disable
        * @param  None
        * @retval None
        */
        virtual void CS_Disable() = 0;

        /**
         * @brief  Exchange one byte with slaver
         * @param  data data want to transmit
         * @retval Date received from slaver
         */
        virtual uint8_t ExchangeByte(uint8_t data) = 0;

        /**
         * @brief  Exchange multi bytes with slaver
         * @note   Send earlier than receive, pdatatx and pdatarx can be same
         * @param  pdatatx Pointer of data buffer that want to be sent
         * @param  pdatarx Pointer of buffer that used to receive data
         * @param  num Number of bytes
         * @retval Function execution status(Successful:0 Failed:1)
         */
        virtual uint8_t ExchangeByte(uint8_t *pdatatx, uint8_t *pdatarx, uint32_t num) = 0;

        /**
         * @brief  Send multi bytes to slaver
         * @param  pdata Pointer of data buffer that want to be sent
         * @param  num Number of bytes
         * @retval Function execution status(Successful:0 Failed:1)
         */
        virtual uint8_t WriteByte(uint8_t *pdata, uint32_t num) = 0;

        /**
         * @brief  Read multi bytes from slaver
         * @note   In most cases, it's much more recommended to used ExchangeByte rather than ReadByte
         * @param  pdata Pointer of buffer that used to received data
         * @param  num Number of bytes
         * @retval Function execution status(Successful:0 Failed:1)
         */
        virtual uint8_t ReadByte(uint8_t *pdata, uint32_t num) = 0;

        /**
         * @brief  Send multi bytes to slaver through DMA
         * @note   Received data will be abandoned. When transmission finished, SPI IRQ will be called
         * @param  pdata Pointer of data buffer that want to be sent
         * @param  num Number of bytes
         * @retval Function execution status(Successful:0 Failed:1)
         */
        virtual uint8_t TransmitDMA(uint8_t *pdata, uint32_t num){return 0x01;}

        /**
         * @brief  Receive multi bytes from slaver through DMA
         * @note   These function will not start SPI transmission. Another SPI send function should be used to start transmission. When transmission finished, DMA-RX-CHN IRQ will be called.
         * @param  pdata Pointer of buffer that used to received data
         * @param  num Number of bytes
         * @retval Function execution status(Successful:0 Failed:1)
         */
        virtual uint8_t ReceiveDMA(uint8_t *pdata, uint32_t num){return 0x01;}

        /**
         * @brief  Exchange multi bytes with slaver through DMA
         * @note   Send earlier than receive, pdatatx and pdatarx can be same. When transmission finished, SPI IRQ will be called.
         * @param  pdatatx Pointer of data buffer that want to be sent
         * @param  pdatarx Pointer of buffer that used to receive data
         * @param  num Number of bytes
         * @retval Function execution status(Successful:0 Failed:1)
         */
        virtual uint8_t TransmitReceiveDMA(uint8_t *pdatatx, uint8_t *pdatarx, uint32_t num){return 0x01;}

        /**
         * @brief  DMA-RX-CHN IRQ function that need to be set in interrupt call back function
         * @note   Put this function in DMA RX Channel irq function
         * @param  None
         * @retval Function execution status(Successful:0 Failed:1)
         */
        virtual uint8_t ReceiveDMAIRQ(){return 0x01;}

        /**
         * @brief  DMA-TX-CHN IRQ function that need to be set in interrupt call back function
         * @note   Put this function in DMA TX Channel irq function
         * @param  None
         * @retval Function execution status(Successful:0 Failed:1)
         */
        virtual uint8_t TransmitDMAIRQ(){return 0x01;}

        /**
          * @brief  SPI IRQ function that need to be set in interrupt call back function
          * @note   Put this function in SPI irq function
          * @param  None
          * @retval Function execution status(Successful:0 Failed:1)
          */
        virtual uint8_t SPIIRQ(){return 0x01;}
    };
    /*Mutex should be achieved by users*/
}
#endif