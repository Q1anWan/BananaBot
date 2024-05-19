#pragma once
#ifndef SERVICE_IMU_H
#define SERVICE_IMU_H
#ifdef __cplusplus
#include "tx_api.h"

extern "C" {
void EXTI9_5_IRQHandler();
void EXTI4_IRQHandler();
}

#endif
#endif