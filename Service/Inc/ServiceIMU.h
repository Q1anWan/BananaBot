#pragma once
#ifndef SERVICE_IMU_H
#define SERVICE_IMU_H
#ifdef __cplusplus
#include "tx_api.h"
#define GRAVITY 9.81f
extern "C" {
void EXTI15_10_IRQHandler();
}

#endif
#endif