#pragma once
#ifndef SERVICE_IMU_H
#define SERVICE_IMU_H
#ifdef __cplusplus
#include "tx_api.h"
#define GRAVITY_RAW 9.81f
extern "C" {
extern float GRAVITY_FIXED;
void EXTI15_10_IRQHandler();
}

#endif
#endif