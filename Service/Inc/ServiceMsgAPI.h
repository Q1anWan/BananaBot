/*
 * @Description: Service of message
 * @Author: qianwan
 * @Date: 2023-09-29 14:19:38
 * @LastEditTime: 2023-10-06 15:55:08
 * @LastEditors: qianwan
 */
#ifndef SERVICE_MESSAGE_H
#define SERVICE_MESSAGE_H
#ifdef __cplusplus
#include "tx_api.h"

extern "C" {
[[noreturn]] void usbx_cdc_acm_read_thread_entry(ULONG arg);
[[noreturn]] void usbx_cdc_acm_write_thread_entry(ULONG arg);
}

#endif
#endif