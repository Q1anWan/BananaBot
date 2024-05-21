/*
 * @Description: 
 * @Author: qianwan
 * @Date: 2023-12-20 16:01:58
 * @LastEditTime: 2023-12-26 01:18:34
 * @LastEditors: qianwan
 */
#include "app_threadx.h"
#include "DL_H723.h"
#include "ServiceBooster.h"
//#include "Flash.hpp"
#include "FishMessage.h"
#include "om.h"

/*Communication pool*/
TX_BYTE_POOL ComPool;
SRAM_SET_RAM_D3 UCHAR COM_PoolBuf[4096] = {0};

/*OneMessage pool*/
TX_BYTE_POOL MsgPool;
UCHAR Msg_PoolBuf[4096] = {0};

/*EKF pool*/
TX_BYTE_POOL MathPool;
UCHAR Math_PoolBuf[14336] = {0};

extern TX_THREAD IMUThread;
extern uint8_t IMUThreadStack[1024];

extern void IMUThreadFun(ULONG initial_input);


extern TX_THREAD IMUTemThread;
extern uint8_t IMUTemThreadStack[512];

extern void IMUTemThreadFun(ULONG initial_input);


extern TX_THREAD MsgSchedulerThread;
extern TX_SEMAPHORE MsgCDCSem;
extern uint8_t MsgSchedulerStack[1536];

extern void MsgSchedulerFun(ULONG initial_input);


extern TX_THREAD MsgSPIThread;
extern TX_SEMAPHORE MsgSPITCSem;
extern uint8_t MsgSPIStack[768];

extern void MsgSPIFun(ULONG initial_input);


extern TX_THREAD RemoterThread;
extern TX_SEMAPHORE RemoterThreadSem;
extern uint8_t RemoterThreadStack[1024];

extern void RemoterThreadFun(ULONG initial_input);

void Service_Booster(void) {
    /**********Memory pool in ram***********/
    /*Communication pool in ram*/
    tx_byte_pool_create(
            &ComPool,
            (CHAR *) "COM_PoolBuf",
            COM_PoolBuf,
            sizeof(COM_PoolBuf));
    /*Communication pool in ccram*/
    tx_byte_pool_create(
            &MsgPool,
            (CHAR *) "Msg_Pool",
            Msg_PoolBuf,
            sizeof(Msg_PoolBuf));
    /*Math pool in ccram*/
    tx_byte_pool_create(
            &MathPool,
            (CHAR *) "Math_Pool",
            Math_PoolBuf,
            sizeof(Math_PoolBuf));

//    /*Enable File Service*/
//    flashCore.init();
    /*Enable OneMessage Service*/
    om_init();

/**********信号量***********/
//    tx_semaphore_create(
//            &MsgCDCSem,
//            (CHAR *) "MsgCDCSem",
//            0
//    );
//
//    tx_semaphore_create(
//            &RemoterThreadSem,
//            (CHAR *) "RemoterThreadSem",
//            0
//    );
//
//    tx_semaphore_create(
//            &MsgSPITCSem,
//            (CHAR *) "MsgSPITCSem",
//            0
//    );


/***********互斥量************/

//	tx_mutex_create(
//		&msgtubeLock,
//		(CHAR*)"TubeMutex",
//		TX_NO_INHERIT);

/**********消息队列***********/
// /*DBUS*/
// tx_queue_create(
//         &RemoterRXQue,
//         (CHAR*)"REMOTERQUE",
//         4,
//         RemoterQueueStack,
//         sizeof(RemoterQueueStack));

/**********进程***********/

    tx_thread_create(
            &IMUThread,
            (CHAR *) "IMU",
            IMUThreadFun,
            0x0000,
            IMUThreadStack,
            sizeof(IMUThreadStack),
            3,
            3,
            TX_NO_TIME_SLICE,
            TX_AUTO_START);

    tx_thread_create(
            &IMUTemThread,
            (CHAR *) "IMU_Temp",
            IMUTemThreadFun,
            0x0000,
            IMUTemThreadStack,
            sizeof(IMUTemThreadStack),
            10,
            10,
            TX_NO_TIME_SLICE,
            TX_AUTO_START);
//
//    tx_thread_create(
//            &RemoterThread,
//            (CHAR *) "Remoter",
//            RemoterThreadFun,
//            0x0000,
//            RemoterThreadStack,
//            sizeof(RemoterThreadStack),
//            4,
//            4,
//            TX_NO_TIME_SLICE,
//            TX_AUTO_START);
//
//    tx_thread_create(
//            &MsgSchedulerThread,
//            (CHAR *) "MsgScheduler",
//            MsgSchedulerFun,
//            0x0000,
//            MsgSchedulerStack,
//            sizeof(MsgSchedulerStack),
//            5,
//            5,
//            TX_NO_TIME_SLICE,
//            TX_AUTO_START);
//
//    tx_thread_create(
//            &MsgSPIThread,
//            (CHAR *) "MsgSPI",
//            MsgSPIFun,
//            0x0000,
//            MsgSPIStack,
//            sizeof(MsgSPIStack),
//            4,
//            4,
//            TX_NO_TIME_SLICE,
//            TX_AUTO_START);
}

uint32_t fishPrintf(uint8_t *buf, const char *str, ...) {
    /*计算字符串长度,并将字符串输出到数据区*/
    va_list ap;
    va_start(ap, str);
    uint32_t len = vsnprintf((char *) buf, 512, str, ap);
    va_end(ap);
    return len;
}
