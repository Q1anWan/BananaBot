#include "app_threadx.h"
#include "TaskBooster.h"
extern TX_THREAD MotorThread;
extern uint8_t MotorThreadStack[4096];
extern void MotorThreadFun(ULONG initial_input);
extern TX_SEMAPHORE MotorCANRecvSem;

extern TX_THREAD ControlThread;
extern uint8_t ControlThreadStack[4096];

extern void ControlThreadFun(ULONG initial_input);


extern TX_THREAD HeartBeatThread;
extern uint8_t HeartBeatThreadStack[2048];

[[noreturn]] extern void HeartBeatThreadFun(ULONG initial_input);
void Task_Booster() {


/**********信号量***********/
	tx_semaphore_create(
		&MotorCANRecvSem,
		(CHAR*)"MotorCANRecvSem",
		0
		);


/***********互斥量************/

//	tx_mutex_create(
//		&msgtubeLock,
//		(CHAR*)"TubeMutex",
//		TX_NO_INHERIT);

/**********消息队列***********/
//	/*DBUS*/
//	tx_queue_create(
//		&RemoterRXQue,
//		(CHAR*)"REMOTERQUE",
//		4,
//		RemoterQueueStack,
//		sizeof(RemoterQueueStack));

/**********进程***********/
    tx_thread_create(
            &MotorThread,
            (CHAR *) "Motor",
            MotorThreadFun,
            0x0000,
            MotorThreadStack,
            sizeof(MotorThreadStack),
            6,
            6,
            TX_NO_TIME_SLICE,
            TX_AUTO_START);

    tx_thread_create(
            &ControlThread,
            (CHAR *) "Control",
            ControlThreadFun,
            0x0000,
            ControlThreadStack,
            sizeof(ControlThreadStack),
            7,
            7,
            TX_NO_TIME_SLICE,
            TX_AUTO_START);

    tx_thread_create(
            &HeartBeatThread,
            (CHAR *) "HeatBeat",
            HeartBeatThreadFun,
            0x0000,
            HeartBeatThreadStack,
            sizeof(HeartBeatThreadStack),
            15,
            15,
            TX_NO_TIME_SLICE,
            TX_AUTO_START);
}
