#include "app_threadx.h"
#include "TaskBooster.h"
extern TX_THREAD WheelThread;
extern uint8_t WheelThreadStack[1024];

extern void WheelThreadFun(ULONG initial_input);


extern TX_THREAD ServoThread;
extern uint8_t ServoThreadStack[256];

extern void ServoThreadFun(ULONG initial_input);


extern TX_THREAD HeartBeatThread;
extern uint8_t HeartBeatThreadStack[1024];

[[noreturn]] extern void HeartBeatThreadFun(ULONG initial_input);
void Task_Booster() {


/**********信号量***********/
//	tx_semaphore_create(
//		&MotorHS100Sem,
//		(CHAR*)"MotorHS100Sem",
//		0
//		);


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
//    tx_thread_create(
//            &WheelThread,
//            (CHAR *) "Wheel",
//            WheelThreadFun,
//            0x0000,
//            WheelThreadStack,
//            sizeof(WheelThreadStack),
//            6,
//            6,
//            TX_NO_TIME_SLICE,
//            TX_AUTO_START);
//
//    tx_thread_create(
//            &ServoThread,
//            (CHAR *) "Servo",
//            ServoThreadFun,
//            0x0000,
//            ServoThreadStack,
//            sizeof(ServoThreadStack),
//            6,
//            6,
//            TX_NO_TIME_SLICE,
//            TX_AUTO_START);

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
