#include "TaskMotor.h"

#include "DL_H723.h"
#include "DWT.hpp"

#include "fdcan.h"

#include "om.h"
#include "BananaMsgs.h"

#include "DMDriver.h"
#include "LinkSolver.h"
#include "KF_VelFusion.h"

#include "Filter.hpp"

void CANFilterConfig(void);

#define WHEEL_R 0.06f

class cDM4310 : public cDMMotor {
    /*FDCAN Driver*/
protected:
    uint16_t CANID;
    FDCAN_HandleTypeDef *hfdcan;
    FDCAN_TxHeaderTypeDef TxHeader;

    void CAN_Transmit(uint8_t *pdata) { HAL_FDCAN_AddMessageToTxFifoQ(this->hfdcan, &this->TxHeader, pdata); }

public:
    cDM4310() = default;

    cDM4310(FDCAN_HandleTypeDef *hfdcan, uint16_t ID, uint8_t Para0, uint8_t Para1) {
        this->SetID(hfdcan, ID);
        this->SetMotorMode(Para0, Para1);
    }

    void SetID(FDCAN_HandleTypeDef *hfdcan, uint16_t ID) {
        this->CANID = ID;
        this->hfdcan = hfdcan;

        this->TxHeader.Identifier = ID;
        this->TxHeader.IdType = FDCAN_STANDARD_ID;
        this->TxHeader.TxFrameType = FDCAN_DATA_FRAME;
        this->TxHeader.DataLength = FDCAN_DLC_BYTES_8;
        this->TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        this->TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
        this->TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
        this->TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        this->TxHeader.MessageMarker = 0;
    }

    /*Angel Calibration*/
protected:
    /*[0]表示左右 0左1右*/
    /*[1]表示前后 0前1后*/
    uint8_t MotorMode[2];

    /*限位导致的置零校准值  5 degree安装夹角*/
    //float	 LimitOffset = 0.08726646f;
    float LimitOffset = 0.00296705f;

    /*校正后的弧度制, X正向为车头 ,Y正向竖直向下 ,Z箭矢向左 ,左右侧车轮都这样*/
    float Radian = 0.0f;


public:
    /*通讯丢包计数*/
    uint32_t loseCom = 5;

    /*减速箱编码器计算, 按照设定坐标系进行*/
    void UpdateMotor() {
        /*先直接按电机校正后的编码器计数*/
        float tmp = this->cDMMotor::GetRadian();
        /*右侧翻转电机*/
        if (this->MotorMode[0]) { tmp = 2 * PI - tmp; }

        /*零位校准补偿*/
        if (this->MotorMode[1]) { tmp = tmp + PI - this->LimitOffset; }
        else { tmp = tmp + this->LimitOffset; }

        /* 归一化到 [0,2PI) */
        if (tmp < 0) { tmp = tmp + 2 * PI; }
        else if (tmp > 2 * PI) { tmp = tmp - 2 * PI; }
        else { tmp = tmp; }
        this->Radian = tmp;
    }

    void SetTorque(float torque) {
        this->MITUpdate(0, 0, 0, 0, MotorMode[0] ? -torque : torque);
    }

    float GetRadian() override { return this->Radian; }

    float GetVelocity() override { return this->MotorMode[0] ? -MTR.vel : MTR.vel; }
    /*[0]表示左右 0左1右*/
    /*[1]表示前后 0前1后*/
    void SetMotorMode(uint8_t Para0, uint8_t Para1) {
        this->MotorMode[0] = Para0;
        this->MotorMode[1] = Para1;
    }
};

#define t  0.002f
#define t2 0.000001f
#define t3 0.000000001f
#define t4 0.000000000001f
#define t5 0.000000000000001f

class cVelFusionKF {
protected:
    const float qq = 5.0f;//10
    const float rv = 0.1f;
    const float ra = 60.0f;//25.0f

    const float A_Init[9] = {1, t, t2 / 2, 0, 1, t, 0, 0, 1};
    const float Q_Init[9] = {t5 / 20 * qq, t4 / 8 * qq, t3 / 6 * qq, t4 / 8 * qq, t3 / 3 * qq, t2 / 2 * qq, t3 / 6 * qq,
                             t2 / 2 * qq, t * qq};
    const float H_Init[6] = {0, 1, 0, 0, 0, 1};
    const float P_Init[9] = {10, 0, 0, 0, 10, 0, 0, 0, 10};
    const float R_Init[4] = {rv, 0, 0, ra};

public:
    VelFusionKF_t KF;

    cVelFusionKF() {
        VelFusionKF_Init(&this->KF, 3, 2);//Inertia odome 3 State 2 observation
        memcpy(this->KF.P_data, P_Init, sizeof(P_Init));
        memcpy(this->KF.A_data, A_Init, sizeof(A_Init));
        memcpy(this->KF.Q_data, Q_Init, sizeof(Q_Init));
        memcpy(this->KF.H_data, H_Init, sizeof(H_Init));
        memcpy(this->KF.R_data, R_Init, sizeof(R_Init));
    }

    void ResetKF(void) { VelFusionKF_Reset(&this->KF); }

    void UpdateKalman(float Velocity, float AccelerationX) {
        this->KF.MeasuredVector[0] = Velocity;
        this->KF.MeasuredVector[1] = AccelerationX;
        VelFusionKF_Update(&this->KF);
    }

    float GetXhat() {
        return this->KF.xhat.pData[0];
    }

    float GetVhat() {
        return this->KF.xhat.pData[1];
    }

};

TX_SEMAPHORE MotorLeftSem;
TX_SEMAPHORE MotorRightSem;
TX_THREAD MotorThread;
uint8_t MotorThreadStack[2048] = {0};

void MotorThreadFun(ULONG initial_input) {
    om_config_topic(nullptr, "CA", "MOTOR_CTR", sizeof(Msg_Motor_Ctr_t));
    om_topic_t *odometer_topic = om_config_topic(nullptr, "CA", "ODOMETER", sizeof(Msg_Odometer_t));
    om_topic_t *link_topic = om_config_topic(nullptr, "CA", "LINK", sizeof(Msg_Link_t));

    Msg_Odometer_t odometer_msg = {};
    Msg_Link_t link_msg = {};

    CANFilterConfig();
    cDM4310 MotorUnit[6];
    cLinkSolver Link[2];//0L 1R
    cVelFusionKF kf;

    MotorUnit[0].SetID(&hfdcan2, 0x00);
    MotorUnit[0].SetMotorMode(0, 0);//左前
    MotorUnit[1].SetID(&hfdcan2, 0x01);
    MotorUnit[1].SetMotorMode(0, 1);//左后
    MotorUnit[2].SetID(&hfdcan2, 0x02);
    MotorUnit[2].SetMotorMode(0, 0);//左轮
    MotorUnit[3].SetID(&hfdcan2, 0x03);
    MotorUnit[3].SetMotorMode(1, 0);//右前
    MotorUnit[4].SetID(&hfdcan2, 0x04);
    MotorUnit[4].SetMotorMode(1, 1);//右后
    MotorUnit[5].SetID(&hfdcan2, 0x05);
    MotorUnit[5].SetMotorMode(1, 0);//右轮

    tx_thread_sleep(150);
#define CAL_LEG
#ifdef CAL_LEG
    /*设置零点ROM*/
    MotorUnit[0].SetZero();
    tx_thread_sleep(500);
    MotorUnit[1].SetZero();
    tx_thread_sleep(500);
    MotorUnit[3].SetZero();
    tx_thread_sleep(500);
    MotorUnit[4].SetZero();
    tx_thread_sleep(500);
#endif

    /*保护性置0*/
    for (auto &motor: MotorUnit) {
        motor.SetTorque(0);
    }

    Msg_Motor_Ctr_t motor_ctr = {};
    Msg_INS_t ins = {};
    om_suber_t *motor_recv_suber = om_subscribe(om_find_topic("MOTOR_CTR", UINT32_MAX));
    om_suber_t *ins_suber = om_subscribe(om_find_topic("INS", UINT32_MAX));

    ULONG last_topic_time = tx_time_get();
    ULONG time;
    bool last_enable;
    for (;;) {
        time = tx_time_get();
        if (om_suber_export(motor_recv_suber, &motor_ctr, false) != OM_OK) {
            last_topic_time = tx_time_get();
            if (motor_ctr.enable) {
                for (uint8_t i = 0; i < 6; i++) {
                    MotorUnit[i].SetTorque(motor_ctr.torque[i]);
                }
            }
        } else if (tx_time_get() - last_topic_time > 500) {
            for (auto &motor: MotorUnit) {
                motor.SetTorque(0);
            }
        }

        /*Motor disable or enable*/
        if (last_enable != motor_ctr.enable) {
            if (motor_ctr.enable) {
                for (auto &motor: MotorUnit) {
                    motor.EnableMotor();
                    tx_thread_sleep(1);
                }
            } else {
                for (auto &motor: MotorUnit) {
                    motor.DisableMotor();
                    tx_thread_sleep(1);
                }
            }
        }
        last_enable = motor_ctr.enable;

        MotorUnit[0].MITTransmit();
        MotorUnit[3].MITTransmit();
        tx_semaphore_get(&MotorLeftSem, 2);
        tx_semaphore_get(&MotorRightSem, 2);
        MotorUnit[1].MITTransmit();
        MotorUnit[4].MITTransmit();
        tx_semaphore_get(&MotorLeftSem, 2);
        tx_semaphore_get(&MotorRightSem, 2);
        MotorUnit[2].MITTransmit();
        MotorUnit[5].MITTransmit();
        tx_semaphore_get(&MotorLeftSem, 2);
        tx_semaphore_get(&MotorRightSem, 2);

        //融合里程计
        float vel_temp = 0.5f * (MotorUnit[0].GetVelocity() + MotorUnit[5].GetVelocity()) * WHEEL_R;
        if (om_suber_export(ins_suber, &ins, false) != OM_OK) {
            kf.UpdateKalman(vel_temp, ins.accel[0] * arm_cos_f32(ins.euler[1]));
        }
        odometer_msg.v = kf.GetVhat();
        odometer_msg.x = kf.GetXhat();
        om_publish(odometer_topic, &odometer_msg, sizeof(Link), true, false);

        //电机角度回报
        tx_thread_sleep(2 - tx_time_get() + time);
        link_msg.angel_left[0] = MotorUnit[0].GetRadian();
        link_msg.angel_left[1] = MotorUnit[1].GetRadian();
        link_msg.angel_right[0] = MotorUnit[3].GetRadian();
        link_msg.angel_right[1] = MotorUnit[4].GetRadian();
        om_publish(link_topic, &Link, sizeof(Link), true, false);

        tx_thread_sleep(2 - tx_time_get() + time);
    }
}

TX_THREAD MotorWThread;
uint8_t MotorWThreadStack[2048] = {0};


/*
	在下面的两个FDCAN接收回调函数里面，选一个处理RM电机，选一个处理云台发送回来的数据
*/


uint8_t RxData1[8];

//void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
//    FDCAN_RxHeaderTypeDef RxHeader;
//    HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData1);
//    /*Data is in RxData1*/
//    if (RxHeader.Identifier == 0x205)//Y6020
//    {
//        MotorUnit->Y6020.UpdateMotorRec(RxData1);
//        RobotControl->ChasisControl.SetErrWithGim(0.000244140625 * MotorUnit->Y6020.GetEcdCorrect() - 1.0f);
//    } else if (RxHeader.Identifier == 0x301)//SuperCapacity
//    { RobotControl->ChasisControl.SCPowerRemianInput(RxData1); }
//    else if (RxHeader.Identifier == 0x112)//Gimbal
//    {
//        memcpy(&(RobotControl->ComUP), RxData1, COMUPSIZE);
//        tx_semaphore_put(&CANUpSem);
//    }
//}
//
//uint8_t RxData2[8];
//
//void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs) {
//    FDCAN_RxHeaderTypeDef RxHeader;
//    HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO1, &RxHeader, RxData2);
//
//    /*Data is in RxData2*/
//    if (RxHeader.Identifier == 0x100) {
//        MotorUnit->LEGMotor[(uint8_t) (RxData2[0] & 0x0F) - 2].MessageDecode(RxData2);
//        MotorUnit->LEGMotor[(uint8_t) (RxData2[0] & 0x0F) - 2].UpdateMotor();
//        tx_semaphore_put(&MotorLegSem);
//    }
//}

void CANFilterConfig(void) {
    FDCAN_FilterTypeDef Filter;
    Filter.IdType = FDCAN_STANDARD_ID;
    Filter.FilterIndex = 0;
    Filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    Filter.FilterType = FDCAN_FILTER_MASK;
    Filter.FilterID1 = 0x0000;
    Filter.FilterID2 = 0x0000;


    HAL_FDCAN_ConfigFilter(&hfdcan2, &Filter);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);


    Filter.FilterType = FDCAN_FILTER_DUAL;
    Filter.FilterID1 = 0x0205;/*Y6020*/
    Filter.FilterID2 = 0x0301;/*SuperCapacity*/
    Filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &Filter);

    Filter.FilterID1 = 0x0112;/*Gimbal*/
    Filter.FilterID2 = 0x0000;/*NONE*/
    Filter.FilterIndex = 1;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &Filter);

    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);


    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_Start(&hfdcan2);
}