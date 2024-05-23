#include "TaskMotor.h"

#include "DL_H723.h"
#include "DWT.hpp"
#include "Filter.hpp"

#include "fdcan.h"

#include "om.h"
#include "BananaMsgs.h"

#include "DMDriver.h"
#include "LinkSolver.h"
#include "KF_VelFusion.h"
#include "ServiceIMU.h"

#include "Filter.hpp"

void CANFilterConfig(void);

#define WHEEL_R 0.063f

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
//    float LimitOffset = 0.00296705f;
    float LimitOffset = 0.0f;

    /*校正后的弧度制, X正向为车头 ,Y正向竖直向下 ,Z箭矢向左 ,左右侧车轮都这样*/
    float Radian = 0.0f;


public:
    /*通讯丢包计数*/
    uint32_t loseCom = 0;

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

struct CAN_Rev_t {
    uint8_t data[3][8];
    uint8_t cnt;
};

CAN_Rev_t RxData1;
CAN_Rev_t RxData2;
uint8_t CAN_cnt;

TX_SEMAPHORE MotorCANRecvSem;

TX_THREAD MotorThread;
uint8_t MotorThreadStack[4096] = {0};

float VEL_DM4310;

bool MOTOR_ONLINE[6] = {0, 0, 0, 0, 0,0};


[[noreturn]] void MotorThreadFun(ULONG initial_input) {
    om_config_topic(nullptr, "CA", "MOTOR_CTR", sizeof(Msg_Motor_Ctr_t));
    om_topic_t *odometer_topic = om_config_topic(nullptr, "CA", "ODOMETER", sizeof(Msg_Odometer_t));
    om_topic_t *link_topic = om_config_topic(nullptr, "CA", "LINK", sizeof(Msg_Link_t));
    om_topic_t *status_topic = om_find_topic("STATUS", UINT32_MAX);
    Msg_Odometer_t odometer_msg = {};
    Msg_Link_t link_msg = {};
    Msg_Thread_Status_t status_msg = {};

    CANFilterConfig();
    cDM4310 MotorUnit[6];
    cLinkSolver Link[2];//0L 1R
    cVelFusionKF kf;
    cFilterBTW2_1000Hz_100Hz wheel_filter;
    tx_thread_sleep(3000);

    MotorUnit[0].SetID(&hfdcan2, 0x01);
    MotorUnit[0].SetMotorMode(0, 0);//左前
    MotorUnit[1].SetID(&hfdcan2, 0x02);
    MotorUnit[1].SetMotorMode(0, 1);//左后
    MotorUnit[2].SetID(&hfdcan1, 0x03);
    MotorUnit[2].SetMotorMode(0, 0);//左轮
    MotorUnit[3].SetID(&hfdcan2, 0x04);
    MotorUnit[3].SetMotorMode(1, 0);//右前
    MotorUnit[4].SetID(&hfdcan1, 0x05);
    MotorUnit[4].SetMotorMode(1, 1);//右后
    MotorUnit[5].SetID(&hfdcan1, 0x06);
    MotorUnit[5].SetMotorMode(1, 0);//右轮

    tx_thread_sleep(150);
//#define CAL_LEG
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
        motor.MITTransmit();
        tx_thread_sleep(2);
    }

    Msg_Motor_Ctr_t motor_ctr = {};
    Msg_INS_t ins = {};
    om_suber_t *motor_recv_suber = om_subscribe(om_find_topic("MOTOR_CTR", UINT32_MAX));
    om_suber_t *ins_suber = om_subscribe(om_find_topic("INS", UINT32_MAX));

//    Msg_DBG_t dbg = {};
//    om_topic_t *dbg_topic = om_find_topic("DBG", UINT32_MAX);

    ULONG time;
    ULONG last_topic_time = tx_time_get();
    bool last_enable = true;

    for (;;) {
        time = tx_time_get();
        bool update_flag[6] = {0, 0, 0, 0, 0,0};
        if (om_suber_export(motor_recv_suber, &motor_ctr, false) == OM_OK) {
            last_topic_time = tx_time_get();
            if (motor_ctr.enable) {
                for (uint8_t i = 0; i < 6; i++) {
                    MotorUnit[i].SetTorque(motor_ctr.torque[i]);
                }
            } else {
                for (auto &motor: MotorUnit) {
                    motor.SetTorque(0);
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

        RxData1.cnt = 0;
        RxData2.cnt = 0;
        CAN_cnt = 0;

        for (auto &motor: MotorUnit) {
            motor.MITTransmit();
        }

        if (tx_semaphore_get(&MotorCANRecvSem, 2)) {
            status_msg.thread_id = Msg_ThreadID::MOTOR;
            status_msg.status = Msg_ErrorStatus::ERROR;
            om_publish(status_topic, &status_msg, sizeof(status_msg), true, false);
        }

        uint8_t id;
        for (int i = 0; i < 3; ++i) {
            id = static_cast<uint8_t>(RxData1.data[i][0] & 0x0F) - 1;
            MotorUnit[id].MessageDecode(RxData1.data[i]);
            update_flag[id] = true;
        }
        for (int i = 0; i < 3; ++i) {
            id = static_cast<uint8_t>(RxData2.data[i][0] & 0x0F) - 1;
            MotorUnit[id].MessageDecode(RxData2.data[i]);
            update_flag[id] = true;
        }

        for (int i = 0; i < 6; ++i) {
            if (!update_flag[i]) {
                MOTOR_ONLINE[i] = true;
            }
        }


        for (auto &motor: MotorUnit) {
            motor.UpdateMotor();
        }

        //融合里程计
        float vel_temp = 0.5f * (MotorUnit[2].GetVelocity() + MotorUnit[5].GetVelocity()) * WHEEL_R;
        if (om_suber_export(ins_suber, &ins, false) == OM_OK) {
            float q_inv[4] = {ins.quaternion[0], -ins.quaternion[1], -ins.quaternion[2], -ins.quaternion[3]};
            float a_body[4] = {0, ins.accel[0], ins.accel[1], ins.accel[2]};
            float a_world[4] = {0};
            float tmp[4] = {0};
            arm_quaternion_product_f32(ins.quaternion, a_body, tmp, 1);
            arm_quaternion_product_f32(tmp, q_inv, a_world, 1);
            float a = sqrtf(a_world[1] * a_world[1] + a_world[2] * a_world[2]) *
                      arm_cos_f32(atan2f(a_world[2], a_world[1]) - ins.euler[2]);
            kf.UpdateKalman(vel_temp, a);
        }
        VEL_DM4310 = vel_temp;
        odometer_msg.v = kf.GetVhat();
        odometer_msg.x = kf.GetXhat();
        om_publish(odometer_topic, &odometer_msg, sizeof(odometer_msg), true, false);

        //电机角度回报
        link_msg.angel_left[0] = MotorUnit[0].GetRadian();
        link_msg.angel_left[1] = MotorUnit[1].GetRadian();
        link_msg.angel_right[0] = MotorUnit[3].GetRadian();
        link_msg.angel_right[1] = MotorUnit[4].GetRadian();
        link_msg.torque_left[0] = MotorUnit[0].GetToqReal();
        link_msg.torque_left[1] = MotorUnit[1].GetToqReal();
        link_msg.torque_right[0] = MotorUnit[3].GetToqReal();
        link_msg.torque_right[1] = MotorUnit[4].GetToqReal();
        link_msg.vel_left[0] = MotorUnit[0].GetVelocity();
        link_msg.vel_left[1] = MotorUnit[1].GetVelocity();
        link_msg.vel_right[0] = MotorUnit[3].GetVelocity();
        link_msg.vel_right[1] = MotorUnit[4].GetVelocity();

        om_publish(link_topic, &link_msg, sizeof(link_msg), true, false);

        uint8_t time_to_delay = tx_time_get() - time;
        if (time_to_delay < 2) {
            tx_thread_sleep(2 - time_to_delay);
        }
    }
}

/*
	在下面的两个FDCAN接收回调函数里面，选一个处理RM电机，选一个处理云台发送回来的数据
*/

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    FDCAN_RxHeaderTypeDef RxHeader;
    HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData1.data[RxData1.cnt]);
    if (++RxData1.cnt == 3) {
        RxData1.cnt = 0;
    }
    if (++CAN_cnt == 0x06) {
        tx_semaphore_put(&MotorCANRecvSem);
    }

}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs) {
    FDCAN_RxHeaderTypeDef RxHeader;
    HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO1, &RxHeader, RxData2.data[RxData2.cnt]);
    if (++RxData2.cnt == 3) {
        RxData2.cnt = 0;
    }
    if (++CAN_cnt == 0x06) {
        tx_semaphore_put(&MotorCANRecvSem);
    }
}

void CANFilterConfig(void) {
    FDCAN_FilterTypeDef Filter;
    Filter.IdType = FDCAN_STANDARD_ID;
    Filter.FilterIndex = 0;
    Filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    Filter.FilterType = FDCAN_FILTER_MASK;
    Filter.FilterID1 = 0x0000;
    Filter.FilterID2 = 0x0000;


    HAL_FDCAN_ConfigFilter(&hfdcan1, &Filter);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    Filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    HAL_FDCAN_ConfigFilter(&hfdcan2, &Filter);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);

    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_Start(&hfdcan2);
}