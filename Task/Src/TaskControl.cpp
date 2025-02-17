/*
 * @Description: Task of servos control
 * @Author: qianwan
 * @Date: 2023-12-25 11:44:40
 * @LastEditTime: 2024-10-28 10:19:06
 * @LastEditors: qianwan
 */
#include "TaskControl.h"
#include "DL_H723.h"
#include "tx_api.h"
#include "BananaMsgs.h"
#include "libpid-i-2.1.hpp"
#include "LinkSolver.h"
#include "om.h"
#include "DWT.hpp"
#include "Filter.hpp"
#include "ServiceIMU.h"

TX_THREAD ControlThread;
uint8_t ControlThreadStack[4096] = {0};

float TN_L;
float TN_R;
float FN_L;
float FN_R;
float TN;
float FN;

float FL;
float FR;

float P_LEN;
float P_L;
float P_R;

float Xo0;
float Xo1;
float Xo2;
float Xo3;
float Xo4;
float Xo5;

float Xd2;
float Xd3;

float Yawd;
float Yawo;

uint8_t ID_T;

float RC0;
float RC1;
float RC2;
float RC3;

uint8_t JUMP_STAGE;
bool JUMP_FLAG;
bool SKY;

float V_Stop_Get(float len) {
    if (len < V_LIMIT_LEN) {
        return V_MAX;
    } else {
        return V_MAX * (LEG_MAX_LEN - len) * 2.0f;
    }
}

float V_Accel_Get(float len) {
    if (len < V_LIMIT_LEN) {
        return V_ACCEL_MAX;
    } else {
        return V_ACCEL_MAX * (LEG_MAX_LEN - len) / (LEG_MAX_LEN - V_LIMIT_LEN);
    }
}

float V_Limit_Get(float len) {
    if (len > V_LIMIT_LEN) {
        return V_MAX * (LEG_MAX_LEN - len) / (LEG_MAX_LEN - V_LIMIT_LEN);
    } else {
        return V_MAX;
    }
}

float V_RC_Sensitive_Get(float len) {
    if (len > V_LIMIT_LEN) {
        return (LEG_MAX_LEN - len) / (LEG_MAX_LEN - V_LIMIT_LEN);
    } else {
        return 1.0f;
    }
}

float L_Check(float len) {
    if (len > LEG_MAX_LEN) {
        return LEG_MAX_LEN;
    } else if (len < LEG_MIN_LEN) {
        return LEG_MIN_LEN;
    } else {
        return len;
    }
}

/*Close-loop control wheels*/
[[noreturn]] void ControlThreadFun(ULONG initial_input) {
    /*Creat Subscribers*/
    om_topic_t *motor_control = om_find_topic("MOTOR_CTR", UINT32_MAX);
    om_suber_t *remoter_suber = om_subscribe(om_find_topic("REMOTER", UINT32_MAX));
    om_suber_t *odometer_suber = om_subscribe(om_find_topic("ODOMETER", UINT32_MAX));
    om_suber_t *link_suber = om_subscribe(om_find_topic("LINK", UINT32_MAX));
    om_suber_t *ins_suber = om_subscribe(om_find_topic("INS", UINT32_MAX));

    om_topic_t *dbg_topic = om_find_topic("DBG", UINT32_MAX);
    om_topic_t *status_topic = om_find_topic("STATUS", UINT32_MAX);

    Msg_DBG_t dbg{};
    Msg_Remoter_t remoter{};
    Msg_INS_t ins{};
    Msg_Link_t link{};
    Msg_Odometer_t odometer{};
    Msg_Motor_Ctr_t motor{};
    Msg_Thread_Status_t status_msg = {};

    //workspace 0.15 to 0.35
    TASK_CONTROL::cValUpdate leg_length_updater(LEG_START_LEN, LEG_NORMAL_PATH);

    TASK_CONTROL::cPID_Len pid_len[2];
    pid_len[0].SetParam(850.0f, 0.06f, 3000.0f, -3000.0f);
    pid_len[1].SetParam(850.0f, 0.06f, 3000.0f, -3000.0f);
    cDWT dwt_len[2];


    PID::PID_Inc_f pid_phi0; //Synchronize of legs
    cDWT dwt_phi0;
    pid_phi0.SetParam(0.02f, 0, 0.2f, 0.0f, 0.0f, 5.0f, -5.0f, false, 0, false, 0);


    PID::PID_Inc_f pid_psi_dot; //Yaw inside loop
    cDWT dwt_psi_dot;
    pid_psi_dot.SetParam(0.0008f, 0, 0.0005f, 0, 0, 2.0f, -2.0f, false, 0, false, 0);

    PID::PID_Pst_f pid_psi; //Yaw outside loop
    cDWT dwt_psi;
    pid_psi.SetParam(0.01f, 0, 0.005f, 0, 0, 5.0f, -5.0f, 0, -0, false, 0, false, 0);

    PID::PID_Pst_f pid_gamma; //Roll
    cDWT dwt_gamma;
    pid_gamma.SetParam(0.0007f, 0, 0.0f, 0, 0, 0.2f, -0.2f, 0, -0, false, 0, false, 0);
    pid_gamma.SetRef(0.0f);

    cLinkSolver link_solver[2];

    float observed_x[6];
    float target_x[6];
    float lqr_out[2];

    TASK_CONTROL::cLQR lqr;
    arm_matrix_instance_f32 mat_observed = {6, 1, observed_x};
    arm_matrix_instance_f32 mat_target = {6, 1, target_x};
    lqr.InitMatX(&mat_target, &mat_observed);

    TASK_CONTROL::cValUpdate ref_v_updater(0, 0);
    float ref_v_slop = 0;

    bool link_error = false;

    float leg_theta;
    float leg_theta_dot;
    float leg_theta_dot_last = 0;

    float leg_len;
    float leg_len_dot;
    float leg_len_dot_last = 0;

    float x_dot_right[2];
    float x_dot_left[2];

    float ref_v;
    float ref_yaw = 0;
    float ref_length;
    bool break_enable;
    bool stop_flag;

    bool jump_mode = false;
    uint8_t jump_stage = 0;
    uint64_t jump_time;
    uint8_t swr_last = 1;

    for (;;) {
        om_suber_export(remoter_suber, &remoter, false);
        om_suber_export(ins_suber, &ins, false);
        om_suber_export(odometer_suber, &odometer, false);
        om_suber_export(link_suber, &link, false);

        pid_phi0.SetRef(0.0f);


        link_solver[0].Resolve(link.angel_left[0], link.angel_left[1]);
        link_solver[1].Resolve(link.angel_right[0], link.angel_right[1]);

        float vel[4];
        vel[0] = link.vel_left[0];
        vel[1] = link.vel_left[1];
        vel[2] = link.vel_right[0];
        vel[3] = link.vel_right[1];
        link_solver[1].VMCVelCal(vel + 2, x_dot_right);
        link_solver[0].VMCVelCal(vel, x_dot_left);


        observed_x[0] =
                0.5f * (link_solver[0].GetPendulumRadian() + link_solver[1].GetPendulumRadian()) + ins.euler[1] -
                0.5f * PI;
        observed_x[1] = 0.5f * (x_dot_right[1] + x_dot_left[1]) + ins.gyro[1];
        observed_x[2] = odometer.x;
        observed_x[3] = odometer.v;
        observed_x[4] = -ins.euler[1];
        observed_x[5] = -ins.gyro[1];

        Xo0 = observed_x[0];
        Xo1 = observed_x[1];
        Xo2 = observed_x[2];
        Xo3 = observed_x[3];
        Xo4 = observed_x[4];
        Xo5 = observed_x[5];

        leg_theta = observed_x[0];
        leg_theta_dot = observed_x[1];
        leg_len = 0.5f * (link_solver[0].GetPendulumLen() + link_solver[1].GetPendulumLen());
        leg_len_dot = 0.5f * (x_dot_right[0] + x_dot_left[0]);

        //Reverse VMC
        bool fly_flag = false;

        float rev_ft_l_t[2];
        float rev_ft_r_t[2];
        float real_torque_l[2] = {link.torque_left[0], link.torque_left[1]};
        float real_torque_r[2] = {link.torque_right[0], link.torque_right[1]};

        link_solver[0].VMCRevCal(rev_ft_l_t, real_torque_l);
        link_solver[1].VMCRevCal(rev_ft_r_t, real_torque_r);

        float F_real = rev_ft_r_t[0] + rev_ft_l_t[0];
        float T_real = rev_ft_r_t[1] + rev_ft_l_t[1];

        float cos_theta = arm_cos_f32(observed_x[0]);
        float sin_theta = arm_sin_f32(observed_x[0]);

        float P = F_real * arm_cos_f32(observed_x[0]) + T_real * arm_sin_f32(observed_x[0]) / leg_len;
        float Zw =
                (odometer.a_z - GRAVITY_FIXED)
                - (leg_len_dot - leg_len_dot_last) * cos_theta
                + 2 * (leg_len_dot) * (leg_theta_dot) * sin_theta
                + leg_len * (leg_theta_dot - leg_theta_dot_last) * sin_theta
                + leg_len * (leg_theta_dot * leg_theta_dot) * cos_theta;

        if (P + Zw > 0) {
            fly_flag = true;
            SKY = true;
        }
        SKY = false;
        TN_L = rev_ft_l_t[1];
        TN_R = rev_ft_r_t[1];
        FN_L = rev_ft_l_t[0];
        FN_R = rev_ft_r_t[0];

        TN = T_real;
        FN = P + Zw;

        P_LEN = leg_len;
        P_L = link_solver[0].GetPendulumLen();
        P_R = link_solver[1].GetPendulumLen();

        //Break when stick is free, or the direction is opposite
        //更新加速度
        ref_v_updater.SetPath(V_Accel_Get(leg_len) * 0.002f);
        stop_flag = false;
        //判定是否需要刹车或减速
        if ((fabsf(remoter.ch_3) < 0.001f) || (remoter.ch_3 * ref_v_updater.GetVal() < -0.05f)) {
            ref_v_slop = ref_v_updater.UpdateVal(0);

            if (ref_v_updater.CheckReached()) {
                ref_v_slop = 0;
                stop_flag = true;
            }
        } else {
            ref_v_slop = ref_v_updater.UpdateVal(V_RC_Sensitive_Get(leg_len) * remoter.ch_3 * V_Limit_Get(leg_len));
        }

        ref_v = ref_v_slop;
        ref_yaw -= 0.008f * remoter.ch_0;
        ref_length = L_Check(LEG_MIN_LEN + (LEG_MAX_LEN - LEG_MIN_LEN) * ((remoter.wheel + 1.0f) / 2.0f));

        JUMP_FLAG = jump_mode;
        JUMP_STAGE = jump_stage;

        //Enable Jump
        //Exit Mode

        if (jump_mode && (remoter.switch_right == 3)) {
            jump_mode = false;
            jump_stage = 0;
        } else if ((remoter.switch_left == 1) && (remoter.switch_right == 2) && (!jump_mode) && (swr_last==3)) {
            //First: recovery leg to prepare
            jump_mode = true;
            jump_stage = 1;
        } else if (jump_mode && (jump_stage == 1)) {
            //Wait for trigger
            if (leg_length_updater.CheckReached() && (leg_len < 0.15f)) {
                if (remoter.switch_right == 1) {
                    jump_stage = 2;
                }
            }
        }
        else if (jump_mode && (jump_stage == 2)) {
            //Wait until leave the ground
            if (fly_flag) {
                jump_stage = 3;
                leg_length_updater.SetPath(LEG_REC_PATH);
                jump_time = tx_time_get();
            }
        }
        else if (jump_mode && (jump_stage == 3)) {
            //Wait for touch the ground
            if (P + Zw > 20.0f) {
                jump_stage = 0;
                jump_mode = false;
                leg_length_updater.SetPath(LEG_NORMAL_PATH);
            }
        }


        if ((!remoter.online) || (remoter.switch_left != 1)) {
            for (auto &i: motor.torque) {
                i = 0;
            }
            motor.enable = false;
            om_publish(motor_control, &motor, sizeof(Msg_Motor_Ctr_t), true, false);
            leg_length_updater.SetDefault(LEG_START_LEN);
            leg_length_updater.SetPath(LEG_NORMAL_PATH);
            ref_v_updater.SetDefault(0);

            dwt_len[0].update();
            dwt_len[1].update();
            dwt_phi0.update();
            dwt_psi.update();
            dwt_gamma.update();
            dwt_psi_dot.update();

            pid_phi0.Rst();
            pid_psi.Rst();
            pid_psi_dot.Rst();
            pid_gamma.Rst();

            target_x[0] = 0;
            target_x[1] = 0;
            target_x[2] = observed_x[2];
            target_x[3] = 0;
            target_x[4] = 0;
            target_x[5] = 0;

            link_error = false;
            ref_v_slop = 0;
            break_enable = false;

            ref_yaw = ins.euler[2];

            jump_mode = false;
            jump_stage = 0;
        } else if (remoter.switch_left == 1) {
            //LQR Input
            float force_torque_left[2] = {0.0f, 0.0f};
            float force_torque_right[2] = {0.0f, 0.0f};

            //Motor Input
            float motor_torque_left[2] = {0.0f, 0.0f};
            float motor_torque_right[2] = {0.0f, 0.0f};
            float wheel_torque[2] = {0.0f, 0.0f};


            //Length PID
            if (jump_mode) {
                pid_gamma.Rst();
                if (jump_stage == 1) {
                    leg_length_updater.UpdateVal(JUMP_START_LEN);
                } else if (jump_stage == 2) {
                    fly_flag = true;
                    leg_length_updater.SetDefault(JUMP_TOP_LEN);
                } else if (jump_stage == 3) {
                    fly_flag = true;
                    leg_length_updater.UpdateVal(JUMP_REC_LEN);
                }
            } else {
                leg_length_updater.UpdateVal(ref_length);
                pid_gamma.Calculate(ins.euler[0], dwt_gamma.dt_sec());
            }


            float input_len[2] = {L_Check(leg_length_updater.GetVal() + pid_gamma.Out()),
                                  L_Check(leg_length_updater.GetVal() - pid_gamma.Out())};
            pid_len[0].SetRef(input_len[0]);
            pid_len[1].SetRef(input_len[1]);


            pid_len[0].Calculate(link_solver[0].GetPendulumLen(), x_dot_left[0], dwt_len[0].dt_sec());
            pid_len[1].Calculate(link_solver[1].GetPendulumLen(), x_dot_right[0], dwt_len[1].dt_sec());


            //PHI0 PID
            pid_phi0.Calculate(link_solver[0].GetPendulumRadian() - link_solver[1].GetPendulumRadian(),
                                             dwt_phi0.dt_sec());

            force_torque_left[0] -= pid_len[0].Out();
            force_torque_right[0] -= pid_len[1].Out();

            force_torque_left[1] += pid_phi0.Out();
            force_torque_right[1] -= pid_phi0.Out();


            Yawd = ref_yaw;
            Yawo = ins.euler[2];

            //PSI PIxD
            if (!fly_flag && (jump_stage < 2)) {
                pid_psi.SetRef(ref_yaw);
                pid_psi.Calculate(ins.euler[2], dwt_psi.dt_sec());

                pid_psi_dot.SetRef(pid_psi.Out());
                pid_psi_dot.Calculate(ins.gyro[2], dwt_psi_dot.dt_sec());
            } else {
                ref_yaw = ins.euler[2];
                pid_psi.Rst();
                pid_psi_dot.Rst();
            }

            wheel_torque[0] -= pid_psi_dot.Out();
            wheel_torque[1] += pid_psi_dot.Out();


            //Balance LQR
            target_x[0] = 0.0f;
            target_x[1] = 0.0f;
            if (!stop_flag) {
                target_x[2] = target_x[2] + ref_v * 0.002f;
                break_enable = true;
            } else if (break_enable) {
                if (fabs(odometer.v) < 0.01f) {
                    break_enable = false;
                    target_x[2] = observed_x[2];
                }
            }
//            target_x[2] = target_x[2] + ref_v * 0.002f;

//            if(target_x[2] > observed_x[2] + 1.0f){
//                target_x[2] = observed_x[2] + 1.0f;
//            }
//            else if(target_x[2] < observed_x[2] - 1.0f){
//                target_x[2] = observed_x[2] - 1.0f;
//            }

            target_x[3] = ref_v;

            target_x[4] = 0.0f;
            target_x[5] = 0.0f;


            Xd2 = target_x[2];
            Xd3 = target_x[3];


            lqr.RefreshLQRK(leg_len, fly_flag);
            lqr.LQRCal(lqr_out);

            force_torque_left[1] += 0.5f * lqr_out[1];
            force_torque_right[1] += 0.5f * lqr_out[1];

            wheel_torque[0] += 0.5f * lqr_out[0];
            wheel_torque[1] += 0.5f * lqr_out[0];

            //VMC
            link_solver[0].VMCCal(force_torque_left, motor_torque_left);
            link_solver[1].VMCCal(force_torque_right, motor_torque_right);


            if (link.angel_left[0] > PI / 2) {
                link_error = true;
            } else if (link.angel_left[1] < PI / 2) {
                link_error = true;
            } else if (link.angel_right[0] > PI / 2) {
                link_error = true;
            } else if (link.angel_right[1] < PI / 2) {
                link_error = true;
            }


            if (link_error) {
                motor.enable = false;
                motor.torque[0] = 0;
                motor.torque[1] = 0;
                motor.torque[3] = 0;
                motor.torque[4] = 0;
                motor.torque[2] = 0;
                motor.torque[5] = 0;
                status_msg.thread_id = Msg_ThreadID::CONTROL;
                status_msg.status = Msg_ErrorStatus::WARNING;
                om_publish(status_topic, &status_msg, sizeof(Msg_Thread_Status_t), true, false);
            } else {
                motor.enable = true;
                motor.torque[0] = motor_torque_left[0];
                motor.torque[1] = motor_torque_left[1];
                motor.torque[3] = motor_torque_right[0];
                motor.torque[4] = motor_torque_right[1];
                motor.torque[2] = wheel_torque[0];
                motor.torque[5] = wheel_torque[1];
            }

            om_publish(motor_control, &motor, sizeof(Msg_Motor_Ctr_t), true, false);
        }


        leg_len_dot_last = leg_len_dot;
        leg_theta_dot_last = observed_x[1];
        swr_last = remoter.switch_right;
        tx_thread_sleep(2);
    }
}
