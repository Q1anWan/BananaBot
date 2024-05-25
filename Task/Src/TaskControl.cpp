/*
 * @Description: Task of servos control
 * @Author: qianwan
 * @Date: 2023-12-25 11:44:40
 * @LastEditTime: 2023-12-25 13:15:11
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


TX_THREAD ControlThread;
uint8_t ControlThreadStack[4096] = {0};
float PID_P_T = 0.01f;
float PID_I_T = 0.0f;
//float PID_D_T = -0.015f;
float PID_D_T = 0.0f;
float PID_OUT_T = 0.0f;

float PID_P_PSI = 0.0f;
float PID_I_PSI = 0.0f;
float PID_D_PSI = 0.0f;
float PID_OUT_PSI = 0.0f;

//float LEG_LEN = 0.0f;
//float LEG_LEN_TARGET;
//float LEG_LEN_STARTUP = 0.15f;
//float LEG_LEN_MIN = 0.15f;
//float LEG_LEN_MAX = 0.35f;

float observed_x[6];
float target_x[6];
float lqr_out[2];

float X0;
float X1;
float X2;
float X3;
float X4;
float X5;

float ref_v;
float ref_yaw;
float ref_length;
bool break_enable;

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

    //workspace 0.15 to 0.30
    TASK_CONTROL::cLegUpdate leg_length_updater(0.15f, 0.1f / 500.0f);
    TASK_CONTROL::cPID_Len pid_len[2];
    pid_len[0].SetParam(850.0f, 0.0005f, 3000.0f, -3000.0f);
    pid_len[1].SetParam(850.0f, 0.0005f, 3000.0f, -3000.0f);

    cFilterBTW2_500Hz_100Hz radian[4];
    cDWT dwt_len[2];

    PID::PID_Inc_f pid_phi0; //Synchronize of legs
    cDWT dwt_phi0;
    pid_phi0.SetParam(PID_P_T, 0, PID_D_T, 0.0f, 0.0f, 5, -5, false, 0, true, 0.2);

    PID::PID_Inc_f pid_psi_dot; //Yaw inside loop
    cDWT dwt_psi_dot;
    pid_psi_dot.SetParam(PID_P_PSI, PID_I_PSI, PID_D_PSI, 0, 0, 2, -2, false, 0, false, 0);

    PID::PID_Inc_f pid_psi; //Yaw outside loop
    cDWT dwt_psi;


    PID::PID_Inc_f pid_gamma; //Roll
    cDWT dwt_gamma;

    cLinkSolver link_solver[2];

    TASK_CONTROL::cLQR lqr;
    arm_matrix_instance_f32 mat_observed = {6, 1, observed_x};
    arm_matrix_instance_f32 mat_target = {6, 1, target_x};
    lqr.InitMatX(&mat_target, &mat_observed);

    cFilterBTW2_500Hz_2Hz refv_filter;
    float x0_last = 0;
    bool link_error = false;

    for (;;) {
        om_suber_export(remoter_suber, &remoter, false);
        om_suber_export(ins_suber, &ins, false);
        om_suber_export(odometer_suber, &odometer, false);
        om_suber_export(link_suber, &link, false);

        pid_phi0.SetRef(0.0f);

        // 0.01f 0 0
        pid_phi0.SetParam(PID_P_T, 0, PID_D_T, 0.0f, 0.0f, 5, -5, false, 0, true, 0.2);

        pid_psi_dot.SetParam(PID_P_PSI, PID_I_PSI, PID_D_PSI, 0, 0, 2, -2, false, 0, false, 0);


        link_solver[0].InputLink(radian[0].Update(link.angel_left[0]), radian[1].Update(link.angel_left[1]));
        link_solver[1].InputLink(radian[2].Update(link.angel_right[0]), radian[3].Update(link.angel_right[1]));
        link_solver[0].Resolve();
        link_solver[1].Resolve();

        observed_x[0] =
                0.5f * (link_solver[0].GetPendulumRadian() + link_solver[1].GetPendulumRadian()) + ins.euler[1] -
                PI / 2.0f;
        observed_x[1] = 0.95f * observed_x[1] + 25.0f * (observed_x[0] - x0_last);
        observed_x[2] = odometer.x;
        observed_x[3] = odometer.v;
        observed_x[4] = -ins.euler[1];
        observed_x[5] = -ins.gyro[1];


        x0_last = observed_x[0];

        X0 = observed_x[0];
        X1 = observed_x[1];
        X2 = observed_x[2];
        X3 = observed_x[3];
        X4 = observed_x[4];
        X5 = observed_x[5];

        ref_v = refv_filter.Update(remoter.ch_1);
        ref_yaw = remoter.ch_0;
        ref_length = LEG_MIN_LEN + (LEG_MAX_LEN - LEG_MIN_LEN) * ((remoter.wheel + 1.0f) / 2.0f);
        if (ref_length > LEG_MAX_LEN) {
            ref_length = LEG_MAX_LEN;
        } else if (ref_length < LEG_MIN_LEN) {
            ref_length = LEG_MIN_LEN;
        }

        if ((!remoter.online) || (remoter.switch_left != 1)) {
            for (auto &i: motor.torque) {
                i = 0;
            }
            motor.enable = false;
            om_publish(motor_control, &motor, sizeof(Msg_Motor_Ctr_t), true, false);
            leg_length_updater.Set_Length(LEG_START_LEN);
            dwt_len[0].update();
            dwt_len[1].update();
            dwt_phi0.update();
            dwt_psi.update();
            dwt_gamma.update();
            dwt_psi_dot.update();

            target_x[0] = 0;
            target_x[1] = 0;
            target_x[2] = observed_x[2];
            target_x[3] = 0;
            target_x[4] = 0;
            target_x[5] = 0;

            link_error = false;

        } else if (remoter.switch_left == 1) {
            //LQR Input
            float force_torque_left[2] = {0.0f, 0.0f};
            float force_torque_right[2] = {0.0f, 0.0f};

            //Motor Input
            float motor_torque_left[2] = {0.0f, 0.0f};
            float motor_torque_right[2] = {0.0f, 0.0f};
            float wheel_torque[2] = {0.0f, 0.0f};


            float vel[4];
            vel[0] = link.vel_left[0];
            vel[1] = link.vel_left[1];
            vel[2] = link.vel_right[0];
            vel[3] = link.vel_right[1];

            //Length PID
            float leg_len_set_val = leg_length_updater.update_val(ref_length);

            float x_dot_left[2];
            link_solver[0].VMCRevCal_Radian(vel, x_dot_left);
            pid_len[0].SetRef(leg_len_set_val);
            pid_len[0].Calculate(link_solver[0].GetPendulumLen(), x_dot_left[0], dwt_len[0].dt_sec());

            float x_dot_right[2];
            link_solver[1].VMCRevCal_Radian(vel + 2, x_dot_right);
            pid_len[1].SetRef(leg_len_set_val);
            pid_len[1].Calculate(link_solver[1].GetPendulumLen(), x_dot_right[0], dwt_len[1].dt_sec());


            //PHI0 PID
            PID_OUT_T = pid_phi0.Calculate(link_solver[0].GetPendulumRadian() - link_solver[1].GetPendulumRadian(),
                                           dwt_phi0.dt_sec());

            force_torque_left[0] -= pid_len[0].Out();
            force_torque_right[0] -= pid_len[1].Out();

            force_torque_left[1] += pid_phi0.Out();
            force_torque_right[1] -= pid_phi0.Out();


            //PSI PID
            pid_psi_dot.SetRef(ref_yaw);
            PID_OUT_PSI = pid_psi_dot.Calculate(ins.gyro[2], dwt_psi_dot.dt_sec());

            wheel_torque[0] -= pid_psi_dot.Out();
            wheel_torque[1] += pid_psi_dot.Out();


            //Balance LQR
            target_x[0] = 0.0f;
            target_x[1] = 0.0f;
            if (fabs(ref_v) > 0.1f) {
                target_x[2] = observed_x[2] + ref_v / 500.0f;
                break_enable = true;
            } else if (break_enable) {
                if (fabs(odometer.v) < 0.01f) {
                    break_enable = false;
                    target_x[2] = observed_x[2];
                }
            }
            target_x[3] = ref_v;

            target_x[4] = 0.0f;
            target_x[5] = 0.0f;


            lqr.RefreshLQRK(0.5f * (link_solver[0].GetPendulumLen() + link_solver[1].GetPendulumLen()), false);
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

            if(link_error){
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
            }
            else{
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

        tx_thread_sleep(2);
    }
}
