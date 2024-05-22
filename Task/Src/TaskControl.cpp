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

class cPID_Len {
protected:
    float _kp;
    float _kd;
    float _dt;
    float _max;
    float _min;
    float _ref;
    float _out;
public:
    void SetParam(float kp, float kd, float max, float min) {
        _kp = kp;
        _kd = kd;
        _max = max;
        _min = min;
    }

    void SetRef(float ref) {
        _ref = ref;
    }

    float Out() {
        return _out;
    }

    float Calculate(float x, float x_dot, float dt) {
        float out_val;
        out_val = _kp * (_ref - x) + x_dot * _kd / dt;
        if (out_val > _max) {
            out_val = _max;
        } else if (out_val < _min) {
            out_val = _min;
        }
        _out = out_val;
        return out_val;
    }
};

TX_THREAD ControlThread;
uint8_t ControlThreadStack[2048] = {0};

/*Close-loop control wheels*/
[[noreturn]] void ControlThreadFun(ULONG initial_input) {
    /*Creat Subscribers*/
    om_topic_t *motor_control = om_find_topic("MOTOR_CTR", UINT32_MAX);
    om_suber_t *remoter_suber = om_subscribe(om_find_topic("REMOTER", UINT32_MAX));
    om_suber_t *odometer_suber = om_subscribe(om_find_topic("ODOMETER", UINT32_MAX));
    om_suber_t *link_suber = om_subscribe(om_find_topic("LINK", UINT32_MAX));
    om_suber_t *ins_suber = om_subscribe(om_find_topic("INS", UINT32_MAX));

    om_topic_t *dbg_topic = om_find_topic("DBG", UINT32_MAX);

    Msg_DBG_t dbg{};
    Msg_Remoter_t remoter{};
    Msg_INS_t ins{};
    Msg_Link_t link{};
    Msg_Odometer_t odometer{};
    Msg_Motor_Ctr_t motor{};

    cPID_Len pid_len[2];
    pid_len[0].SetParam(0.1f, 0.1f, 0.1f, -0.1f);

    cFilterBTW2_500Hz_100Hz vel_filter[4];
    cDWT dwt_len[2];

    PID::PID_Inc_f pid_psi; //Yaw
    cDWT dwt_psi;
    PID::PID_Inc_f pid_gamma; //Roll
    cDWT dwt_gamma;


    cLinkSolver link_solver[2];
    for (;;) {
        om_suber_export(remoter_suber, &remoter, false);
        om_suber_export(ins_suber, &ins, false);
        om_suber_export(odometer_suber, &odometer, false);
        om_suber_export(link_suber, &link, false);

//        if (!remoter.online || (remoter.switch_left == 3)) {
//            for (auto &i: motor.torque) {
//                i = 0;
//            }
//            motor.enable = false;
//            om_publish(motor_control, &motor, sizeof(Msg_Motor_Ctr_t), true, false);
//        } else {
        float force_torque_left[2] = {0.0f, 0.0f};
        float motor_torque_left[2] = {0.0f, 0.0f};
        float force_torque_right[2] = {0.0f, 0.0f};
        float motor_torque_right[2] = {0.0f, 0.0f};

        motor.enable = true;
        float vel[4];
        vel[0] = vel_filter[0].Update(link.vel_left[0]);
        vel[1] = vel_filter[1].Update(link.vel_left[1]);
        vel[2] = vel_filter[2].Update(link.vel_right[0]);
        vel[3] = vel_filter[3].Update(link.vel_right[1]);

        link_solver[0].InputLink(link.angel_left[0], link.angel_left[1]);
        link_solver[1].InputLink(link.angel_right[0], link.angel_right[1]);
        link_solver[0].Resolve();
        link_solver[1].Resolve();

        float x_dot[2];
        link_solver[0].VMCRevCal_Radian(vel, x_dot);
        pid_len[0].SetRef(0.15f);
        pid_len[0].Calculate(link_solver[0].GetPendulumLen(), x_dot[0], dwt_len[0].dt_sec());
        dbg.dbg[0] = vel[0];
        dbg.dbg[1] = vel[1];
        dbg.dbg[2] = x_dot[0];
        dbg.dbg[3] = x_dot[1];


        link_solver[1].VMCRevCal_Radian(vel + 2, x_dot);
        pid_len[1].SetRef(0.15f);
        pid_len[1].Calculate(link_solver[1].GetPendulumLen(), x_dot[0], dwt_len[1].dt_sec());
        dbg.dbg[4] = x_dot[0];
        dbg.dbg[5] = x_dot[1];

        //Length PID
        force_torque_left[0] -= pid_len[0].Out();
        force_torque_right[0] -= pid_len[1].Out();


        //VMC
        link_solver[0].VMCCal(force_torque_left, motor_torque_left);
//            motor.torque[0] = motor_torque_left[0];
//            motor.torque[1] = motor_torque_left[1];
//            dbg.dbg[0] = link.angel_left[0];
//            dbg.dbg[1] = link.angel_left[1];
//            dbg.dbg[2] = link.angel_right[0];
//            dbg.dbg[3] = link.angel_right[1];

//            dbg.dbg[0] = link_solver[0].GetPendulumLen();
//            dbg.dbg[1] = link_solver[0].GetPendulumRadian();
//            dbg.dbg[2] = link_solver[1].GetPendulumLen();
//            dbg.dbg[3] = link_solver[1].GetPendulumRadian();

        om_publish(dbg_topic, &dbg, sizeof(Msg_DBG_t), true, false);
//            om_publish(motor_control, &motor, sizeof(Msg_Motor_Ctr_t), true, false);
//        }

        tx_thread_sleep(2);
    }
}
