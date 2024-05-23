/*
 * @Description: Task of Servos control
 * @Author: qianwan
 * @Date: 2023-12-25 11:44:40
 * @LastEditTime: 2024-05-23 17:05:08
 * @LastEditors: qianwan
 */
#pragma once
#ifndef TASK_SERVO_H
#define TASK_SERVO_H

#include "arm_math.h"

namespace TASK_CONTROL {
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

    class cLegUpdate {
    protected:
        float _length;
        float _path;
    public:
        cLegUpdate(float default_length, float path) : _length(default_length), _path(path) {}

        void Set_Length(float length) {
            _length = length;
        }

        void Set_Path(float path) {
            _path = path;
        }

        float update_val(float new_val) {
            if (fabsf(new_val - _length) < 1.5f * _path) {
                _length = new_val;
            } else {
                if (new_val < _length) {
                    _length -= _path;
                } else {
                    _length += _path;
                }
            }
            return _length;
        }
    };


#define LEGMAX 0.210f
#define LEGMIN 0.200f

#define LQRRESOLUTION 0.010f
#define LQRKNUM 4

    /*行进控制*/
    class cLQR {
    protected:
        /*LQR -K matrix*/
        /*2N means normal LQR*/
        /*2N+1 means off-ground LQR*/
        float LQRKbuf[LQRKNUM][12] =
                {

                        //-K in Arm Math Matrix Order: K00 K01 K02 K03 K04 K05 K10 K11 K12 K13 K14 K15
                        /* Normal -K	L=0.200000	R00=1.00	R11=15.00 */
                        {26.57501, 2.629931, 22.251168, 17.404777, -9.515445, -2.846324, -0.347140, -0.044249, -0.570704, -0.285943, -18.389619, -1.635714},
                        /* OffGround -K	L=0.200000	R00=1.00	R11=15.00 */
                        {0, 0, 0, 0, 0, 0, -0.347140, -0.044249, 0, 0, 0, 0},
                        /* Normal -K	L=0.210000	R00=1.00	R11=15.00 */
                        {26.76601, 2.627485, 22.247311, 17.320610, -9.534165, -2.929805, -0.335517, -0.042286, -0.580640, -0.286206, -18.388939, -1.683310},
                        /* OffGround -K	L=0.210000	R00=1.00	R11=15.00 */
                        {0, 0, 0, 0, 0, 0, -0.335517, -0.042286, 0, 0, 0, 0},
                };
        float LQROutBuf[2] = {0};
        float LQRXerrorBuf[6] = {0};

        arm_matrix_instance_f32 *LQRXRefX;
        arm_matrix_instance_f32 *LQRXObsX;


        arm_matrix_instance_f32 MatLQRNegK = {2, 6, (float *) LQRKbuf[0]};
        arm_matrix_instance_f32 MatLQRErrX = {6, 1, LQRXerrorBuf};
        arm_matrix_instance_f32 MatLQROutU = {2, 1, LQROutBuf};


    public:

        /*Calculate X. Output is u (T,Tp)`*/
        void LQRCal(float *Tout) {
            //Calculate error
            arm_mat_sub_f32(this->LQRXObsX, this->LQRXRefX, &this->MatLQRErrX);
            //Calculate output value
            arm_mat_mult_f32(&this->MatLQRNegK, &this->MatLQRErrX, &this->MatLQROutU);
            //return Value
            Tout[0] = this->LQROutBuf[0];
            Tout[1] = this->LQROutBuf[1];
        }

        /*
            SetLQR -K paramaters
            LegLenth : Really length of leg
            isFly	 : Is robot in the sky, should compare with offground detection
        */
        void RefreshLQRK(float LegLenth, uint8_t isFly) {
            LegLenth = (LegLenth < LEGMIN) ? LEGMIN : LegLenth;
            LegLenth = (LegLenth > LEGMAX) ? LEGMAX : LegLenth;
            volatile uint8_t ID = roundf((LegLenth - LEGMIN) / LQRRESOLUTION);
            this->MatLQRNegK.pData = (float *) LQRKbuf[2 * ID + isFly];
        }

        /* Set LQR Error Variate*/
        void InitMatX(arm_matrix_instance_f32 *pMatXRef, arm_matrix_instance_f32 *pMatXObs) {
            this->LQRXRefX = pMatXRef;
            this->LQRXObsX = pMatXObs;
        }
    };

}
#endif