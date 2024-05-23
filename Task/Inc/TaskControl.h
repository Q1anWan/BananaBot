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


#define LEGMAX 0.300f
#define LEGMIN 0.200f

#define LQRRESOLUTION 0.010f
#define LQRKNUM 22

    /*行进控制*/
    class cLQR {
    protected:
        /*LQR -K matrix*/
        /*2N means normal LQR*/
        /*2N+1 means off-ground LQR*/
        float LQRKbuf[LQRKNUM][12] =
                {



                        //-K in Arm Math Matrix Order: K00 K01 K02 K03 K04 K05 K10 K11 K12 K13 K14 K15
                        /* Normal -K	L=0.200000	R00=70.00	R11=500.00 */
                        { 3.74755, 0.576842, 0.532467, 0.818044, -0.471806, -0.245303, -0.212182, -0.023877, -0.017524, -0.012318, -0.723708, -0.397553},
                        /* OffGround -K	L=0.200000	R00=70.00	R11=500.00 */
                        {0, 0, 0, 0, 0, 0, -0.212182, -0.023877, 0, 0, 0, 0},
                        /* Normal -K	L=0.210000	R00=70.00	R11=500.00 */
                        { 3.80276, 0.582304, 0.532447, 0.815314, -0.460357, -0.246651, -0.212642, -0.023499, -0.017606, -0.012169, -0.724474, -0.405413},
                        /* OffGround -K	L=0.210000	R00=70.00	R11=500.00 */
                        {0, 0, 0, 0, 0, 0, -0.212642, -0.023499, 0, 0, 0, 0},
                        /* Normal -K	L=0.220000	R00=70.00	R11=500.00 */
                        { 3.85576, 0.587875, 0.532434, 0.812907, -0.449646, -0.248149, -0.212841, -0.023159, -0.017663, -0.011991, -0.725201, -0.413503},
                        /* OffGround -K	L=0.220000	R00=70.00	R11=500.00 */
                        {0, 0, 0, 0, 0, 0, -0.212841, -0.023159, 0, 0, 0, 0},
                        /* Normal -K	L=0.230000	R00=70.00	R11=500.00 */
                        { 3.90672, 0.593544, 0.532425, 0.810785, -0.439599, -0.249776, -0.212805, -0.022854, -0.017700, -0.011791, -0.725893, -0.421810},
                        /* OffGround -K	L=0.230000	R00=70.00	R11=500.00 */
                        {0, 0, 0, 0, 0, 0, -0.212805, -0.022854, 0, 0, 0, 0},
                        /* Normal -K	L=0.240000	R00=70.00	R11=500.00 */
                        { 3.95579, 0.599304, 0.532420, 0.808911, -0.430155, -0.251518, -0.212561, -0.022578, -0.017721, -0.011575, -0.726550, -0.430322},
                        /* OffGround -K	L=0.240000	R00=70.00	R11=500.00 */
                        {0, 0, 0, 0, 0, 0, -0.212561, -0.022578, 0, 0, 0, 0},
                        /* Normal -K	L=0.250000	R00=70.00	R11=500.00 */
                        { 4.00311, 0.605144, 0.532418, 0.807256, -0.421262, -0.253362, -0.212133, -0.022328, -0.017730, -0.011349, -0.727175, -0.439028},
                        /* OffGround -K	L=0.250000	R00=70.00	R11=500.00 */
                        {0, 0, 0, 0, 0, 0, -0.212133, -0.022328, 0, 0, 0, 0},
                        /* Normal -K	L=0.260000	R00=70.00	R11=500.00 */
                        { 4.04881, 0.611058, 0.532418, 0.805796, -0.412873, -0.255295, -0.211541, -0.022099, -0.017728, -0.011116, -0.727768, -0.447917},
                        /* OffGround -K	L=0.260000	R00=70.00	R11=500.00 */
                        {0, 0, 0, 0, 0, 0, -0.211541, -0.022099, 0, 0, 0, 0},
                        /* Normal -K	L=0.270000	R00=70.00	R11=500.00 */
                        { 4.09299, 0.617039, 0.532421, 0.804508, -0.404949, -0.257310, -0.210808, -0.021890, -0.017719, -0.010880, -0.728332, -0.456978},
                        /* OffGround -K	L=0.270000	R00=70.00	R11=500.00 */
                        {0, 0, 0, 0, 0, 0, -0.210808, -0.021890, 0, 0, 0, 0},
                        /* Normal -K	L=0.280000	R00=70.00	R11=500.00 */
                        { 4.13576, 0.623079, 0.532424, 0.803375, -0.397455, -0.259399, -0.209951, -0.021697, -0.017704, -0.010645, -0.728867, -0.466201},
                        /* OffGround -K	L=0.280000	R00=70.00	R11=500.00 */
                        {0, 0, 0, 0, 0, 0, -0.209951, -0.021697, 0, 0, 0, 0},
                        /* Normal -K	L=0.290000	R00=70.00	R11=500.00 */
                        { 4.17721, 0.629172, 0.532428, 0.802379, -0.390361, -0.261555, -0.208988, -0.021519, -0.017686, -0.010412, -0.729375, -0.475578},
                        /* OffGround -K	L=0.290000	R00=70.00	R11=500.00 */
                        {0, 0, 0, 0, 0, 0, -0.208988, -0.021519, 0, 0, 0, 0},
                        /* Normal -K	L=0.300000	R00=70.00	R11=500.00 */
                        { 4.21741, 0.635314, 0.532433, 0.801507, -0.383637, -0.263773, -0.207934, -0.021355, -0.017665, -0.010184, -0.729857, -0.485098},
                        /* OffGround -K	L=0.300000	R00=70.00	R11=500.00 */
                        {0, 0, 0, 0, 0, 0, -0.207934, -0.021355, 0, 0, 0, 0},\






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