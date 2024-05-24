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
                        /* Normal -K	L=0.200000	R00=8.00	R11=13.00 */
                        {15.87709, 2.462891, 7.612573, 6.835189, -8.263785, -1.571163, -2.983159, -0.528598, -1.673087, -1.308408, -18.855566, -1.501275},
                        /* OffGround -K	L=0.200000	R00=8.00	R11=13.00 */
                        {0, 0, 0, 0, 0, 0, -2.983159, -0.528598, 0, 0, 0, 0},
                        /* Normal -K	L=0.210000	R00=8.00	R11=13.00 */
                        {16.02707, 2.469410, 7.619916, 6.802775, -8.121601, -1.609870, -2.918859, -0.512315, -1.652389, -1.271370, -18.892896, -1.551756},
                        /* OffGround -K	L=0.210000	R00=8.00	R11=13.00 */
                        {0, 0, 0, 0, 0, 0, -2.918859, -0.512315, 0, 0, 0, 0},
                        /* Normal -K	L=0.220000	R00=8.00	R11=13.00 */
                        {16.16845, 2.475782, 7.626469, 6.772964, -7.992031, -1.649647, -2.856976, -0.497008, -1.633679, -1.237190, -18.926332, -1.603021},
                        /* OffGround -K	L=0.220000	R00=8.00	R11=13.00 */
                        {0, 0, 0, 0, 0, 0, -2.856976, -0.497008, 0, 0, 0, 0},
                        /* Normal -K	L=0.230000	R00=8.00	R11=13.00 */
                        {16.30201, 2.482034, 7.632301, 6.745443, -7.874037, -1.690373, -2.797466, -0.482592, -1.616832, -1.205611, -18.956299, -1.655016},
                        /* OffGround -K	L=0.230000	R00=8.00	R11=13.00 */
                        {0, 0, 0, 0, 0, 0, -2.797466, -0.482592, 0, 0, 0, 0},
                        /* Normal -K	L=0.240000	R00=8.00	R11=13.00 */
                        {16.42848, 2.488188, 7.637474, 6.719946, -7.766686, -1.731942, -2.740274, -0.468992, -1.601729, -1.176404, -18.983165, -1.707687},
                        /* OffGround -K	L=0.240000	R00=8.00	R11=13.00 */
                        {0, 0, 0, 0, 0, 0, -2.740274, -0.468992, 0, 0, 0, 0},
                        /* Normal -K	L=0.250000	R00=8.00	R11=13.00 */
                        {16.54847, 2.494262, 7.642044, 6.696248, -7.669135, -1.774261, -2.685334, -0.456142, -1.588257, -1.149368, -19.007249, -1.760985},
                        /* OffGround -K	L=0.250000	R00=8.00	R11=13.00 */
                        {0, 0, 0, 0, 0, 0, -2.685334, -0.456142, 0, 0, 0, 0},
                        /* Normal -K	L=0.260000	R00=8.00	R11=13.00 */
                        {16.66254, 2.500272, 7.646063, 6.674158, -7.580625, -1.817250, -2.632577, -0.443980, -1.576309, -1.124326, -19.028829, -1.814862},
                        /* OffGround -K	L=0.260000	R00=8.00	R11=13.00 */
                        {0, 0, 0, 0, 0, 0, -2.632577, -0.443980, 0, 0, 0, 0},
                        /* Normal -K	L=0.270000	R00=8.00	R11=13.00 */
                        {16.77118, 2.506231, 7.649576, 6.653510, -7.500466, -1.860837, -2.581930, -0.432452, -1.565786, -1.101118, -19.048150, -1.869274},
                        /* OffGround -K	L=0.270000	R00=8.00	R11=13.00 */
                        {0, 0, 0, 0, 0, 0, -2.581930, -0.432452, 0, 0, 0, 0},
                        /* Normal -K	L=0.280000	R00=8.00	R11=13.00 */
                        {16.87483, 2.512149, 7.652623, 6.634163, -7.428035, -1.904959, -2.533320, -0.421511, -1.556596, -1.079601, -19.065425, -1.924180},
                        /* OffGround -K	L=0.280000	R00=8.00	R11=13.00 */
                        {0, 0, 0, 0, 0, 0, -2.533320, -0.421511, 0, 0, 0, 0},
                        /* Normal -K	L=0.290000	R00=8.00	R11=13.00 */
                        {16.97388, 2.518036, 7.655242, 6.615992, -7.362762, -1.949560, -2.486674, -0.411111, -1.548650, -1.059648, -19.080843, -1.979541},
                        /* OffGround -K	L=0.290000	R00=8.00	R11=13.00 */
                        {0, 0, 0, 0, 0, 0, -2.486674, -0.411111, 0, 0, 0, 0},
                        /* Normal -K	L=0.300000	R00=8.00	R11=13.00 */
                        {17.06869, 2.523900, 7.657467, 6.598888, -7.304129, -1.994591, -2.441919, -0.401213, -1.541868, -1.041144, -19.094569, -2.035321},
                        /* OffGround -K	L=0.300000	R00=8.00	R11=13.00 */
                        {0, 0, 0, 0, 0, 0, -2.441919, -0.401213, 0, 0, 0, 0},




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