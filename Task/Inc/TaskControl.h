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
                        /* Normal -K	L=0.200000	R00=20.00	R11=10.00 */
                        {11.62604, 1.907133, 4.499196, 4.633974, -8.010212, -1.294614, -6.680449, -1.292370, -3.084553, -2.908114, -19.738663, -1.367282},
                        /* OffGround -K	L=0.200000	R00=20.00	R11=10.00 */
                        {0, 0, 0, 0, 0, 0, -6.680449, -1.292370, 0, 0, 0, 0},
                        /* Normal -K	L=0.210000	R00=20.00	R11=10.00 */
                        {11.78288, 1.920494, 4.514715, 4.628304, -7.869265, -1.325293, -6.553782, -1.257743, -3.038863, -2.831895, -19.849679, -1.420385},
                        /* OffGround -K	L=0.210000	R00=20.00	R11=10.00 */
                        {0, 0, 0, 0, 0, 0, -6.553782, -1.257743, 0, 0, 0, 0},
                        /* Normal -K	L=0.220000	R00=20.00	R11=10.00 */
                        {11.93092, 1.933216, 4.528947, 4.623096, -7.737879, -1.356718, -6.429822, -1.224869, -2.996210, -2.760762, -19.950931, -1.474168},
                        /* OffGround -K	L=0.220000	R00=20.00	R11=10.00 */
                        {0, 0, 0, 0, 0, 0, -6.429822, -1.224869, 0, 0, 0, 0},
                        /* Normal -K	L=0.230000	R00=20.00	R11=10.00 */
                        {12.07090, 1.945367, 4.541991, 4.618261, -7.615442, -1.388805, -6.308800, -1.193626, -2.956456, -2.694307, -20.043367, -1.528598},
                        /* OffGround -K	L=0.230000	R00=20.00	R11=10.00 */
                        {0, 0, 0, 0, 0, 0, -6.308800, -1.193626, 0, 0, 0, 0},
                        /* Normal -K	L=0.240000	R00=20.00	R11=10.00 */
                        {12.20348, 1.957007, 4.553940, 4.613729, -7.501387, -1.421480, -6.190876, -1.163903, -2.919462, -2.632166, -20.127828, -1.583641},
                        /* OffGround -K	L=0.240000	R00=20.00	R11=10.00 */
                        {0, 0, 0, 0, 0, 0, -6.190876, -1.163903, 0, 0, 0, 0},
                        /* Normal -K	L=0.250000	R00=20.00	R11=10.00 */
                        {12.32925, 1.968189, 4.564880, 4.609448, -7.395185, -1.454677, -6.076153, -1.135598, -2.885089, -2.574010, -20.205056, -1.639261},
                        /* OffGround -K	L=0.250000	R00=20.00	R11=10.00 */
                        {0, 0, 0, 0, 0, 0, -6.076153, -1.135598, 0, 0, 0, 0},
                        /* Normal -K	L=0.260000	R00=20.00	R11=10.00 */
                        {12.44875, 1.978959, 4.574890, 4.605373, -7.296344, -1.488340, -5.964694, -1.108616, -2.853204, -2.519545, -20.275711, -1.695423},
                        /* OffGround -K	L=0.260000	R00=20.00	R11=10.00 */
                        {0, 0, 0, 0, 0, 0, -5.964694, -1.108616, 0, 0, 0, 0},
                        /* Normal -K	L=0.270000	R00=20.00	R11=10.00 */
                        {12.56246, 1.989359, 4.584040, 4.601473, -7.204407, -1.522417, -5.856524, -1.082870, -2.823676, -2.468507, -20.340383, -1.752091},
                        /* OffGround -K	L=0.270000	R00=20.00	R11=10.00 */
                        {0, 0, 0, 0, 0, 0, -5.856524, -1.082870, 0, 0, 0, 0},
                        /* Normal -K	L=0.280000	R00=20.00	R11=10.00 */
                        {12.67081, 1.999425, 4.592399, 4.597719, -7.118949, -1.556863, -5.751646, -1.058282, -2.796380, -2.420654, -20.399595, -1.809231},
                        /* OffGround -K	L=0.280000	R00=20.00	R11=10.00 */
                        {0, 0, 0, 0, 0, 0, -5.751646, -1.058282, 0, 0, 0, 0},
                        /* Normal -K	L=0.290000	R00=20.00	R11=10.00 */
                        {12.77420, 2.009190, 4.600026, 4.594091, -7.039574, -1.591640, -5.650041, -1.034776, -2.771196, -2.375769, -20.453815, -1.866808},
                        /* OffGround -K	L=0.290000	R00=20.00	R11=10.00 */
                        {0, 0, 0, 0, 0, 0, -5.650041, -1.034776, 0, 0, 0, 0},
                        /* Normal -K	L=0.300000	R00=20.00	R11=10.00 */
                        {12.87298, 2.018681, 4.606975, 4.590571, -6.965912, -1.626712, -5.551673, -1.012286, -2.748011, -2.333654, -20.503465, -1.924790},
                        /* OffGround -K	L=0.300000	R00=20.00	R11=10.00 */
                        {0, 0, 0, 0, 0, 0, -5.551673, -1.012286, 0, 0, 0, 0},


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