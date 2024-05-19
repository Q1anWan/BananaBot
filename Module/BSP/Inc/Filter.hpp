/*
	2023.12.23
	V1.2
	Add BufClean function
*/
#pragma once
#ifndef FILTER_H
#define FILTER_H
#include "main.h"
#include "arm_math.h"
#ifdef __cplusplus
class cFilterBTW2_100Hz
{
	arm_biquad_casd_df1_inst_f32 BTW;
	float IIRBuf[4]={0};
	float IIRCoeff[5]={1.0f, 2.0f, 1.0f, 1.142980502539901133118860343529377132654f, -0.412801598096188770981029847462195903063f};
	float Gain = 0.067455273889071895587754568168747937307f;

	public:
    cFilterBTW2_100Hz(){arm_biquad_cascade_df1_init_f32(&BTW, 1, IIRCoeff, IIRBuf);}
	float Update(float data){
        float tmp;
        arm_biquad_cascade_df1_f32(&BTW, &data, &tmp ,1);
        return tmp*this->Gain;
    }
	void CleanBuf(void)
	{IIRBuf[0]=0;IIRBuf[1]=0;IIRBuf[2]=0;IIRBuf[3]=0;}
};

#endif
#endif