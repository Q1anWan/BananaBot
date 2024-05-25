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
class cFilterBTW2_1000Hz_100Hz
{
	arm_biquad_casd_df1_inst_f32 BTW{};
	float IIRBuf[4]={0};
	float IIRCoeff[5]={1.0f, 2.0f, 1.0f, 1.142980502539901133118860343529377132654f, -0.412801598096188770981029847462195903063f};
	float Gain = 0.067455273889071895587754568168747937307f;

	public:
    cFilterBTW2_1000Hz_100Hz(){arm_biquad_cascade_df1_init_f32(&BTW, 1, IIRCoeff, IIRBuf);}
	float Update(float data){
        float tmp;
        arm_biquad_cascade_df1_f32(&BTW, &data, &tmp ,1);
        return tmp*this->Gain;
    }
	void CleanBuf()
	{IIRBuf[0]=0;IIRBuf[1]=0;IIRBuf[2]=0;IIRBuf[3]=0;}
};

class cFilterBTW2_500Hz_100Hz
{
    arm_biquad_casd_df1_inst_f32 BTW{};
    float IIRBuf[4]={0};
    float IIRCoeff[5]={1.0f, 2.0f, 1.0f, 0.369527377351241470559983781640767119825f, -0.195815712655833140676264747526147402823f};
    float Gain = 0.20657208382614791752907024147134507075f;

public:
    cFilterBTW2_500Hz_100Hz(){arm_biquad_cascade_df1_init_f32(&BTW, 1, IIRCoeff, IIRBuf);}
    float Update(float data){
        float tmp;
        arm_biquad_cascade_df1_f32(&BTW, &data, &tmp ,1);
        return tmp*this->Gain;
    }
    void CleanBuf()
    {IIRBuf[0]=0;IIRBuf[1]=0;IIRBuf[2]=0;IIRBuf[3]=0;}
};

class cFilterBTW2_500Hz_10Hz
{
    arm_biquad_casd_df1_inst_f32 BTW{};
    float IIRBuf[4]={0};
    float IIRCoeff[5]={1.0f, 2.0f, 1.0f, 1.822694925196308268766642868285998702049f, -0.837181651256022618667884671594947576523};
    float Gain = 0.003621681514928642119099944096660692594f;

public:
    cFilterBTW2_500Hz_10Hz(){arm_biquad_cascade_df1_init_f32(&BTW, 1, IIRCoeff, IIRBuf);}
    float Update(float data){
        float tmp;
        arm_biquad_cascade_df1_f32(&BTW, &data, &tmp ,1);
        return tmp*this->Gain;
    }
    void CleanBuf()
    {IIRBuf[0]=0;IIRBuf[1]=0;IIRBuf[2]=0;IIRBuf[3]=0;}
};

class cFilterBTW2_500Hz_2Hz
{
    arm_biquad_casd_df1_inst_f32 BTW{};
    float IIRBuf[4]={0};
    float IIRCoeff[5]={1.0f, 2.0f, 1.0f, 1.964460580205231954309397224278654903173f, -0.965081173899134947546940566098783165216};
    float Gain = 0.000155148423475699032397095988855539872f;

public:
    cFilterBTW2_500Hz_2Hz(){arm_biquad_cascade_df1_init_f32(&BTW, 1, IIRCoeff, IIRBuf);}
    float Update(float data){
        float tmp;
        arm_biquad_cascade_df1_f32(&BTW, &data, &tmp ,1);
        return tmp*this->Gain;
    }
    void CleanBuf()
    {IIRBuf[0]=0;IIRBuf[1]=0;IIRBuf[2]=0;IIRBuf[3]=0;}
};
#endif
#endif