#pragma once
#ifndef 	LINKSOLVER_H
#define		LINKSOLVER_H

#include "cstdint"
#include "arm_math.h"
#ifdef __cplusplus

class cLinkSolver
{
	protected:
    /*Jacobian矩阵 {00,01,10,11}*/

	float JTRM_mat[4]={0};
	float JTRMInv_mat_c[4]={0}; // [1,0;0,1/L0]*Inv(JTRM), for easy calculation

	/*单位mm*/
	//腿长
	float L1 = 0.120f;
	float L2 = 0.218f;
	float MotoDistance = 0.100f;
	float HalfMotoDistance = MotoDistance/2.0f;
	
	/*关节电机弧度*/
	float phi1 = 0.0f;
	float phi4 = 0.0f;
	
	/*极限值*/
	float phi1_max = PI;
	float phi4_max = PI / 2;
    float phi1_min = PI / 2;
    float phi4_min = 0.0f;
	
	/*倒立摆长度*/
	float PendulumLength = 0.0f;
	/*倒立摆角度*/
	float PendulumRadian = PI/2;
	/*倒立摆坐标*/
	float CoorC[2]={0.0f,0.0f};
	/*第二象限节点坐标*/
	float CoorB[2]={0.0f,0.0f};
	float U2 = 0.0f;
	/*第二象限节点坐标*/
	float CoorD[2]={0.0f,0.0f};
	float U3 = 0.0f;

	
	public:
	void	Resolve(float phi4_radian, float psi1_radian);
	void	VMCCal(float *F, float *T);
	void	VMCRevCal(float *F, float *T);
    void    VMCVelCal(float *phi_dot, float *phi0_dot_vt_dot);

	inline float GetPendulumLen()
	{return PendulumLength;}
	
	inline float GetPendulumRadian()
	{return PendulumRadian;}
	
	inline void GetPendulumCoor(float* Coor) 
	{Coor[0]=this->CoorC[0];Coor[1]=this->CoorC[1];}
	inline void GetCoorB(float* Coor)
	{Coor[0]=this->CoorB[0];Coor[1]=this->CoorB[1];}
	inline void GetCoorD(float*Coor)
	{Coor[0]=this->CoorD[0];Coor[1]=this->CoorD[1];}
};

#endif
#endif