#ifndef 	LINKSOLVER_H
#define		LINKSOLVER_H

#include "cstdint"
#include "arm_math.h"
#ifdef __cplusplus

enum eLinkStatue
{
	LINK_NORMAL	= 0,
	LINK_ERROR	= 1
};

class cLinkSolver
{
	protected:
	float JacobianBuf[4]={0};
	float JacobianRevBuf[4]={0};
	/*Jacobian 矩阵*/
	arm_matrix_instance_f32 MatVMCJ = {2, 2, JacobianBuf};
	arm_matrix_instance_f32 MatVMCJRev = {2, 2, JacobianRevBuf};
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
	/*杆状态*/
	eLinkStatue LinkStatue = LINK_ERROR;
	
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
	void	Resolve(void);
	void	VMCUpdate(void);
	void	VMCCal(float *F, float *T);
	void	VMCRevCal(float *F, float *T);
	void    VMCRevCal_Radian(float *R,float *x_dot);
	void	SetRadLimit(float phi4_max, float phi1_max)
	{
		this->phi4_max = phi4_max;
		this->phi1_max = phi1_max;
	}
	
	uint8_t	InputLink(float phi4_radian, float psi1_radian);
	
	inline uint8_t GetLinkStatue(void)
	{return (uint8_t)this->LinkStatue;}
	
	inline float GetPendulumLen(void)
	{return PendulumLength;}
	
	inline float GetPendulumRadian(void)
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