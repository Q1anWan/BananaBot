/*
	LinkSolver 
	Pendulum postive motion solver
	
	2023/7/31 V1.2
	Fix CoorD Error

*/

#include "LinkSolver.h"

void cLinkSolver::Resolve(void)
{	
	volatile float SIN2=arm_sin_f32(this->Theta2);
	volatile float COS2=arm_cos_f32(this->Theta2);
	volatile float SIN3=arm_sin_f32(this->Theta3);
	volatile float COS3=arm_cos_f32(this->Theta3);

	float xdb = this->MotoDistance + this->L1 * (COS3-COS2);
	float ydb = this->L1 * (SIN3-SIN2);
	
	float A0 = 2 * L2 * xdb;
	float B0 = 2 * L2 * ydb;
	float C0 = xdb*xdb + ydb*ydb;
	float lBD = sqrtf(C0);
	
	/*����u2*/
	float u2t=0.0f;
	arm_atan2_f32( (B0+sqrtf(A0*A0 + B0*B0 - C0*C0)) , (A0+C0) , &u2t);
	this->U2=2.0f*(u2t+PI);
	
	
	
	/*����B����*/
	this->CoorB[0] = this->L1 * COS2 - this->HalfMotoDistance;
	this->CoorB[1] = this->L1 * SIN2;
	/*����C����*/
	this->CoorC[0] = this->CoorB[0] + L2 * arm_cos_f32(this->U2);
	this->CoorC[1] = this->CoorB[1] + L2 * arm_sin_f32(this->U2);
	/*����D����*/
	this->CoorD[0] = this->L1 * COS3 + this->HalfMotoDistance;
	this->CoorD[1] = this->L1 * SIN3;
	/*����u3*/
	float u3t;
	arm_atan2_f32(this->CoorD[1]-this->CoorC[1], this->CoorD[0]-this->CoorC[0], &u3t);
	this->U3 = u3t + PI;
	
	/*����ڳ��ڽ�*/
	arm_atan2_f32(this->CoorC[1],this->CoorC[0],&this->PendulumRadian);
	this->PendulumLength = sqrtf(this->CoorC[0]*this->CoorC[0] + this->CoorC[1]*this->CoorC[1]);
}

uint8_t	cLinkSolver::InputLink(float Theta3, float Theta2)
{
	this->Theta3 = Theta3;
	this->Theta2 = Theta2;
	
	if(Theta3>this->Theta3Max)
	{this->LinkStatue = LINK_ERROR;}
	else if(Theta2<this->Theta2Min)
	{this->LinkStatue = LINK_ERROR;}
	else
	{this->LinkStatue = LINK_NORMAL;}
	
	return (uint8_t)this->LinkStatue;
}
void cLinkSolver::VMCUpdate(void)
{
	/*�м����*/
	volatile float sin32 = arm_sin_f32(this->U3-this->U2);
	volatile float sin12 = arm_sin_f32(this->Theta2 - this->U2);
	volatile float sin34 = arm_sin_f32(this->U3 - this->Theta3);
	
	/*�ӿƱȾ���*/
	this->JacobianBuf[0] = this->L1 * arm_sin_f32(this->PendulumRadian - this->U3) * sin12 / sin32;
	this->JacobianBuf[1] = this->L1 * arm_cos_f32(this->PendulumRadian - this->U3) * sin12 / (this->PendulumLength *sin32);
	this->JacobianBuf[2] = this->L1 * arm_sin_f32(this->PendulumRadian - this->U2) * sin34 / sin32;
	this->JacobianBuf[3] = this->L1 * arm_cos_f32(this->PendulumRadian - this->U2) * sin34 / (this->PendulumLength *sin32);
	
	
}
/*����VMC ��Force Torque->T3 T2*/
void cLinkSolver::VMCCal(float *FT, float *Tmotor)
{
	this->VMCUpdate();
	/*����˷�*/
	Tmotor[0] = this->JacobianBuf[0]*FT[0] + this->JacobianBuf[1]*FT[1];
	Tmotor[1] = this->JacobianBuf[2]*FT[0] + this->JacobianBuf[3]*FT[1];
}

/*����VMC ��T3 T2 -> Force Torque*/
void cLinkSolver::VMCRevCal(float *FT, float *Tmotor)
{
	this->VMCUpdate();
	/*����*/
	arm_mat_inverse_f32(&this->MatVMCJ,&this->MatVMCJRev);
	FT[0] = this->JacobianRevBuf[0]*Tmotor[0] + this->JacobianRevBuf[1]*Tmotor[1];
	FT[1] = this->JacobianRevBuf[2]*Tmotor[0] + this->JacobianRevBuf[3]*Tmotor[1];	
}
/*����VMC �ɵ�����ٶ�->�ڳ��ٶȺͰڳ����ٶ�*/
void cLinkSolver::VMCRevCal_Radian(float *R,float *x_dot)
{
	this->VMCUpdate();
	/*����*/
	arm_mat_inverse_f32(&this->MatVMCJ,&this->MatVMCJRev);
	x_dot[0] = this->JacobianRevBuf[0]*R[0] + this->JacobianRevBuf[1]*R[1];
	x_dot[1] = this->JacobianRevBuf[2]*R[0] + this->JacobianRevBuf[3]*R[1];	  	
}