/*
	LinkSolver 
	Pendulum Forward And Inverse Kinematics Solver

    2024/5/25 V2.0
	Strengthen performance;
    Fix bug on calculating MR^TJ matrix;

	2023/7/31 V1.2
	Fix CoorD Error
*/

#include "LinkSolver.h"
#include "cmath"

void cLinkSolver::Resolve(float phi4_radian, float psi1_radian) {
    this->phi4 = phi4_radian;
    this->phi1 = psi1_radian;

    float SIN1 = arm_sin_f32(this->phi1);
    float COS1 = arm_cos_f32(this->phi1);
    float SIN4 = arm_sin_f32(this->phi4);
    float COS4 = arm_cos_f32(this->phi4);

    float xdb = this->MotoDistance + this->L1 * (COS4 - COS1);
    float ydb = this->L1 * (SIN4 - SIN1);

    float A0 = 2 * L2 * xdb;
    float B0 = 2 * L2 * ydb;
    float C0 = xdb * xdb + ydb * ydb;// equal to lbd^2


    /*计算u2*/
    float u2t = 0.0f;
    arm_atan2_f32((B0 + sqrtf(A0 * A0 + B0 * B0 - C0 * C0)), (A0 + C0), &u2t);
    this->U2 = 2.0f * (u2t + PI); //Move u2t to 0 - 2PI


    /*计算B坐标*/
    this->CoorB[0] = this->L1 * COS1 - this->HalfMotoDistance;
    this->CoorB[1] = this->L1 * SIN1;
    /*计算C坐标*/
    this->CoorC[0] = this->CoorB[0] + L2 * arm_cos_f32(this->U2);
    this->CoorC[1] = this->CoorB[1] + L2 * arm_sin_f32(this->U2);
    /*计算D坐标*/
    this->CoorD[0] = this->L1 * COS4 + this->HalfMotoDistance;
    this->CoorD[1] = this->L1 * SIN4;
    /*计算u3*/
    float u3t;
    arm_atan2_f32(this->CoorD[1] - this->CoorC[1], this->CoorD[0] - this->CoorC[0], &u3t);
    this->U3 = u3t + PI; //Move u3t to 0 - 2PI

    /*输出摆长摆角*/
    arm_atan2_f32(this->CoorC[1], this->CoorC[0], &this->PendulumRadian);
    this->PendulumLength = sqrtf(this->CoorC[0] * this->CoorC[0] + this->CoorC[1] * this->CoorC[1]);

    /*计算J与J^T*R*M*/
    float sin32 = arm_sin_f32(this->U3 - this->U2);
    float sin12 = arm_sin_f32(this->phi1 - this->U2);
    float sin34 = arm_sin_f32(this->U3 - this->phi4);
    float cos03 = arm_cos_f32(this->PendulumRadian - this->U3);
    float cos02 = arm_cos_f32(this->PendulumRadian - this->U2);
    float sin03 = arm_sin_f32(this->PendulumRadian - this->U3);
    float sin02 = arm_sin_f32(this->PendulumRadian - this->U2);

    JTRM_mat[0] = L1 * sin03 * sin12 / sin32;
    JTRM_mat[1] = L1 * cos03 * sin12 / (sin32 * PendulumLength);
    JTRM_mat[2] = L1 * sin02 * sin34 / sin32;
    JTRM_mat[3] = L1 * cos02 * sin34 / (sin32 * PendulumLength);

    JTRMRev_mat[0] = -cos02 / (sin12 * L1);
    JTRMRev_mat[1] = cos03 / (sin34 * L1);
    JTRMRev_mat[2] = PendulumLength * sin02 / (sin12 * L1);
    JTRMRev_mat[3] = -PendulumLength * sin03 / (sin34 * L1);

    MTRTJ_mat[0] = L1 * sin03 * sin12 / sin32;
    MTRTJ_mat[1] = L1 * sin02 * sin34 / sin32;
    MTRTJ_mat[2] = L1 * cos03 * sin12 / (sin32 * PendulumLength);
    MTRTJ_mat[3] = L1 * cos02 * sin34 / (sin32 * PendulumLength);
}

/*正向VMC 由Force Torque->T3 T2*/
void cLinkSolver::VMCCal(float *FT, float *Tmotor) {
    Tmotor[0] = this->JTRM_mat[0] * FT[0] + this->JTRM_mat[1] * FT[1];
    Tmotor[1] = this->JTRM_mat[2] * FT[0] + this->JTRM_mat[3] * FT[1];
}

/*逆向VMC 由MOTOR_FORWARD MOTOR_BACKWORD -> Force Torque*/
void cLinkSolver::VMCRevCal(float *FT, float *Tmotor) {
    FT[0] = this->JTRMRev_mat[0] * Tmotor[0] + this->JTRMRev_mat[1] * Tmotor[1];
    FT[1] = this->JTRMRev_mat[2] * Tmotor[0] + this->JTRMRev_mat[3] * Tmotor[1];
}

/*逆向VMC 由电机角速度->沿着摆方向和垂直摆方向角速度*/
void cLinkSolver::VMCVelCal(float *phi_dot, float *v_dot) {
    v_dot[0] = this->MTRTJ_mat[0] * phi_dot[0] + this->MTRTJ_mat[1] * phi_dot[1];
    v_dot[1] = this->MTRTJ_mat[2] * phi_dot[0] + this->MTRTJ_mat[3] * phi_dot[1];
}