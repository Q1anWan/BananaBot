/*
	适用于轮恒加速度模型的卡尔曼滤波
	Author: NUAA-Wang Jian
*/
#ifndef KF_VELFUSION_H
#define KF_VELFUSION_H
#ifdef __cplusplus
extern "C" {
#endif
#include "main.h"
#include "arm_math.h"

#define mat arm_matrix_instance_f32
#define Matrix_Init arm_mat_init_f32
#define Matrix_Add arm_mat_add_f32
#define Matrix_Subtract arm_mat_sub_f32
#define Matrix_Multiply arm_mat_mult_f32
#define Matrix_Transpose arm_mat_trans_f32
#define Matrix_Inverse arm_mat_inverse_f32

typedef struct
{
	float *MeasuredVector;
	
	uint8_t xhatSize;
	uint8_t uSize;
	uint8_t zSize;
	uint8_t SkipEq1, SkipEq2, SkipEq3, SkipEq4, SkipEq5;
	
	mat xhat;      // x(k|k)
	mat xhatminus; // x(k|k-1)
	mat z;         // measurement vector z
	mat P;         // covariance matrix P(k|k)
	mat Pminus;    // covariance matrix P(k|k-1)
	mat A,AT; // state transition matrix A AT
	mat H, HT;     // measurement matrix H
	mat Q;         // process noise covariance matrix Q
	mat R;         // measurement noise covariance matrix R
	mat K;         // kalman gain  K                         
	mat S,temp_matrix,temp_matrix1,temp_vector, temp_vector1;
	mat ek,ekT;
	mat rk;
	
	int8_t MatStatus;

	float *xhat_data,*xhatminus_data;
	float *z_data;
	float *P_data,*Pminus_data;
	float *A_data,*AT_data;
	float *H_data,*HT_data;
	float *Q_data;
	float *R_data;
	float *K_data;
	float *S_data, *temp_matrix_data,*temp_matrix_data1, *temp_vector_data, *temp_vector_data1;

	float *ek_data;
	float *ekT_data;
	float *rk_data;
} VelFusionKF_t;

void VelFusionKF_Init(VelFusionKF_t *F, uint8_t xhatSize,uint8_t zSize);
void VelFusionKF_Update(VelFusionKF_t *kf);
void VelFusionKF_Reset(VelFusionKF_t *F);

#ifdef __cplusplus
}
#endif
#endif

