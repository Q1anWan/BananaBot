/*
	适用于轮恒加速度模型的卡尔曼滤波
	Author: NUAA-Wang Jian
	
*/
#include "KF_VelFusion.h"
#include "tx_api.h"

static uint16_t sizeof_float = sizeof(float);

extern TX_BYTE_POOL MathPool;
static void* user_malloc(size_t size)
{
	void* tmp;
    tx_byte_allocate(&MathPool, (VOID **)&tmp, size, TX_NO_WAIT);
	return tmp;
}

void VelFusionKF_Init(VelFusionKF_t *kf, uint8_t xhatSize,uint8_t zSize)
{
	kf->xhatSize = xhatSize;
	kf->zSize = zSize;

	kf->MeasuredVector = (float *)user_malloc(sizeof_float * zSize);
	memset(kf->MeasuredVector, 0, sizeof_float * zSize);

	// xhat x(k|k)
	kf->xhat_data = (float *)user_malloc(sizeof_float * xhatSize);
	memset(kf->xhat_data, 0, sizeof_float * xhatSize);
	Matrix_Init(&kf->xhat, kf->xhatSize, 1, (float *)kf->xhat_data);

	// xhatminus x(k|k-1)
	kf->xhatminus_data = (float *)user_malloc(sizeof_float * xhatSize);
	memset(kf->xhatminus_data, 0, sizeof_float * xhatSize);
	Matrix_Init(&kf->xhatminus, kf->xhatSize, 1, (float *)kf->xhatminus_data);

	// measurement vector z
	kf->z_data = (float *)user_malloc(sizeof_float * zSize);
	memset(kf->z_data, 0, sizeof_float * zSize);
	Matrix_Init(&kf->z, kf->zSize, 1, (float *)kf->z_data);

	// covariance matrix P(k|k)
	kf->P_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
	memset(kf->P_data, 0, sizeof_float * xhatSize * xhatSize);
	Matrix_Init(&kf->P, kf->xhatSize, kf->xhatSize, (float *)kf->P_data);

	//create covariance matrix P(k|k-1)
	kf->Pminus_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
	memset(kf->Pminus_data, 0, sizeof_float * xhatSize * xhatSize);
	Matrix_Init(&kf->Pminus, kf->xhatSize, kf->xhatSize, (float *)kf->Pminus_data);

	// state transition matrix A AT
	kf->A_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
	kf->AT_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
	memset(kf->A_data, 0, sizeof_float * xhatSize * xhatSize);
	memset(kf->AT_data, 0, sizeof_float * xhatSize * xhatSize);
	Matrix_Init(&kf->A, kf->xhatSize, kf->xhatSize, (float *)kf->A_data);
	Matrix_Init(&kf->AT, kf->xhatSize, kf->xhatSize, (float *)kf->AT_data);

	 // measurement matrix H
	kf->H_data = (float *)user_malloc(sizeof_float * zSize * xhatSize);
	kf->HT_data = (float *)user_malloc(sizeof_float * xhatSize * zSize);
	memset(kf->H_data, 0, sizeof_float * zSize * xhatSize);
	memset(kf->HT_data, 0, sizeof_float * xhatSize * zSize);
	Matrix_Init(&kf->H, kf->zSize, kf->xhatSize, (float *)kf->H_data);
	Matrix_Init(&kf->HT, kf->xhatSize, kf->zSize, (float *)kf->HT_data);

	// process noise covariance matrix Q
	kf->Q_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
	memset(kf->Q_data, 0, sizeof_float * xhatSize * xhatSize);
	Matrix_Init(&kf->Q, kf->xhatSize, kf->xhatSize, (float *)kf->Q_data);

	// measurement noise covariance matrix R
	kf->R_data = (float *)user_malloc(sizeof_float * zSize * zSize);
	memset(kf->R_data, 0, sizeof_float * zSize * zSize);
	Matrix_Init(&kf->R, kf->zSize, kf->zSize, (float *)kf->R_data);

	// kalman gain K
	kf->K_data = (float *)user_malloc(sizeof_float * xhatSize * zSize);
	memset(kf->K_data, 0, sizeof_float * xhatSize * zSize);
	Matrix_Init(&kf->K, kf->xhatSize, kf->zSize, (float *)kf->K_data);

	//Chi-Squre Test
	kf->ek_data = (float *)user_malloc(sizeof_float * zSize);
	kf->ekT_data = (float *)user_malloc(sizeof_float * zSize);
	kf->rk_data = (float *)user_malloc(sizeof_float);
	memset(kf->ek_data, 0, sizeof_float * zSize);
	memset(kf->ekT_data, 0, sizeof_float * zSize);
	memset(kf->rk_data, 0, sizeof_float);
	Matrix_Init(&kf->ek, kf->zSize, 1, (float *)kf->ek_data);
	Matrix_Init(&kf->ekT, 1,kf->zSize , (float *)kf->ekT_data);
	Matrix_Init(&kf->rk, 1, 1, (float *)kf->rk_data);

	//temp var
	kf->S_data = (float *)user_malloc(sizeof_float * kf->xhatSize * kf->xhatSize);
	kf->temp_matrix_data = (float *)user_malloc(sizeof_float * kf->xhatSize * kf->xhatSize);
	kf->temp_matrix_data1 = (float *)user_malloc(sizeof_float * kf->xhatSize * kf->xhatSize);
	kf->temp_vector_data = (float *)user_malloc(sizeof_float * kf->xhatSize);
	kf->temp_vector_data1 = (float *)user_malloc(sizeof_float * kf->xhatSize);
	Matrix_Init(&kf->S, kf->xhatSize, kf->xhatSize, (float *)kf->S_data);
	Matrix_Init(&kf->temp_matrix, kf->xhatSize, kf->xhatSize, (float *)kf->temp_matrix_data);
	Matrix_Init(&kf->temp_matrix1, kf->xhatSize, kf->xhatSize, (float *)kf->temp_matrix_data1);
	Matrix_Init(&kf->temp_vector, kf->xhatSize, 1, (float *)kf->temp_vector_data);
	Matrix_Init(&kf->temp_vector1, kf->xhatSize, 1, (float *)kf->temp_vector_data1);


//			Matrix_Init(&F->ek,3,1,ek_data);
//			Matrix_Init(&F->ekT,1,3,ekT_data);
//			Matrix_Init(&F->rk,1,1,&rk_data);
}

void VelFusionKF_Update(VelFusionKF_t *kf)
{

	memcpy(kf->z_data, kf->MeasuredVector, sizeof_float * kf->zSize);
	memset(kf->MeasuredVector, 0, sizeof_float * kf->zSize);	
  //1. xhat'(k)= A xhat(k-1)
	if (!kf->SkipEq1)
		kf->MatStatus = Matrix_Multiply(&kf->A, &kf->xhat, &kf->xhatminus);
  
	// 预测更新
  //2. P'(k) = A P(k-1) AT + Q
	if (!kf->SkipEq2)
	{
		kf->MatStatus = Matrix_Transpose(&kf->A,&kf->AT);
		kf->MatStatus = Matrix_Multiply(&kf->A, &kf->P, &kf->Pminus);
		kf->temp_matrix.numRows = kf->Pminus.numRows;
		kf->temp_matrix.numCols = kf->AT.numCols;
		kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->AT, &kf->temp_matrix); //temp_matrix = F P(k-1) FT
		kf->MatStatus = Matrix_Add(&kf->temp_matrix, &kf->Q, &kf->Pminus);
		//Chi-Squre Test  rk=ekT(Hk Pkmin Hkt +Rk)-1ek
		kf->MatStatus	=	Matrix_Multiply(&kf->H,&kf->xhatminus,&kf->ek);
		kf->MatStatus	=	Matrix_Subtract(&kf->z,&kf->ek,&kf->ek);
		kf->MatStatus	=	Matrix_Transpose(&kf->ek,&kf->ekT);
		
		kf->temp_matrix.numRows = kf->H.numRows;
		kf->temp_matrix.numCols = kf->Pminus.numCols;
		kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
		kf->temp_matrix1.numCols = kf->HT.numCols;
		kf->MatStatus	=	Matrix_Multiply(&kf->H,&kf->Pminus,&kf->temp_matrix);
		kf->MatStatus	=	Matrix_Multiply(&kf->temp_matrix,&kf->HT,&kf->temp_matrix1);
		kf->MatStatus	=	Matrix_Add(&kf->temp_matrix1,&kf->R,&kf->temp_matrix1);
		kf->temp_matrix.numRows = kf->temp_matrix1.numRows;
		kf->temp_matrix.numCols = kf->temp_matrix1.numCols;
		kf->MatStatus	=	Matrix_Inverse(&kf->temp_matrix1, &kf->temp_matrix);
		
		kf->temp_vector.numRows = 1;
		kf->temp_vector.numCols = kf->temp_matrix.numCols;
		kf->MatStatus	= Matrix_Multiply(&kf->ekT,&kf->temp_matrix,&kf->temp_vector);
		kf->MatStatus	= Matrix_Multiply(&kf->temp_vector,&kf->ek,&kf->rk);
	}
	
   // 量测更新
  //3. K(k) = P'(k) HT / (H P'(k) HT + R)
	if (!kf->SkipEq3)
	{
		kf->MatStatus = Matrix_Transpose(&kf->H, &kf->HT); 
		kf->temp_matrix.numRows = kf->H.numRows;
		kf->temp_matrix.numCols = kf->Pminus.numCols;
		kf->MatStatus = Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix); //temp_matrix = H·P'(k)
		kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
		kf->temp_matrix1.numCols = kf->HT.numCols;
		kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1); //temp_matrix1 = H·P'(k)·HT
		kf->S.numRows = kf->R.numRows;
		kf->S.numCols = kf->R.numCols;
		kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S); //S = H P'(k) HT + R
		kf->MatStatus = Matrix_Inverse(&kf->S, &kf->temp_matrix1);     //temp_matrix1 = inv(H·P'(k)·HT + R)
		kf->temp_matrix.numRows = kf->Pminus.numRows;
		kf->temp_matrix.numCols = kf->HT.numCols;
		kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix); //temp_matrix = P'(k)·HT
		kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);
	}
  //4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
	if (!kf->SkipEq4)
	{
			kf->temp_vector.numRows = kf->H.numRows;
			kf->temp_vector.numCols = 1;
			kf->MatStatus = Matrix_Multiply(&kf->H, &kf->xhatminus, &kf->temp_vector); //temp_vector = H xhat'(k)
			kf->temp_vector1.numRows = kf->z.numRows;
			kf->temp_vector1.numCols = 1;
			kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1); //temp_vector1 = z(k) - H·xhat'(k)
			kf->temp_vector.numRows = kf->K.numRows;
			kf->temp_vector.numCols = 1;
			kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector); //temp_vector = K(k)·(z(k) - H·xhat'(k))
			kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);
	}

  //5. P(k) = (1-K(k)H)P'(k) ==> P(k) = P'(k)-K(k)·H·P'(k)
	if (!kf->SkipEq5)
	{
			kf->temp_matrix.numRows = kf->K.numRows;
			kf->temp_matrix.numCols = kf->H.numCols;
			kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
			kf->temp_matrix1.numCols = kf->Pminus.numCols;
			kf->MatStatus = Matrix_Multiply(&kf->K, &kf->H, &kf->temp_matrix);                 //temp_matrix = K(k)·H
			kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->Pminus, &kf->temp_matrix1); //temp_matrix1 = K(k)·H·P'(k)
			kf->MatStatus = Matrix_Subtract(&kf->Pminus, &kf->temp_matrix1, &kf->P);
	}
}//CA模型

void VelFusionKF_Reset(VelFusionKF_t *kf)//卡尔曼滤波器复位
{
	memset(kf->xhat.pData, 0, sizeof(float) * kf->xhat.numRows);
	memset(kf->xhatminus.pData, 0, sizeof(float) * kf->xhatminus.numRows);
//	memcpy(F->P.pData,Pminus_data,sizeof(Pminus_data));
	memset(kf->P.pData, 0, sizeof(float) * kf->P.numRows*kf->P.numRows);
}


