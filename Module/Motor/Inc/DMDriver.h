/*
	DM Driver
	
	2320/8/06
	Fix temperatur decoder

	2320/7/29
	DM Driver
*/
#include <cstdint>

#ifndef DMDRIVER_H
#define DMDRIVER_H
#ifdef __cplusplus

#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f

/* DM4315 */
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define T_MIN -10.0f
#define T_MAX 10.0f


///* DM8006 */
//#define P_MIN -12.5f
//#define P_MAX 12.5f
//#define V_MIN -25.0f
//#define V_MAX 25.0f
//#define T_MIN -40.0f
//#define T_MAX 40.0f

///* DM8009 */
//#define P_MIN -12.5f
//#define P_MAX 12.5f
//#define V_MIN -30.0f
//#define V_MAX 30.0f
//#define T_MIN -40.0f//40
//#define T_MAX 40.0f

typedef struct{
	uint8_t state;
	float pos;
	float vel;
	float toq;
	uint8_t Tmos;
	uint8_t Tcoil;
	float Kp;
	float Kd;
}Motor_Inf;

class cDMMotor
{
	protected:
		virtual void CAN_Transmit(uint8_t *pdata) = 0;
		Motor_Inf MTR={0},CMD={0};
		uint16_t float_to_uint(float x, float x_min, float x_max, int bits)
		{
			/// Converts a float to an unsigned int, given range and number of bits ///
			float span = x_max - x_min;
			float offset = x_min;
			return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
		}

		float uint_to_float(int x_int, float x_min, float x_max, int bits)
		{
			/// converts unsigned int to float, given range and number of bits ///
			float span = x_max - x_min;
			float offset = x_min;
			return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
		}
	public:
		
		void EnableMotor(void);
		void DisableMotor(void);
		void SetZero(void);
		void MITUpdate(float Position, float Velocity, float KP, float KD, float Torque);
		void MITTransmit(void);
		uint8_t MessageDecode(uint8_t *buf);

        virtual float GetRadian(void)
		{return this->MTR.pos;}
		virtual float GetVelocity(void)
		{return this->MTR.vel;}
        virtual uint8_t GetTem(void)
		{return this->MTR.Tcoil;}
        virtual float GetToqReal(void)
		{return this->MTR.toq;}
};

#endif
#endif