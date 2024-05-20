/*
	DM Driver
	
	2320/8/06
	Fix temperatur decoder

	2320/7/29
	DM Driver
*/
#include "DMDriver.h"

void cDMMotor::EnableMotor(void)
{
	uint8_t txbuf[8]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
	this->CAN_Transmit(txbuf);
}

void cDMMotor::DisableMotor(void)
{
	uint8_t txbuf[8]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};
	this->CAN_Transmit(txbuf);
}

void cDMMotor::SetZero(void)
{
	uint8_t txbuf[8]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE};
	this->CAN_Transmit(txbuf);
}
	
void cDMMotor::MITUpdate(float Position, float Velocity, float KP, float KD, float Torque)
{
	this->CMD.pos = Position;
	this->CMD.vel = Velocity;
	this->CMD.Kp = KP;
	this->CMD.Kd = KD;
	this->CMD.toq = Torque;
}

void cDMMotor::MITTransmit(void)
{
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	pos_tmp = this->float_to_uint(this->CMD.pos, P_MIN, P_MAX, 16);
	vel_tmp = this->float_to_uint(this->CMD.vel, V_MIN, V_MAX, 12);
	kp_tmp  = this->float_to_uint(this->CMD.Kp, KP_MIN, KP_MAX, 12);
	kd_tmp  = this->float_to_uint(this->CMD.Kd, KD_MIN, KD_MAX, 12);
	tor_tmp = this->float_to_uint(this->CMD.toq, T_MIN, T_MAX, 12);

	volatile uint8_t txbuf[8];
	txbuf[0] = (pos_tmp >> 8);
	txbuf[1] = pos_tmp;
	txbuf[2] = (vel_tmp >> 4);
	txbuf[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	txbuf[4] = kp_tmp;
	txbuf[5] = (kd_tmp >> 4);
	txbuf[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	txbuf[7] = tor_tmp;
	
	this->CAN_Transmit((uint8_t*)txbuf);
}

uint8_t cDMMotor::MessageDecode(uint8_t *buf)
{
	this->MTR.state = buf[0]>>4;
	uint16_t tmp;
	
	tmp = (buf[1]<<8)|buf[2];
	this->MTR.pos = this->uint_to_float(tmp,P_MIN,P_MAX,16);
	tmp = (buf[3]<<4)|(buf[4]>>4);
	this->MTR.vel = this->uint_to_float(tmp,V_MIN,V_MAX,12);
	tmp = ((buf[4]&0xF)<<8)|buf[5];
	this->MTR.toq = this->uint_to_float(tmp,V_MIN,V_MAX,12);
	this->MTR.Tmos = buf[6];
	this->MTR.Tcoil = buf[7];
	return 1;
}
