/* 2012/08/08 created by s.sahara ��N���A�P�̃e�X�g�p */

/*
��������v�Z�֘A
*/

#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

#include "Control.h"

float ControlP(float kp, float command, float error, float max, float min)
{
	float hensa = command - error;
	float mv = kp * hensa;

	if(mv > max) mv = max;
	if(mv < min) mv = min;
	return mv;
}

float ControlPD(float kp, float kd, float command, float error, float hensa0, float* hensa1, float max, float min)
{
	*hensa1 = command - error;
	float mvp = kp * *hensa1;
	float mvd = kd * (*hensa1 - hensa0) / DELTA_T;

	float mv = mvp + mvd;
	if(mv > max) mv = max;
	if(mv < min) mv = min;
	return mv;
}

float ControlPID(float kp, float ki, float kd, float command, float error, float hensa0, float* hensa1, float* integral, float max, float min)
{
	*hensa1 = command - error;
	*integral += (*hensa1 + hensa0) / 2.0f * DELTA_T;
	float mvp = kp * *hensa1;
	float mvi = ki * *integral;
	float mvd = kd * (*hensa1 - hensa0) / DELTA_T;

	float mv = mvp + mvi + mvd;

#ifdef USE_LOG
	//ecrobot_bt_data_logger((S8)mvp, (S8)mvi);
#endif

	if(mv > max) mv = max;
	if(mv < min) mv = min;
	return mv;
}

// S���J�[�u
// Vaft : S����̒l
// Vbfr : S���O�̒l
// Tmax : S���I������[sec]
// T : S���J�n����̎���[sec]
float SCurve(float Vaft, float Vbfr, float Tmax, float* T)
{
	float Yrange, Xrange, x, x2, mv, offset;
	if(*T > Tmax) *T = Tmax; // �O�a����

	Yrange = (Vaft - Vbfr) * 0.5; // S�����x��(Y����Βlmax)
	Xrange = (Tmax * 0.5f);	// S�����Ԃ̔���
	x = *T - Xrange; // X�i�����̏ꍇ�A�O�������A�㔼�����ƂȂ�j
	if(x >= 0.0f)
	{
		x2 = Xrange - x;
	}
	else
	{
		x2 = Xrange + x;
	}
	mv = 1 - (x2 * x2) / (Xrange * Xrange); // �͈͍ő�Ł}1�ɂȂ�悤����
	*T += 0.004f; // ����������ɌĂяo���ŁA����������Ԃ����Z
	offset = Vbfr + Yrange;

	if(x >= 0.0f)
	{
		return mv * Yrange + offset;
	}
	else
	{
		return mv * Yrange * -1.0f + offset;
	}
}
