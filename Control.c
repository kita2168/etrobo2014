/* 2012/08/08 created by s.sahara 難所クリア単体テスト用 */

/*
自動制御計算関連
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

// S字カーブ
// Vaft : S字後の値
// Vbfr : S字前の値
// Tmax : S字終了時間[sec]
// T : S字開始からの時間[sec]
float SCurve(float Vaft, float Vbfr, float Tmax, float* T)
{
	float Yrange, Xrange, x, x2, mv, offset;
	if(*T > Tmax) *T = Tmax; // 飽和処理

	Yrange = (Vaft - Vbfr) * 0.5; // S字レベル(Y軸絶対値max)
	Xrange = (Tmax * 0.5f);	// S字期間の半分
	x = *T - Xrange; // X（加速の場合、前半が負、後半が正となる）
	if(x >= 0.0f)
	{
		x2 = Xrange - x;
	}
	else
	{
		x2 = Xrange + x;
	}
	mv = 1 - (x2 * x2) / (Xrange * Xrange); // 範囲最大で±1になるよう調節
	*T += 0.004f; // 制御周期毎に呼び出しで、制御周期時間を加算
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
