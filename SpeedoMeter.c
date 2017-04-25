/* 2012/08/10 created by k.nakagawa 速度計 */
#include "mytypes.h"

#include "Calc.h"

static int freqSpeedoMeter = 1;
static float prevDistance = 0.0F;

//*****************************************************************************
// 関数名 : SpeedoMeter
// 引数 : なし
// 返り値 : 現在のロボットの速度(m/s)
// 概要 : 速度計として用いる、必ずinitした周期で呼び出すこと
//*****************************************************************************
float SpeedoMeter(void)
{
	// 今回の距離の取得(m)
	float Distance = calc_getDistanceFromStart();
	
	// 前回との距離の差(m)
	float diff = Distance - prevDistance;
	
	prevDistance = Distance;
	
	return diff * (1000.0F / (float)freqSpeedoMeter);
}

//*****************************************************************************
// 関数名 : initSpeedoMeter
// 引数 : freq=周期(ms)(≠0)
// 返り値 : なし
// 概要 : 周期のセット、前回値の初期化
//*****************************************************************************
void initSpeedoMeter(U8 freq)
{
	freqSpeedoMeter = freq;
	prevDistance = 0.0F;
	return;
}
