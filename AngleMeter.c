/* 2013/09/15 created by s.sahara 角度計 */
#include "mytypes.h"
#include "ecrobot_interface.h"
#include "LineTraceControl.h"
#include "Control.h"

static S16 angle;

// 数値が安定しないので、積算して絶対角度を測定するのは難しい。
// 角速度を測定するが、ばらつきが大きいので平均化する。

#define ANGLE_COUNT 20 // 平均回数 …80ms
#define ANGLE_CYCLE (CONTROL_CYCLE*ANGLE_COUNT)
static S16 gyroBuf[ANGLE_COUNT];
static U8 idxGyroBuf;

//*****************************************************************************
// 関数名 : AngleMeter
// 引数 : なし
// 返り値 : 現在のロボットの角速度（level/sec）
// 概要 : 角度計として用いる、必ず制御周期(CONTROL_CYCLE)で呼び出すこと
//*****************************************************************************
S32 AngleMeter(void)
{
	U16 gyro = ecrobot_get_gyro_sensor(NXT_PORT_S1);
	gyroBuf[idxGyroBuf++] = gyro - GYRO_OFFSET;
	if(idxGyroBuf >= ANGLE_COUNT) idxGyroBuf = 0;

	angle = 0;
	for(int i=0;i<ANGLE_COUNT;i++) angle += gyroBuf[i];
	angle = angle / (1000 / ANGLE_CYCLE); // level/sec

	// ecrobot_debug1(angle, gyro, GYRO_OFFSET);

	return angle;
}

//*****************************************************************************
// 関数名 : initAngleMeter
// 引数 : freq=周期(ms)(≠0)
// 返り値 : なし
// 概要 : 周期のセット、前回値の初期化
//*****************************************************************************
void initAngleMeter()
{
	angle = 0;
	for(int i=0;i<ANGLE_COUNT;i++) gyroBuf[ANGLE_COUNT] = 0;
	idxGyroBuf = 0;
	return;
}
