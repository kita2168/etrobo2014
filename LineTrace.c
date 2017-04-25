/* 2012/07/21 created by s.sahara 難所クリア単体テスト用 */

#include "LineTrace.h"
#include "LineTraceParam.h"

#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

#include "CourseDef.h"

#include "Calc.h"
#include "SensorIn.h"
#include "Control.h"

#include "LineTraceControl.h"

#include "CourseDef.h"

#ifdef MAKE_INSIDE
#include "LineTraceInsideBasicSmc.h"
#include "LineTraceInsideBonusSmc.h"
#else // #ifdef MAKE_INSIDE
#include "LineTraceOutsideBasicSmc.h"
#include "LineTraceOutsideBonusSmc.h"
#endif // #ifdef MAKE_INSIDE
#include "LineTraceTestSmc.h"


// 2012/08/13 add by s.sahara >>>>
void LineTrace_Init()
{
	//InitTurnDetect();

	#ifdef MAKE_INSIDE
	LineTraceInsideBasicSmc_Init(0);
	#else // MAKE_INSIDE
	LineTraceOutsideBasicSmc_Init(0);
	#endif // MAKE_INSIDE

	#ifdef USE_SAMPLE_COURCE
	LineTraceTestSmc_Init(0);
	#endif // #ifdef USE_SAMPLE_COURCE
}
// 2012/08/13 add by s.sahara <<<<

int LineTraceMain(int courseSide, int part, float distanceAbs, int* courseJointDetected)
{
#ifdef MAKE_INSIDE
	#ifndef TEST_BONUS
	static UINT seqNo_In = 0;
	static UINT seqNo_In1 = 0;
	static UINT seqNo_In2 = 0;
	#else
	static UINT seqNo_In = 0;
	static UINT seqNo_In1 = LT_IN1_SEQ_00;
	static UINT seqNo_In2 = 0;
	static UINT distanceInit = 0;
	#endif
#else // #ifdef MAKE_INSIDE
	#ifndef TEST_BONUS
	static UINT seqNo_Out = 0;
	static UINT seqNo_Out1 = 0;
	static UINT seqNo_Out2 = 0;
	#else
	static UINT seqNo_Out = 0;
	static UINT seqNo_Out1 = LT_OUT1_SEQ_00;
	static UINT seqNo_Out2 = 0;
	static UINT distanceInit = 0;
	#endif
#endif // #ifdef MAKE_INSIDE

	// 単体テスト用
	#ifdef USE_SAMPLE_COURCE
	#ifdef TEST_SLOW_FORWARD_RUN
	seqNo_In1 = 100;
	#endif
	#ifdef ADJUST_LT_PID
	seqNo_In1 = 200;
	#endif
	#endif

	static float distanceCorrection = 0.0f;

#ifdef MAKE_INSIDE
	float distanceAbsAtIn2Start = IN1_NORMAL_DISTANCE; // IN後半開始時距離
#else // #ifdef MAKE_INSIDE
	float distanceAbsAtOut2Start = OUT1_NORMAL_DISTANCE; // OUT後半開始時距離
#endif // #ifdef MAKE_INSIDE
	
	signed char forward;

	#ifdef TEST_BONUS
	// 途中から開始時の距離がゼロなので、距離補正値を初期化
	if(!distanceInit)
	{
#ifdef MAKE_INSIDE
		if(COURSE_IN_SIDE == courseSide)
		{
			distanceCorrection += IN1_NORMAL_DISTANCE - 0.5f;
		}
#else
		if(COURSE_OUT_SIDE == courseSide)
		{
			distanceCorrection += OUT1_NORMAL_DISTANCE - 0.5f;
		}
#endif
		distanceInit = 1;
	}
	#endif
	
	distanceAbs = distanceAbs + distanceCorrection; // 距離補正

	if (sonar_alert(20) == 1) /* 障害物検知 */
	{
		tail_control(TAIL_ANGLE_DRIVE); /* バランス走行用角度に制御 */
		nxt_motor_set_speed(NXT_PORT_C, (S8)0, 1);
		nxt_motor_set_speed(NXT_PORT_B, (S8)0, 1);
	}

	#ifndef USE_SAMPLE_COURCE
	#ifdef MAKE_INSIDE
	if(COURSE_IN_SIDE == courseSide)
	{
		switch(seqNo_In)
		{
		case 0:
			// コース前半 走行シーケンス
			seqNo_In1 = LineTraceInsideBasicSmc_Smc(seqNo_In1, distanceAbs, courseJointDetected, &distanceCorrection);
			if(IN_COURSE_PART_BEFORE_FIGUREL == *courseJointDetected)
			{
				forward = LineTraceInsideBasicSmc_GetForward();

				LineTraceInsideBonusSmc_Init(forward);
				seqNo_In++;
			}
			break;

		case 1:
			// コース後半 走行シーケンス
			seqNo_In2 = LineTraceInsideBonusSmc_Smc(seqNo_In2, distanceAbs - distanceAbsAtIn2Start, courseJointDetected, &distanceCorrection);
			break;
		}
	}

	#else // #ifdef MAKE_INSIDE
	if(COURSE_OUT_SIDE == courseSide)
	{
		switch(seqNo_Out)
		{
		case 0:
			// コース前半 走行シーケンス
			seqNo_Out1 = LineTraceOutsideBasicSmc_Smc(seqNo_Out1, distanceAbs, courseJointDetected, &distanceCorrection);
			if(OUT_COURSE_PART_BEFORE_LOOKUPGATE== *courseJointDetected)
			{
				forward = LineTraceOutsideBasicSmc_GetForward();

				LineTraceOutsideBonusSmc_Init(forward);
				seqNo_Out++;
			}
			break;

		case 1:
			// コース後半 走行シーケンス
			seqNo_Out2 = LineTraceOutsideBonusSmc_Smc(seqNo_Out2, distanceAbs - distanceAbsAtOut2Start, courseJointDetected, &distanceCorrection);
			break;
		}
	}
	#endif // #ifdef MAKE_INSIDE

	#else // #ifndef USE_SAMPLE_COURCE
	seqNo_In1 = LineTraceTestSmc_Smc(seqNo_In1, distanceAbs, courseJointDetected, &distanceCorrection);
	#endif // #ifndef USE_SAMPLE_COURCE

	return 0;
}

//*****************************************************************************
// 関数名 : sonar_alert
// 引数 : 無し
// 返り値 : 1(障害物あり)/0(障害物無し)
// 概要 : 超音波センサによる障害物検知
//*****************************************************************************
int sonar_alert(int alert_distance)
{
	static unsigned int counter = 0;
	static int alert = 0;

	signed int distance;

	if (++counter == 40/4) /* 約40msec周期毎に障害物検知  */
	{
		/*
		 * 超音波センサによる距離測定周期は、超音波の減衰特性に依存します。
		 * NXTの場合は、40msec周期程度が経験上の最短測定周期です。
		 */
		distance = ecrobot_get_sonar_sensor(NXT_PORT_S2);
		if ((distance <= alert_distance) && (distance >= 0))
		{
			alert = 1; /* 障害物を検知 */
		}
		else
		{
			alert = 0; /* 障害物無し */
		}
		counter = 0;
	}

	return alert;
}


/* 
 * 超信地旋回
 * その場で方向転換を行う
 * @param	forward(基本0固定)
 * @param	turn(基本0固定)
 * @param	rotate_power			回転速度に関係
 * @param	rotate_LR	0:左,1:右	回転方向に関係
 */
void BalanceControlTurn(char forward, char turn, char rotate_power, char rotate_LR)
{
	signed char pwm_L, pwm_R; /* 左右モータPWM出力 */
	signed char add_pwm_L, add_pwm_R;
	
	if(0 == rotate_LR)
	{
		add_pwm_L = (-1) * rotate_power;
		add_pwm_R = rotate_power;
	}
	else
	{
		add_pwm_L = rotate_power;
		add_pwm_R = (-1) * rotate_power;
	}

	/* 倒立振子制御(forward = 0, turn = 0で静止バランス) */
	balance_control(
		(float)forward,								 /* 前後進命令(+:前進, -:後進) */
		(float)turn,								 /* 旋回命令(+:右旋回, -:左旋回) */
		(float)ecrobot_get_gyro_sensor(NXT_PORT_S1), /* ジャイロセンサ値 */
		(float)GYRO_OFFSET,							 /* ジャイロセンサオフセット値 */
		(float)nxt_motor_get_count(NXT_PORT_C),		 /* 左モータ回転角度[deg] */
		(float)nxt_motor_get_count(NXT_PORT_B),		 /* 右モータ回転角度[deg] */
		(float)ecrobot_get_battery_voltage(),		 /* バッテリ電圧[mV] */
		&pwm_L,										 /* 左モータPWM出力値 */
		&pwm_R);									 /* 右モータPWM出力値 */
	nxt_motor_set_speed(NXT_PORT_C, (pwm_L + add_pwm_L), 1); /* 左モータPWM出力セット(-100～100) */
	nxt_motor_set_speed(NXT_PORT_B, (pwm_R + add_pwm_R), 1); /* 右モータPWM出力セット(-100～100) */
}
