/* 2013/08/27 created by t.akiyama 倒立から尻尾走行移行 */

#include "BalanceToTail.h"

#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

#include "LineTrace.h"
#include "LineTraceParam.h"
#include "Calc.h"
#include "Control.h"
#include "SensorIn.h"
#include "Timer.h"
#include "SpeedoMeter.h"

/* シーケンス番号定義 */
#define BTOT_SEQ_NO_00 0
#define BTOT_SEQ_NO_01 1
#define BTOT_SEQ_NO_02 2
#define BTOT_SEQ_NO_03 3

/* 前後進命令 */
static signed char forward;
/* 旋回命令 */
static signed char turn;
/* シーケンス番号 */
static signed char btotSeqNo = 0;
/* 周期カウンタ */
static int count = 0;

/* 尻尾角度 */
signed int angle;
static signed char angleTarget = 0;
static signed char angleTargetPrev = 0;
static float angleAccelT = 0.0f;
static float angleAccelTmax = 0;

/* 前進値 */
static signed char forwardTarget = 0;		// 目標速度指令
static signed char forwardTargetPrev = 0;	// 前回目標速度指令
static float accelT = 0.0f;
static float accelTmax = 0;

void BalanceToTail_ChangeAngle(float ang, float Tmax)
{
	angleTargetPrev = nxt_motor_get_count(NXT_PORT_A);
	angleTarget = ang;
	angleAccelT = 0.0f;
	angleAccelTmax = Tmax;
}

void FigureLStageStm_ChangeSpeed(float spd, float Tmax)
{
	forwardTargetPrev = forward;
	forwardTarget = spd;
	accelT = 0.0f;
	accelTmax = Tmax;
}

void BalanceToTail_Init()
{
	forward = 0;
	turn = 0;
	btotSeqNo = 0;
	count = 0;
}

int BalanceToTailMain()
{
	switch(btotSeqNo)
	{
		/* 倒立待機 */
		case BTOT_SEQ_NO_00:
	
			
			if(count == 0)
			{
				InitBalanceControlStraightP();
			}
			
			if(TIMER_1S < count)
			{
				btotSeqNo++;
				count = 0;
				
				BalanceToTail_ChangeAngle(TAIL_ANGLE_TAILRUN, 1.0f);
			}
			else
			{
				/* 尻尾は邪魔にならない位置に上げておく */
				tail_pid_control(TAIL_KP, TAIL_KI, TAIL_KD, TAIL_ANGLE_BALANCE_RUN, TAIL_MAX_MV, TAIL_MIN_MV);
				forward = 0;
			
				count++;
			}
			
			break;

 		/* 倒立走行→尻尾走行に移行する前準備 */
		case BTOT_SEQ_NO_01:
			
			// 尻尾走行の角度に尻尾を下げる
			angle = (signed char)SCurve((float)angleTarget, (float)angleTargetPrev, angleAccelTmax, &angleAccelT);
			tail_pid_control(TAIL_KP, TAIL_KI, TAIL_KD, angle, TAIL_MAX_MV, TAIL_MIN_MV);
				
			if(TIMER_2S < count)
			{
				btotSeqNo++;
				count = 0;
			}
			else
			{
				forward = 0;
				turn = 0;
				count++;
				
			}
			
			break;
	
		/* 尻尾走行へ遷移 */
		case BTOT_SEQ_NO_02:
				
			tail_pid_control(TAIL_KP, TAIL_KI, TAIL_KD, TAIL_ANGLE_TAILRUN - 5, TAIL_MAX_MV, TAIL_MIN_MV);
		
			if(TIMER_1S / 2 < count)
			{
				btotSeqNo++;
				count = 0;
			}
			else
			{
//				forward = 30;
				forward = 25;
				turn = 0;
				count++;
			}
			
			break;
	
		/* 安定待ち→メインに処理を返す */
		case BTOT_SEQ_NO_03:
			
			tail_pid_control(TAIL_KP, TAIL_KI, TAIL_KD, TAIL_ANGLE_TAILRUN, TAIL_MAX_MV, TAIL_MIN_MV);
				
			if(TIMER_1S < count)
			{
				count = 0;
				return 1;
			}
			else
			{
				forward = 0;
				turn = 0;
				count++;
			}
				
			break;
	
	}
	
	ecrobot_debug1((UINT)btotSeqNo, 0, 0);
	
	/* とりあえず個別に記述 */
	if(btotSeqNo == BTOT_SEQ_NO_00)
	{
		BalanceControlStraightP(forward, 3.0f);
	}
	else if(btotSeqNo == BTOT_SEQ_NO_01)
	{
		BalanceControlStraightP(forward, 3.0f);
	}
	else if(btotSeqNo == BTOT_SEQ_NO_02)
	{
		TailRunControl(forward, turn);
	}
	else if(btotSeqNo == BTOT_SEQ_NO_03)
	{
		TailRunControl(forward, turn);
	}
	
	return 0;
}
