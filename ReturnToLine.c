/* 2013/08/27 created by t.akiyama ライン復帰処理（倒立） */
#include "ReturnToLine.h"

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
#define RTOL_SEQ_NO_00 0
#define RTOL_SEQ_NO_01 1
#define RTOL_SEQ_NO_02 2
#define RTOL_SEQ_NO_03 3
#define RTOL_SEQ_NO_04 4
#define RTOL_SEQ_NO_05 5
#define RTOL_SEQ_NO_06 6

#define DIRECTION_KP (2.0f)			/* 方向舵 比例ゲイン ※要実機調整 */
#define DIRECTION_KD (0.1f)			/* 方向舵 微分ゲイン ※要実機調整 */
#define DIRECTION_MV_MAX (50.0f)
#define DIRECTION_MV_MIN (-50.0f)


/* 前後進命令 */
static signed char forward;
/* 旋回命令 */
static signed char turn;
/* シーケンス番号 */
static signed char rtolSeqNo = 0;
/* 周期カウンタ */
static int count = 0;

void ReturnToLine_Init()
{
	forward = 0;
	turn = 0;
	rtolSeqNo = 0;
	count = 0;
}

int ReturnToLineMain()
{
	/* ライントレース用 */
	U16 lightValue;
	float command;
	float mv;
	
	/* 走行距離 */
	float distance = 0;
	
	/* キャリブレーション値 */
	U16 black = LineTraceControl_LightBlackThreshold();
	U16 white = LineTraceControl_LightWhiteThreshold();
	
	switch(rtolSeqNo)
	{
		/* 倒立待機 */
		case RTOL_SEQ_NO_00:
			
			/* 尻尾は邪魔にならない位置に上げておく */
			tail_pid_control(TAIL_KP, TAIL_KI, TAIL_KD, TAIL_ANGLE_BALANCE_RUN, TAIL_MAX_MV, TAIL_MIN_MV);
			
			if(count == 0)
			{
				InitBalanceControlStraightP();
			}
			
			if(TIMER_1S < count)
			{
				rtolSeqNo++;
				count = 0;
			}
			else
			{
				forward = 0;
				count++;
			}
			
			break;

		/* ライン復帰するため一旦左側に回転 */
		case RTOL_SEQ_NO_01:
			
			tail_pid_control(TAIL_KP, TAIL_KI, TAIL_KD, TAIL_ANGLE_BALANCE_RUN, TAIL_MAX_MV, TAIL_MIN_MV);
			
			/* TODO 回転速度は要調整 */
			forward = 0;
			turn = -30;
				
			if(count == 0)
			{
				/* 回転計測初期化 */
				init_rotateDistance(0);	/* 左回転 */
				count++;
			}
			
			if(check_rotateComplete(25))
			{
				rtolSeqNo++;
				count = 0;
			}
			
			break;

		/* 一定距離直進 */
		case RTOL_SEQ_NO_02:
		
			tail_pid_control(TAIL_KP, TAIL_KI, TAIL_KD, TAIL_ANGLE_BALANCE_RUN, TAIL_MAX_MV, TAIL_MIN_MV);
			
			/* TODO 速度は要調整 */
			forward = 10;
			turn = 0;
			
			if(count == 0)
			{
				/* 距離計測初期化 */
				init_DistanceFromPoint();
				InitBalanceControlStraightP();
				count++;
			}
			
			/* 走行距離取得 */
			distance = calc_getDistanceFromPoint();
			
			/* TODO 走行距離は要調整 */
			if(0.09f < distance)
			{
				rtolSeqNo++;
				forward = 0;
				count = 0;
			}
			
			break;

		/* 右に回転してライン復帰方向へ旋回 */
		case RTOL_SEQ_NO_03:
				
			tail_pid_control(TAIL_KP, TAIL_KI, TAIL_KD, TAIL_ANGLE_BALANCE_RUN, TAIL_MAX_MV, TAIL_MIN_MV);
				
			/* TODO 回転速度は要調整 */
			forward = 0;
			turn = 20;
				
			if(count == 0)
			{
				init_rotateDistance(1);	/* 右回転 */
				count++;
			}
			
			if(check_rotateComplete(60))	/* 60度左回転 */
			{
				rtolSeqNo++;
				count = 0;
			}
			
			break;

		/* ライン検知するまで直進 */
		case RTOL_SEQ_NO_04:
				
			tail_pid_control(TAIL_KP, TAIL_KI, TAIL_KD, TAIL_ANGLE_BALANCE_RUN, TAIL_MAX_MV, TAIL_MIN_MV);
				
			/* TODO 速度は要調整 */
			forward = 10;
			turn = 0;
				
			if(count == 0)
			{
				InitBalanceControlStraightP();
				count++;
			}
			
			if(ecrobot_get_light_sensor(NXT_PORT_S3) > black)
			{
				rtolSeqNo++;
				forward = 0;
				count = 0;
			}
			
			break;
			
		/* ライントレース再開 */
		case RTOL_SEQ_NO_05:
		
			tail_pid_control(TAIL_KP, TAIL_KI, TAIL_KD, TAIL_ANGLE_BALANCE_RUN, TAIL_MAX_MV, TAIL_MIN_MV);
			
			/* TODO 要調整 */
			if(TIMER_2S < count)
			{
				rtolSeqNo++;
				count = 0;
			}
			else
			{
				forward = 10;
				
				/* TODO PID制御にするか要検討 */
				lightValue = ecrobot_get_light_sensor(NXT_PORT_S3); /* 0-1023 */
				command = (white + black)/2.0f; /* 白と黒の中間 */
				mv = ControlP(DIRECTION_KP, command, (float)lightValue, DIRECTION_MV_MAX, DIRECTION_MV_MIN);
				turn = (signed char)mv;
				count++;
			}
			
			break;

		/* ライントレースへ処理を戻す */
		case RTOL_SEQ_NO_06:
		
			tail_pid_control(TAIL_KP, TAIL_KI, TAIL_KD, TAIL_ANGLE_BALANCE_RUN, TAIL_MAX_MV, TAIL_MIN_MV);
			
			forward = 0;
			turn = 0;
			
			if(TIMER_1S < count)
			{
				return 1;
				count = 0;
			}
			else
			{
				count++;
			}

			break;
	}

	ecrobot_debug1((UINT)rtolSeqNo, 0, 0);
	
	if(rtolSeqNo == RTOL_SEQ_NO_02 || rtolSeqNo == RTOL_SEQ_NO_04)
	{
		BalanceControlStraightP(forward, 3.0f);
	}
	else
	{
		BalanceControl(forward, turn);
	}

	return 0;
}
