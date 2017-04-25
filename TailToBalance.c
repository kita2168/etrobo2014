/* 2013/08/27 created by t.akiyama 尻尾走行から倒立移行 */

#include "TailToBalance.h"

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
#define TTOB_SEQ_NO_00 0
#define TTOB_SEQ_NO_01 1
#define TTOB_SEQ_NO_02 2
#define TTOB_SEQ_NO_03 3

/* 前後進命令 */
static signed char forward;
/* 旋回命令 */
static signed char turn;
/* シーケンス番号 */
static signed char ttobSeqNo = 0;
/* 周期カウンタ */
static int count = 0;

int TailToBalanceMain()
{
	static signed int angle_variable;
	
	switch(ttobSeqNo)
	{
		/* 尻尾待機 */
		case TTOB_SEQ_NO_00:
		
			tail_pid_control(TAIL_KP, TAIL_KI, TAIL_KD, TAIL_ANGLE_TAILRUN, TAIL_MAX_MV, TAIL_MIN_MV);
			
			forward = 0;
			turn = 0;
			
			if(count == 0)
			{
				InitTailRunControlStraightP();
				angle_variable = 0.0;
			}

			/* 1秒間待機 */
			if(TIMER_1S > count)
			{
				count++;
			}
			else
			{
				ttobSeqNo++;
				count = 0;
			}
			
			break;

		/* 尻尾走行→倒立へ遷移 */
		case TTOB_SEQ_NO_01:

			forward = 0;
			turn = 0;

			if(TIMER_2S > count)
			{
				if(count == 0)
				{
					//モータ速度をリセット
					nxt_motor_set_speed(NXT_PORT_B, 0, 1); /* 右モータPWM出力セット(-100～100) */
					nxt_motor_set_speed(NXT_PORT_C, 0, 1); /* 左モータPWM出力セット(-100～100) */
			
					angle_variable = TAIL_ANGLE_TAILRUN + 2;
				}
				/* 時間経過毎に尻尾角度を変化させる */
				if(count == 100 || count == 200 || count == 300 || count == 400)
				{
					angle_variable++;
					angle_variable++;
				}
				/* 倒立状態の角度に制御 */
				tail_pid_control(TAIL_KP, TAIL_KI, TAIL_KD, angle_variable, TAIL_MAX_MV, TAIL_MIN_MV);
				count++;
			}
			else
			{
				ttobSeqNo++;
				count = 0;
			}
			
			break;

		/* 倒立維持→メイン処理に返す */
		case TTOB_SEQ_NO_02:

			tail_pid_control(TAIL_KP, TAIL_KI, TAIL_KD, TAIL_ANGLE_BALANCE_RUN, TAIL_MAX_MV, TAIL_MIN_MV);

			forward = 0;
			turn = 0;
				
			if(count == 0)
			{
				//倒立前にモータ角度をリセット
				nxt_motor_set_count(NXT_PORT_B, 0);
				nxt_motor_set_count(NXT_PORT_C, 0);

				init_DistanceFromPoint(); //位置判断用

				InitBalanceControlStraightP();
				balance_init();
			}
			
			/* 1秒間待機 */
			if(TIMER_1S > count)
			{
				count++;
			}
			else
			{
				return 1;
			}
			
			break;
	}
	
	ecrobot_debug1((UINT)ttobSeqNo, 0, 0);
	
	/* とりあえず個別に記述 */
	if(ttobSeqNo == TTOB_SEQ_NO_00)
	{
		TailRunControl(forward, turn);
	}
	else if(ttobSeqNo == TTOB_SEQ_NO_01)
	{
		TailRunControl(forward, turn);
	}
	else if(ttobSeqNo == TTOB_SEQ_NO_02)
	{
		BalanceControlStraightP(forward, 3.0f);
	}
	
	return 0;
}
