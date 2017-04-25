/* 2013/08/27 created by t.akiyama �K�����s����|���ڍs */

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

/* �V�[�P���X�ԍ���` */
#define TTOB_SEQ_NO_00 0
#define TTOB_SEQ_NO_01 1
#define TTOB_SEQ_NO_02 2
#define TTOB_SEQ_NO_03 3

/* �O��i���� */
static signed char forward;
/* ���񖽗� */
static signed char turn;
/* �V�[�P���X�ԍ� */
static signed char ttobSeqNo = 0;
/* �����J�E���^ */
static int count = 0;

int TailToBalanceMain()
{
	static signed int angle_variable;
	
	switch(ttobSeqNo)
	{
		/* �K���ҋ@ */
		case TTOB_SEQ_NO_00:
		
			tail_pid_control(TAIL_KP, TAIL_KI, TAIL_KD, TAIL_ANGLE_TAILRUN, TAIL_MAX_MV, TAIL_MIN_MV);
			
			forward = 0;
			turn = 0;
			
			if(count == 0)
			{
				InitTailRunControlStraightP();
				angle_variable = 0.0;
			}

			/* 1�b�ԑҋ@ */
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

		/* �K�����s���|���֑J�� */
		case TTOB_SEQ_NO_01:

			forward = 0;
			turn = 0;

			if(TIMER_2S > count)
			{
				if(count == 0)
				{
					//���[�^���x�����Z�b�g
					nxt_motor_set_speed(NXT_PORT_B, 0, 1); /* �E���[�^PWM�o�̓Z�b�g(-100�`100) */
					nxt_motor_set_speed(NXT_PORT_C, 0, 1); /* �����[�^PWM�o�̓Z�b�g(-100�`100) */
			
					angle_variable = TAIL_ANGLE_TAILRUN + 2;
				}
				/* ���Ԍo�ߖ��ɐK���p�x��ω������� */
				if(count == 100 || count == 200 || count == 300 || count == 400)
				{
					angle_variable++;
					angle_variable++;
				}
				/* �|����Ԃ̊p�x�ɐ��� */
				tail_pid_control(TAIL_KP, TAIL_KI, TAIL_KD, angle_variable, TAIL_MAX_MV, TAIL_MIN_MV);
				count++;
			}
			else
			{
				ttobSeqNo++;
				count = 0;
			}
			
			break;

		/* �|���ێ������C�������ɕԂ� */
		case TTOB_SEQ_NO_02:

			tail_pid_control(TAIL_KP, TAIL_KI, TAIL_KD, TAIL_ANGLE_BALANCE_RUN, TAIL_MAX_MV, TAIL_MIN_MV);

			forward = 0;
			turn = 0;
				
			if(count == 0)
			{
				//�|���O�Ƀ��[�^�p�x�����Z�b�g
				nxt_motor_set_count(NXT_PORT_B, 0);
				nxt_motor_set_count(NXT_PORT_C, 0);

				init_DistanceFromPoint(); //�ʒu���f�p

				InitBalanceControlStraightP();
				balance_init();
			}
			
			/* 1�b�ԑҋ@ */
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
	
	/* �Ƃ肠�����ʂɋL�q */
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
