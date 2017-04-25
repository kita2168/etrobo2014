/* 2013/08/27 created by t.akiyama ���C�����A�����i�|���j */
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

/* �V�[�P���X�ԍ���` */
#define RTOL_SEQ_NO_00 0
#define RTOL_SEQ_NO_01 1
#define RTOL_SEQ_NO_02 2
#define RTOL_SEQ_NO_03 3
#define RTOL_SEQ_NO_04 4
#define RTOL_SEQ_NO_05 5
#define RTOL_SEQ_NO_06 6

#define DIRECTION_KP (2.0f)			/* ������ ���Q�C�� ���v���@���� */
#define DIRECTION_KD (0.1f)			/* ������ �����Q�C�� ���v���@���� */
#define DIRECTION_MV_MAX (50.0f)
#define DIRECTION_MV_MIN (-50.0f)


/* �O��i���� */
static signed char forward;
/* ���񖽗� */
static signed char turn;
/* �V�[�P���X�ԍ� */
static signed char rtolSeqNo = 0;
/* �����J�E���^ */
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
	/* ���C���g���[�X�p */
	U16 lightValue;
	float command;
	float mv;
	
	/* ���s���� */
	float distance = 0;
	
	/* �L�����u���[�V�����l */
	U16 black = LineTraceControl_LightBlackThreshold();
	U16 white = LineTraceControl_LightWhiteThreshold();
	
	switch(rtolSeqNo)
	{
		/* �|���ҋ@ */
		case RTOL_SEQ_NO_00:
			
			/* �K���͎ז��ɂȂ�Ȃ��ʒu�ɏグ�Ă��� */
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

		/* ���C�����A���邽�߈�U�����ɉ�] */
		case RTOL_SEQ_NO_01:
			
			tail_pid_control(TAIL_KP, TAIL_KI, TAIL_KD, TAIL_ANGLE_BALANCE_RUN, TAIL_MAX_MV, TAIL_MIN_MV);
			
			/* TODO ��]���x�͗v���� */
			forward = 0;
			turn = -30;
				
			if(count == 0)
			{
				/* ��]�v�������� */
				init_rotateDistance(0);	/* ����] */
				count++;
			}
			
			if(check_rotateComplete(25))
			{
				rtolSeqNo++;
				count = 0;
			}
			
			break;

		/* ��苗�����i */
		case RTOL_SEQ_NO_02:
		
			tail_pid_control(TAIL_KP, TAIL_KI, TAIL_KD, TAIL_ANGLE_BALANCE_RUN, TAIL_MAX_MV, TAIL_MIN_MV);
			
			/* TODO ���x�͗v���� */
			forward = 10;
			turn = 0;
			
			if(count == 0)
			{
				/* �����v�������� */
				init_DistanceFromPoint();
				InitBalanceControlStraightP();
				count++;
			}
			
			/* ���s�����擾 */
			distance = calc_getDistanceFromPoint();
			
			/* TODO ���s�����͗v���� */
			if(0.09f < distance)
			{
				rtolSeqNo++;
				forward = 0;
				count = 0;
			}
			
			break;

		/* �E�ɉ�]���ă��C�����A�����֐��� */
		case RTOL_SEQ_NO_03:
				
			tail_pid_control(TAIL_KP, TAIL_KI, TAIL_KD, TAIL_ANGLE_BALANCE_RUN, TAIL_MAX_MV, TAIL_MIN_MV);
				
			/* TODO ��]���x�͗v���� */
			forward = 0;
			turn = 20;
				
			if(count == 0)
			{
				init_rotateDistance(1);	/* �E��] */
				count++;
			}
			
			if(check_rotateComplete(60))	/* 60�x����] */
			{
				rtolSeqNo++;
				count = 0;
			}
			
			break;

		/* ���C�����m����܂Œ��i */
		case RTOL_SEQ_NO_04:
				
			tail_pid_control(TAIL_KP, TAIL_KI, TAIL_KD, TAIL_ANGLE_BALANCE_RUN, TAIL_MAX_MV, TAIL_MIN_MV);
				
			/* TODO ���x�͗v���� */
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
			
		/* ���C���g���[�X�ĊJ */
		case RTOL_SEQ_NO_05:
		
			tail_pid_control(TAIL_KP, TAIL_KI, TAIL_KD, TAIL_ANGLE_BALANCE_RUN, TAIL_MAX_MV, TAIL_MIN_MV);
			
			/* TODO �v���� */
			if(TIMER_2S < count)
			{
				rtolSeqNo++;
				count = 0;
			}
			else
			{
				forward = 10;
				
				/* TODO PID����ɂ��邩�v���� */
				lightValue = ecrobot_get_light_sensor(NXT_PORT_S3); /* 0-1023 */
				command = (white + black)/2.0f; /* ���ƍ��̒��� */
				mv = ControlP(DIRECTION_KP, command, (float)lightValue, DIRECTION_MV_MAX, DIRECTION_MV_MIN);
				turn = (signed char)mv;
				count++;
			}
			
			break;

		/* ���C���g���[�X�֏�����߂� */
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
