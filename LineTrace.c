/* 2012/07/21 created by s.sahara ��N���A�P�̃e�X�g�p */

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

	// �P�̃e�X�g�p
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
	float distanceAbsAtIn2Start = IN1_NORMAL_DISTANCE; // IN�㔼�J�n������
#else // #ifdef MAKE_INSIDE
	float distanceAbsAtOut2Start = OUT1_NORMAL_DISTANCE; // OUT�㔼�J�n������
#endif // #ifdef MAKE_INSIDE
	
	signed char forward;

	#ifdef TEST_BONUS
	// �r������J�n���̋������[���Ȃ̂ŁA�����␳�l��������
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
	
	distanceAbs = distanceAbs + distanceCorrection; // �����␳

	if (sonar_alert(20) == 1) /* ��Q�����m */
	{
		tail_control(TAIL_ANGLE_DRIVE); /* �o�����X���s�p�p�x�ɐ��� */
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
			// �R�[�X�O�� ���s�V�[�P���X
			seqNo_In1 = LineTraceInsideBasicSmc_Smc(seqNo_In1, distanceAbs, courseJointDetected, &distanceCorrection);
			if(IN_COURSE_PART_BEFORE_FIGUREL == *courseJointDetected)
			{
				forward = LineTraceInsideBasicSmc_GetForward();

				LineTraceInsideBonusSmc_Init(forward);
				seqNo_In++;
			}
			break;

		case 1:
			// �R�[�X�㔼 ���s�V�[�P���X
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
			// �R�[�X�O�� ���s�V�[�P���X
			seqNo_Out1 = LineTraceOutsideBasicSmc_Smc(seqNo_Out1, distanceAbs, courseJointDetected, &distanceCorrection);
			if(OUT_COURSE_PART_BEFORE_LOOKUPGATE== *courseJointDetected)
			{
				forward = LineTraceOutsideBasicSmc_GetForward();

				LineTraceOutsideBonusSmc_Init(forward);
				seqNo_Out++;
			}
			break;

		case 1:
			// �R�[�X�㔼 ���s�V�[�P���X
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
// �֐��� : sonar_alert
// ���� : ����
// �Ԃ�l : 1(��Q������)/0(��Q������)
// �T�v : �����g�Z���T�ɂ���Q�����m
//*****************************************************************************
int sonar_alert(int alert_distance)
{
	static unsigned int counter = 0;
	static int alert = 0;

	signed int distance;

	if (++counter == 40/4) /* ��40msec�������ɏ�Q�����m  */
	{
		/*
		 * �����g�Z���T�ɂ�鋗����������́A�����g�̌��������Ɉˑ����܂��B
		 * NXT�̏ꍇ�́A40msec�������x���o����̍ŒZ��������ł��B
		 */
		distance = ecrobot_get_sonar_sensor(NXT_PORT_S2);
		if ((distance <= alert_distance) && (distance >= 0))
		{
			alert = 1; /* ��Q�������m */
		}
		else
		{
			alert = 0; /* ��Q������ */
		}
		counter = 0;
	}

	return alert;
}


/* 
 * ���M�n����
 * ���̏�ŕ����]�����s��
 * @param	forward(��{0�Œ�)
 * @param	turn(��{0�Œ�)
 * @param	rotate_power			��]���x�Ɋ֌W
 * @param	rotate_LR	0:��,1:�E	��]�����Ɋ֌W
 */
void BalanceControlTurn(char forward, char turn, char rotate_power, char rotate_LR)
{
	signed char pwm_L, pwm_R; /* ���E���[�^PWM�o�� */
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

	/* �|���U�q����(forward = 0, turn = 0�ŐÎ~�o�����X) */
	balance_control(
		(float)forward,								 /* �O��i����(+:�O�i, -:��i) */
		(float)turn,								 /* ���񖽗�(+:�E����, -:������) */
		(float)ecrobot_get_gyro_sensor(NXT_PORT_S1), /* �W���C���Z���T�l */
		(float)GYRO_OFFSET,							 /* �W���C���Z���T�I�t�Z�b�g�l */
		(float)nxt_motor_get_count(NXT_PORT_C),		 /* �����[�^��]�p�x[deg] */
		(float)nxt_motor_get_count(NXT_PORT_B),		 /* �E���[�^��]�p�x[deg] */
		(float)ecrobot_get_battery_voltage(),		 /* �o�b�e���d��[mV] */
		&pwm_L,										 /* �����[�^PWM�o�͒l */
		&pwm_R);									 /* �E���[�^PWM�o�͒l */
	nxt_motor_set_speed(NXT_PORT_C, (pwm_L + add_pwm_L), 1); /* �����[�^PWM�o�̓Z�b�g(-100�`100) */
	nxt_motor_set_speed(NXT_PORT_B, (pwm_R + add_pwm_R), 1); /* �E���[�^PWM�o�̓Z�b�g(-100�`100) */
}
