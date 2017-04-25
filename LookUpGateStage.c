/* 2012/07/21 created by s.sahara ��N���A�P�̃e�X�g�p */
#include "LookUpGateStage.h"

#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

#include "SensorIn.h"
#include "Control.h"

#include "Calc.h"

#include "LineTrace.h"
#include "LineTraceParam.h"

#include "BalanceToTail.h"
#include "TailToBalance.h"

static signed char forward; 	 /* �O��i���� */
static signed char turn;		 /* ���񖽗� */

#if 1 // add by s.sahara
static signed char forwardTarget = 0;		// �ڕW���x�w��
static signed char forwardTargetPrev = 0;		// �O��ڕW���x�w��
static float accelT = 0.0f;
static float accelTmax = 0;

static signed char angleTarget = 0;
static signed char angleTargetPrev = 0;
static float angleAccelT = 0.0f;
static float angleAccelTmax = 0;
#endif

/* �V�[�P���X�ԍ� */
#define BALANCE_TO_TAIL_SEQ_NO	0	/* �K�����s�� */
#define BEFORE_GATE_SEQ_NO		1	/* ���b�N�A�b�v�Q�[�g���O */
#define BALANCE_OFF_SEQ_NO		2	/* ���s�̌X���� */
#define UNDER_GATE_SEQ_NO		3	/* �K�����s�� */
#define BRAKE_SEQ_NO			4	/* �Î~ */
#define UNDER_RETURN_SEQ_NO		5	/* ���^�[�����s */
#define BRAKE2_SEQ_NO			6	/* �Î~ */
#define UNDER_GATE2_SEQ_NO		7	/* �K�����s�Q */
#define BALANCE_ON_SEQ_NO		8	/* ���s�̕��A�� */
#define AFTER_GATE_SEQ_NO		9	/* �S�[���� */
#define END_GATE_SEQ_NO			10	/* �e�X�g�p */
static signed char SeqNo = BALANCE_TO_TAIL_SEQ_NO;


/* �W���C�����o */
#define STOP_THRESHOLD (0.05f) //��~�Ɣ��f����臒l
static int gyroInitflg = 0; // �W���C�����o�������t���O

/* �K�����s�ڍs�V�[�P���X */
static int countBalanceToTailSeq = 0;

/* ���b�N�A�b�v�Q�[�g���O�V�[�P���X */
#define INIT_SONAR_SENSOR 25 // �����g�Z���T�̍ŏ��𖳎�����
static int countBeforGateSeq = 0;
static int prevSonarAlertValue = -1;

/* ���s�̌X�����V�[�P���X */
#define ASSIST_BLANCE_OFF_COUNT 100 // �X���̃A�V�X�g(�X��1�i�K�ڂƓ����ɍs��)
#define ONE_BLANCE_OFF_COUNT 250 // �X��1�i�K��(�K����|����ԂƌX����Ԃ̒��Ԃɂ���)

#define TWO_BLANCE_OFF_COUNT 250 // �X��2�i�K��(�K�����X����Ԃɂ���)

#define THREE_BLANCE_OFF_COUNT 450 // �Î~�̌��o(GyroDetect()==0�Ői��)
static int countBalanceOffSeq = 0;
#define ASSIST_MOTOR_SPEED 30 // �X�����A�V�X�g����͂̑傫��

/* �K�����s�� */
#define UNDER_BAR_RUN_CM 25  //35
#define UNDER_BAR_RUN_RET_CM 30 //45
static int countUnderGateSeq = 0;
static int countReturnGateSeq = 0;

/* ���s�̕��A�� */
#define ASSIST_BLANCE_ON_COUNT 149 // ���A�̃A�V�X�g(���A1�i�K�ڂƓ����ɍs��)
#define ZERO_BALANCE_ON_COUNT 250 // ���΂炭�Î~
#define ONE_BLANCE_ON_COUNT 500 // ���A1�i�K��(�K����|����ԂƌX����Ԃ̒��Ԃɂ���)

/* �Q�[�g�ʉߌ� */
static int countAfterGateSeq = 0;

#define TWO_BLANCE_ON_COUNT 400 // ���A2�i�K��(�K�����X����Ԃɂ���)
#define THREE_BLANCE_ON_COUNT 550 // �Î~�̌��o(GyroDetect()==0�Ői��)
static int countBalanceOnSeq = 0;
#define ASSIST_STAND_MOTOR_SPEED (-30) // ���A���A�V�X�g����͂̑傫��

// ���s�̎��]�X�e�[�^�X
#define TURN_LEFT 0
#define TURN_RIGHT 1

/* �ʏ�K�����s����p�Q�C�� */
#define DIRECTION_KP_TEST 			(0.5f)

#define DIRECTION_MV_MAX 			(100.0f)
#define DIRECTION_MV_MIN 			(-100.0f)

/* �L�����u���[�V�������Z���T�l */
static U16 black;
static U16 white;

//static int isreturn = 0;

//#################################################################
// kuni���C���g���[�X�p�̊֐��̐錾 add by kunitake <2014/08/13>
//#################################################################
extern void init_model();
extern void ctrl_direction(float ref_vel);
extern signed char get_model_forward();
extern signed char get_model_turn();
extern float get_model_mu();
extern float get_model_y();
extern void change_model_black_to_gray();
extern void change_model_gray_to_black();
extern void change_model_black_to_tail();
extern void change_model_tail_to_black();
extern void reset_model_state();
extern U16 get_model_center();
extern U16 get_model_light();


void LookUpGateStageStm_ChangeSpeed(float spd, float Tmax)
{
	forwardTargetPrev = forward;
	forwardTarget = spd;
	accelT = 0.0f;
	accelTmax = Tmax;
}

void LookUpGateStageStm_ChangeAngle(float ang, float Tmax)
{
	angleTargetPrev = nxt_motor_get_count(NXT_PORT_A);
	angleTarget = ang;
	angleAccelT = 0.0f;
	angleAccelTmax = Tmax;
}

int LookUpGateStageMain()
{
	static char hensa0Initialized = 0;
	static float hensa0;
	static float integral;

	float dirKp = TAILRUN_DIRECTION_KP;
	float dirKi = TAILRUN_DIRECTION_KI;
	float dirKd = TAILRUN_DIRECTION_KD;
	float Kstr = 0.6f; // �������s���Q�C�������W��

	// �K��PID�Ƃ肠�����K��
	float tailKp = TAIL_KP;
	float tailKi = TAIL_KI;
	float tailKd = TAIL_KD;
	float tailMaxMv = 100.0f;
	float tailMinMv = -100.0f;

	U16 lightValue = GetLightInLPF();
	signed int angle;

	if(!hensa0Initialized)
	{
		// �O��΍��ێ��l�̏�����
		hensa0 = 0.0f;
		integral = 0.0f;
		hensa0Initialized = 1;
	}

	switch(SeqNo)
	{
		case BALANCE_TO_TAIL_SEQ_NO:

			if(BalanceToTailMain() == 1)
			{
				SeqNo = BEFORE_GATE_SEQ_NO;
			}

			break;
		case BEFORE_GATE_SEQ_NO:		//�Q�[�g�̌��o

			if(0 == countBeforGateSeq)
			{
				LookUpGateStageStm_ChangeAngle(GATE_TAIL_ANGLE_SONAR, 0.5f);
 				LookUpGateStageStm_ChangeSpeed(20, 1.0f);
 			}
			forward = (signed char)SCurve((float)forwardTarget, (float)forwardTargetPrev, accelTmax, &accelT);
			// ������PID
			ctrl_direction((float)forward);
			turn = get_model_turn();
			forward = get_model_forward();
			TailRunControl(forward, turn);

			angle = (signed char)SCurve((float)angleTarget, (float)angleTargetPrev, angleAccelTmax, &angleAccelT);
			tail_pid_control(tailKp, tailKi, tailKd, angle, tailMaxMv, tailMinMv);

			//�؂�ւ��i�Q�[�g�̌��o�j
			int SonarAlertValue = sonar_alert(12); // 12cm
			if (prevSonarAlertValue == -1)
			{
				prevSonarAlertValue = SonarAlertValue;
			}

			//�����g�Z���T��10cm�ȓ��ɂȂ�������𔻒�A�������ŏ���100ms�͖�������
			if (SonarAlertValue == 1 && prevSonarAlertValue == 0 && countBeforGateSeq == INIT_SONAR_SENSOR) // ��Q�����m
			{
				forward = turn = 0; // ��Q�������m�������~
				SeqNo = BALANCE_OFF_SEQ_NO;
				countBalanceOffSeq = 0;
			}

			if(countBeforGateSeq < INIT_SONAR_SENSOR)
			{
				countBeforGateSeq++;
			}

			prevSonarAlertValue = SonarAlertValue;

#ifdef USE_LOG
//			ecrobot_bt_data_logger((S8)(SeqNo * 10), SonarAlertValue); // BlueTooth�փ��O�����o��(�V�[�P���X�ԍ�*10, �����g�Z���T���o)
#endif
			break;

		case BALANCE_OFF_SEQ_NO:	//�Q�[�g��ʂ��p�x�ɑ��s�̂��X����
			if(0 == countBalanceOffSeq)
			{
				LookUpGateStageStm_ChangeAngle(GATE_TAIL_ANGLE_UNDER_GATE, 1.0f);
				LookUpGateStageStm_ChangeSpeed(0, 0.5f);
				turn = 0;
			}
			forward = (signed char)SCurve((float)forwardTarget, (float)forwardTargetPrev, accelTmax, &accelT);
			TailRunControl(forward, turn);
			angle = (signed char)SCurve((float)angleTarget, (float)angleTargetPrev, angleAccelTmax, &angleAccelT);
			tail_pid_control(tailKp, tailKi, tailKd, angle, tailMaxMv, tailMinMv);

			/* �Î~�̌��o */
			if(TWO_BLANCE_OFF_COUNT <= countBalanceOffSeq && countBalanceOffSeq < THREE_BLANCE_OFF_COUNT)
			{
				// �W���C�����o�̏�����
				if(gyroInitflg == 0)
				{
					InitGyroDetect(STOP_THRESHOLD);
					gyroInitflg = 1;
				}
				else
				{
					if(GyroDetect() == 0)
					{
						countBalanceOffSeq++;
					}
				}
			}
			else
			{
				countBalanceOffSeq++;
			}

			/* �Î~��Ԃň�莞�Ԃ��ĂΎ��̃V�[�P���X */
			if(countBalanceOffSeq == THREE_BLANCE_OFF_COUNT)
			{
				hensa0 = 0;
				integral = 0;
				InitTailRunControlStraightP();
				SeqNo = UNDER_GATE_SEQ_NO;
				countUnderGateSeq = 0;
//				change_model_black_to_tail();
//				reset_model_state();
			}
			break;

		case UNDER_GATE_SEQ_NO:		//�Q�[�g����O�i���s����


			forward = (signed char)SCurve((float)forwardTarget, (float)forwardTargetPrev, accelTmax, &accelT);
			// ������PID
//			ctrl_direction((float)forward);
//			turn = get_model_turn();
//			forward = get_model_forward();
//			TailRunControl(forward, turn);
			TailRunControlStraightP(forward, 10.0f);
			tail_pid_control(tailKp, tailKi, tailKd, GATE_TAIL_ANGLE_UNDER_GATE, tailMaxMv, tailMinMv);

			if(countUnderGateSeq == 0)
			{
				init_DistanceFromPoint();
				LookUpGateStageStm_ChangeSpeed(30, 0.5f);
			}
			// �p�x�ő��s���������𔻒f
			if(calc_getDistanceFromPoint() * 100.0f >= UNDER_BAR_RUN_CM)
			{
				//�؂�ւ��i��苗���𑖍s�����j
				SeqNo = BRAKE_SEQ_NO;
				countBalanceOffSeq = 0;
				gyroInitflg = 0;
			}

			countUnderGateSeq++;

#ifdef USE_LOG
//			ecrobot_bt_data_logger((S8)(SeqNo * 10), (S8)(countUnderGateSeq / 10));
#endif
			break;

		case BRAKE_SEQ_NO:		//�Î~����
			if(0 == countBalanceOffSeq)
			{
				LookUpGateStageStm_ChangeAngle(GATE_TAIL_ANGLE_UNDER_GATE, 1.0f);
				LookUpGateStageStm_ChangeSpeed(0, 0.5f);
				turn = 0;
			}
			forward = (signed char)SCurve((float)forwardTarget, (float)forwardTargetPrev, accelTmax, &accelT);
			TailRunControl(forward, turn);

			tail_pid_control(tailKp, tailKi, tailKd, GATE_TAIL_ANGLE_UNDER_GATE, tailMaxMv, tailMinMv);

			/* �Î~�̌��o */
			countBalanceOffSeq++;

			/* �Î~��Ԃň�莞�Ԃ��ĂΎ��̃V�[�P���X */
			if(countBalanceOffSeq == ASSIST_BLANCE_OFF_COUNT)
			{
				hensa0 = 0;
				integral = 0;
				SeqNo = UNDER_RETURN_SEQ_NO;
				countBalanceOnSeq = 0;
				countReturnGateSeq = 0;
				InitTailRunControlStraightP();
			}
			break;


		case UNDER_RETURN_SEQ_NO:		//�Q�[�g������i���s����@�i���i���s�j
			turn = 0;
			forward = (signed char)SCurve((float)forwardTarget, (float)forwardTargetPrev, accelTmax, &accelT);
			// ������PID
//			ctrl_direction((float)forward);
//			turn = get_model_turn();
//			forward = get_model_forward();
//			TailRunControl(forward, turn);
			TailRunControlStraightP(forward, -10.0f);
			tail_pid_control(tailKp, tailKi, tailKd, GATE_TAIL_ANGLE_UNDER_GATE, tailMaxMv, tailMinMv);

			if(countReturnGateSeq == 0)
			{
				init_DistanceFromPoint();

				LookUpGateStageStm_ChangeSpeed(-30, 0.5f);
				InitTailRunControlStraightP();
			}

			//���s��������
			if((calc_getDistanceFromPoint() * 100.0f * (-1.0)) >= UNDER_BAR_RUN_RET_CM)
			{
				turn = 0;
				SeqNo = BRAKE2_SEQ_NO;
				countBalanceOffSeq =0;
			}

			countReturnGateSeq++;
#ifdef USE_LOG
//			ecrobot_bt_data_logger((S8)(SeqNo * 10), (S8)(countReturnGateSeq / 10));
#endif
			break;

		case BRAKE2_SEQ_NO:		//�Î~����
			if(0 == countBalanceOffSeq)
			{
				LookUpGateStageStm_ChangeAngle(GATE_TAIL_ANGLE_UNDER_GATE, 1.0f);
				LookUpGateStageStm_ChangeSpeed(0, 0.5f);
				turn = 0;
			}
			forward = (signed char)SCurve((float)forwardTarget, (float)forwardTargetPrev, accelTmax, &accelT);
			TailRunControl(forward, turn);

			tail_pid_control(tailKp, tailKi, tailKd, GATE_TAIL_ANGLE_UNDER_GATE, tailMaxMv, tailMinMv);

			/* �Î~�̌��o */
			countBalanceOffSeq++;

			/* �Î~��Ԃň�莞�Ԃ��ĂΎ��̃V�[�P���X */
			if(countBalanceOffSeq == ASSIST_BLANCE_OFF_COUNT)
			{
				hensa0 = 0;
				integral = 0;
				SeqNo = UNDER_GATE2_SEQ_NO;
				countUnderGateSeq = 0;
//				InitTailRunControlStraightP();
				change_model_black_to_tail();
				reset_model_state();
			}
			break;


		case UNDER_GATE2_SEQ_NO:		//�Q�[�g����O�i���s���� �i���i���s�j


			turn = 0;
			forward = (signed char)SCurve((float)forwardTarget, (float)forwardTargetPrev, accelTmax, &accelT);
			// ������PID
			ctrl_direction((float)forward);
			turn = get_model_turn();
			forward = get_model_forward();
			TailRunControl(forward, turn);
//			TailRunControlStraightP(forward, 10.0f);
			tail_pid_control(tailKp, tailKi, tailKd, GATE_TAIL_ANGLE_UNDER_GATE, tailMaxMv, tailMinMv);

			if(countUnderGateSeq == 0)
			{
				init_DistanceFromPoint();

				LookUpGateStageStm_ChangeSpeed(30, 0.5f);
//				InitTailRunControlStraightP();
			}

			//���s��������
			if((calc_getDistanceFromPoint() * 100.0f) >= UNDER_BAR_RUN_CM)
			{
				turn = 0;
				LookUpGateStageStm_ChangeSpeed(0, 1.0f);
				SeqNo = BALANCE_ON_SEQ_NO;
				countBalanceOnSeq =0;
				//isreturn = isreturn++;
			}

			countUnderGateSeq++;
#ifdef USE_LOG
//			ecrobot_bt_data_logger((S8)(SeqNo * 10), (S8)(countUnderGateSeq / 10));
#endif
			break;

		case BALANCE_ON_SEQ_NO: // �̂��N����
			if(0 == countBalanceOnSeq) LookUpGateStageStm_ChangeAngle(TAIL_ANGLE_TAILRUN, 1.0f);
			forward = (signed char)SCurve((float)forwardTarget, (float)forwardTargetPrev, accelTmax, &accelT);
			TailRunControl(forward, 0);
			angle = (signed char)SCurve((float)angleTarget, (float)angleTargetPrev, angleAccelTmax, &angleAccelT);
			tail_pid_control(tailKp, tailKi, tailKd, angle, tailMaxMv, tailMinMv);

			if(TWO_BLANCE_ON_COUNT <= countBalanceOnSeq && countBalanceOnSeq < THREE_BLANCE_ON_COUNT)
			{
				// �W���C�����o�̏�����
				if(gyroInitflg == 1)
				{
					InitGyroDetect(STOP_THRESHOLD);
					gyroInitflg = 2;
				}
				else
				{
					if(GyroDetect() == 0)
					{
						countBalanceOnSeq++;
					}
				}
			}
			else
			{
				countBalanceOnSeq++;
			}

			/* �Î~��Ԃň�莞�Ԃ��ĂΎ��̃V�[�P���X */
			if(countBalanceOnSeq == THREE_BLANCE_ON_COUNT)
			{
				reset_model_state();
				SeqNo = AFTER_GATE_SEQ_NO;
				countAfterGateSeq = 0;
				InitTailRunControlStraightP();
				change_model_tail_to_black();
				reset_model_state();
			}
			break;
		case AFTER_GATE_SEQ_NO:

			if(countAfterGateSeq == 0)
			{
				init_DistanceFromPoint();
				LookUpGateStageStm_ChangeSpeed(-20, 0.5f);
			}

			//���s��������
//			if((calc_getDistanceFromPoint() * 100.0f) >= 15)
			if((calc_getDistanceFromPoint() * 100.0f) <= -10)
			{
				return 1;
			}
			else
			{
				forward = (signed char)SCurve((float)forwardTarget, (float)forwardTargetPrev, accelTmax, &accelT);
//				// ������PID
//				ctrl_direction((float)forward);
//				turn = get_model_turn();
//				forward = get_model_forward();
//				TailRunControl(forward, turn);
				TailRunControlStraightP(forward, -10.0f);
				tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv);

				countAfterGateSeq++;
			}
#if 0
			tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv);
#ifdef USE_LOG
			ecrobot_bt_data_logger((S8)(SeqNo * 10), 0);
#endif
			//default��
			SeqNo++;
#else
			//�K�����s�̂܂܃S�[����
			//if(TailToBalanceMain() == 1)
			//{
			//	return 1;
			//}
#endif
			break;
		case END_GATE_SEQ_NO:
			break;

		default:
			return 1;
			break;
	}
//	ecrobot_bt_data_logger((S8)SeqNo+10, (S8)(get_model_mu()*1000)); // �}�[�J���m臊m�F�p
//	ecrobot_debug1((UINT)(SeqNo), (UINT)0, (UINT)0);
	ecrobot_bt_data_logger((S8)SeqNo+10, (S8)(get_model_light()/8)); // �}�[�J���m臊m�F�p
	return 0;
}

#define ANGLE_CORRECT_VALUE 5 // �K���̊p�x����̕␳
/* �K�����ڕW�p�x�ɋ߂��Ȃ�1��Ԃ� */
int TailAngleCheck(int reqAngle)
{
	int angle = nxt_motor_get_count(NXT_PORT_A);
	if(reqAngle - ANGLE_CORRECT_VALUE < angle && angle < reqAngle + ANGLE_CORRECT_VALUE)
	{
		return 1;
	}
	return 0;
}

