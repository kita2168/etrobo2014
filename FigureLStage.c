/* 2014/07/16 created by y.tanaka ��N���A�P�̃e�X�g�p */

#include "FigureLStage.h"

#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

#include "LineTrace.h"
#include "Calc.h"
#include "Control.h"
#include "SensorIn.h"
#include "LineTraceParam.h"
#include "SpeedoMeter.h"
#include "BalanceToTail.h"

/* �V�[�P���X�ԍ���` */
#define FIGUREL_SEQ_NO_00 0	// �����Z�b�g
#define FIGUREL_SEQ_NO_01 1 // �K�����s�i���C���g���[�XP�j	�ʂ̌��o
#define FIGUREL_SEQ_NO_02 2 // �K�����s�i�Œ葀��j			�ʂւ̓��č���
#define FIGUREL_SEQ_NO_03 3 // �K�����s�i���i����j			��苗�����
#define FIGUREL_SEQ_NO_04 4 // ���s�Ȃ�						�K��PID����
#define FIGUREL_SEQ_NO_05 5 // ���s�Ȃ�						�|�����s�O���Z�b�g
#define FIGUREL_SEQ_NO_06 6 // �|�����s�i���i����j			�i��
#define FIGUREL_SEQ_NO_07 7 // �|�����s�i���i����j			�E�ɉ�]�i���C���̉E�ɏo�邽�߉E�������j
#define FIGUREL_SEQ_NO_08 8 // �|�����s�i���i����j			�E�ɏ����O�i
#define FIGUREL_SEQ_NO_09 9 // �|�����s�i���i����j			���ɉ�]�i���C���̕��������j
#define FIGUREL_SEQ_NO_10 10 // �|�����s					�̏�Ń��C���g���[�X���A�X�s���ʒu�܂ňړ�
#define FIGUREL_SEQ_NO_11 11 // �|�����s���K�����s
#define FIGUREL_SEQ_NO_12 12 // �K�����s					�X�s��
#define FIGUREL_SEQ_NO_13 13 // ���s�Ȃ�					�K��PID����
#define FIGUREL_SEQ_NO_14 14 // ���s�Ȃ�					�|�����s�O���Z�b�g
#define FIGUREL_SEQ_NO_15 15 // �̏�Ń��C���g���[�X���s���Ȃ���A�i�����~���
#define FIGUREL_SEQ_NO_15_1 25 // �̏�Ń��C���g���[�X���s���Ȃ���A�i�����~���
#define FIGUREL_SEQ_NO_16 16 // �|�����s���K�����s
#define FIGUREL_SEQ_NO_17 17 // �K�����s					�E��90�x��]�i���C����T�����߁j
#define FIGUREL_SEQ_NO_18 18 // �|�����s�i���i����j		���C���̉E���ɏo��
#define FIGUREL_SEQ_NO_19 19 // �|�����s�i���i����j		���C����T��
#define FIGUREL_SEQ_NO_20 20 // �K�����s					���ɉ�]�i���C�����A�̂��߁j

/* �O�i�l��` */
#define SEQ_NO_00_FORWARD 			15
#define SEQ_NO_01_FORWARD 			15
#define SEQ_NO_02_FORWARD 			15 // �ʍ��킹
#define SEQ_NO_03_FORWARD 			(-20) //�ꎞ���
#define SEQ_NO_06_FORWARD 			20	//�i��
#define SEQ_NO_08_FORWARD			10	//���C�������̂��߁A�E�Ɉړ�
#define SEQ_NO_10_FORWARD 			15	//���C�����A���A�X�s���ʒu�܂�
#define SEQ_NO_15_FORWARD 			15	//�i�����~��āA�}�[�J����������܂�
#define SEQ_NO_18_FORWARD 			15	//�}�[�J�̉E�Ɉړ�
#define SEQ_NO_19_FORWARD 			(-15)	//�}�[�J��T��

/* ����l��` */
#define SEQ_NO_02_TURN				15 //�ʍ��킹
#define SEQ_NO_07_TURN				30 //���C�������̂��߁A�E�Ɉړ�
#define SEQ_NO_09_TURN				30 //���C�����A�̂��߁A���Ɉړ�
#define SEQ_NO_12_TURN				365 //�X�s��
#define SEQ_NO_17_TURN				90 //���C�������̂��߁A�E�Ɉړ�
#define SEQ_NO_20_TURN				65 //���C�����A

/* ������`(cm) */
#define SEQ_NO_03_DISTANCE 			(-15.0f) // ��ދ���
#define SEQ_NO_06_DISTANCE 			(35.0f) // �������鋗���܂�
#define SEQ_NO_07_DISTANCE 			(20.0f) // �X�s�����鋗���܂�
#define SEQ_NO_08_DISTANCE 			(5.0f) // ���C�������̂��߁A�E�Ɉړ�
#define SEQ_NO_15_DISTANCE 			(5.0f) // �X�s���ʒu����i���~���܂�
#define SEQ_NO_15_1_DISTANCE 			(25.0f) // �X�s���ʒu����i���~���܂�
#define SEQ_NO_18_DISTANCE 			(10.0f) // ���C���Ɉړ����鋗��

/* ���� */
#define SEQ_NO_06_TIMER_S 			250		// �ɂ����邽�߂̉����J�n
#define SEQ_NO_06_TIMER_E 			525		// �ɂ����邽�߂̉����I��

/* �D�F�}�[�J���m�W�� */
#define SEQ_NO_19_GRAY				(0.95)

// ���s�̎��]�X�e�[�^�X
#define TURN_LEFT 0
#define TURN_RIGHT 1

/* �W���C���I�t�Z�b�g��` */
#define SEQ_NO_06_OFFSET 			10	// �i���o��p

/* ���݃V�[�P���X�ԍ� */
static S8 figurelSeqNo = FIGUREL_SEQ_NO_00;

/* ���Z���T���ݒl */
static U16 lightValue;		/* 0-1023 */

/* �L�����u���[�V�������Z���T�l */
static U16 black;
static U16 white;
	
/* ���s�p�p�����[�^ */
static signed char forward;			/* �O��i���� */
static signed char turn;			/* ���񖽗� */

static signed char forwardTarget = 0;		// �ڕW���x�w��
static signed char forwardTargetPrev = 0;	// �O��ڕW���x�w��
static float accelT = 0.0f;
static float accelTmax = 0;

/* �K���p�x */
signed int angle;
static signed char angleTarget = 0;
static signed char angleTargetPrev = 0;
static float angleAccelT = 0.0f;
static float angleAccelTmax = 0;

/* ���Ԕ���p�p�����[�^ */
static U32 count = 0;		/* �V�[�P���X�ɓ����Ă���̃J�E���g�l */
static U16 timer_1s = 250;
static U16 timer_2s = 500;

/* ���xLPF�p�p�����[�^ */
static S16 velocityInLPF = 0;	/* ���x���ݒl(cm/s)�i���[�p�X�t�j */
static float vel_omega0;
static float vel_a;
static float vel_b0;
static float vel_b1;
static float vel_b2;
static float vel_a1;
static float vel_a2;
static float vel_pre1;
static float vel_pre2;

/* �W���C���I�t�Z�b�g���s�p */
static S8 offset;

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
extern void reset_model_state();
extern U16 get_model_center();
extern U16 get_model_light();

void InitFigureLStageStm_ChangeSpeed()
{
	forward = 0.0f;
	forwardTargetPrev = 0.0f;
	accelT = 0.0f;
}
void FigureLStageStm_ChangeSpeed(float spd, float Tmax)
{
	forwardTargetPrev = forward;
	forwardTarget = spd;
	accelT = 0.0f;
	accelTmax = Tmax;
}

void BalanceToTail_ChangeAngle(float ang, float Tmax)
{
	angleTargetPrev = nxt_motor_get_count(NXT_PORT_A);
	angleTarget = ang;
	angleAccelT = 0.0f;
	angleAccelTmax = Tmax;
}

int FigureLStageMain(void)
{
	static char hensa0Initialized = 0;
	static float hensa0;
	static float integral;
	
	// �K��PID�Ƃ肠�����K��
	float tailKp = TAIL_KP;
	float tailKi = TAIL_KI;
	float tailKd = TAIL_KD;
	float tailMaxMv = 100.0f;
	float tailMinMv = -100.0f;
	
	// ���Z���T�l�擾
	lightValue = ecrobot_get_light_sensor(NXT_PORT_S3);
	
	// �L�����u���[�V�����l�擾
	black = LineTraceControl_LightBlackThreshold();
	white = LineTraceControl_LightWhiteThreshold();
	
	// LPF�����������x����Ɍv�Z����i�ŏ��̌v�Z�Ɏ��Ԃ������邽�߁j
	float velocity = SpeedoMeter();
	velocityInLPF = LPF((S16)(velocity * 100.0f), vel_omega0, vel_a, vel_b0, vel_b1, vel_b2, vel_a1, vel_a2, &vel_pre1, &vel_pre2);

	// �O�i�l�v�Z
	forward = (signed char)SCurve((float)forwardTarget, (float)forwardTargetPrev, accelTmax, &accelT);
			
	if(!hensa0Initialized)
	{
		// �O��΍��ێ��l�̏�����
		hensa0 = 0.0f;
		integral = 0.0f;
		hensa0Initialized = 1;
		change_model_gray_to_black();
		init_DistanceFromPoint();
	}
	
	switch(figurelSeqNo)
	{
		case FIGUREL_SEQ_NO_00:		// �����Z�b�g
		{
			
			FigureLStageStm_ChangeSpeed(SEQ_NO_00_FORWARD, 0.3f);
			figurelSeqNo = FIGUREL_SEQ_NO_01;
			
			#ifdef USE_LOG
			ecrobot_debug1((UINT)figurelSeqNo, (UINT)(forward), 0);
			ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(forward));
			#endif
			break;
		}
		case FIGUREL_SEQ_NO_01: // �K�����s�i���C���g���[�XP�j	�ʂ̌��o
		{
			
			// ������PID
			ctrl_direction((float)forward);
			turn = get_model_turn();
			forward = get_model_forward();	
			
			TailRunControl(forward, turn);

			tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv);

			if(timer_1s / 2 == count)
			{
				float distance = calc_getDistanceFromPoint();
				if(distance <= 0.01f)
				{
					figurelSeqNo = FIGUREL_SEQ_NO_02;
				}
				
				count = 0;
				
			}
			else
			{
				if(count == 0)
				{
					init_DistanceFromPoint();
				}
				
				count++;
			}
			#ifdef USE_LOG
			ecrobot_debug1((UINT)figurelSeqNo, (UINT)(forward), 0);
			ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(forward));
			#endif
			break;
		}
		case FIGUREL_SEQ_NO_02: // �K�����s�i�Œ葀��j			�ʂւ̓��č���
		{
			
			if(count == 0)
			{
				FigureLStageStm_ChangeSpeed(SEQ_NO_02_FORWARD, 0.1f);
			}
			
			// �̒[�Ƀ^�C�������킹�铮��
			if(timer_1s / 2 > count)
			{
				turn = SEQ_NO_02_TURN;
				count++;
			}
			else if(timer_1s > count)
			{
				turn = -SEQ_NO_02_TURN;
				count++;
			}
			else
			{
				turn = 0;
				
				figurelSeqNo = FIGUREL_SEQ_NO_03;
				count = 0;
			}
			
			
			TailRunControl(forward, turn);
			tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv);

			#ifdef USE_LOG
			ecrobot_debug1((UINT)figurelSeqNo, (UINT)(forward), 0);
			ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(forward));
			#endif
			break;
		}
		case FIGUREL_SEQ_NO_03: // �K�����s�i���i����j			��苗�����
		{
			if(count == 0)
			{
				init_DistanceFromPoint();
				InitTailRunControlStraightP();
				
				FigureLStageStm_ChangeSpeed(SEQ_NO_03_FORWARD, 0.3f);
			}
			
			// ��ނ�������(cm)
			float distance = calc_getDistanceFromPoint() * 100.0f;
			
			if(distance > SEQ_NO_03_DISTANCE)
			{
				TailRunControlStraightP(forward, -2.0f);
				tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv);

				count++;
			}
			else
			{
				forward=0;
				turn = 0;
				FigureLStageStm_ChangeSpeed(0, 0.0f);
				
				figurelSeqNo = FIGUREL_SEQ_NO_04;
				count = 0;
			}
			#ifdef USE_LOG
			ecrobot_debug1((UINT)figurelSeqNo, (UINT)(forward), 0);
			ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(forward));
			#endif
			break;
		}
		case FIGUREL_SEQ_NO_04: // ���s�Ȃ�						�K��PID����
		{
			if(timer_1s > count)
			{
				if(count == 0)
				{
					
					//�|���O�Ƀ��[�^�p�x�����Z�b�g
					nxt_motor_set_count(NXT_PORT_B, 0);
					nxt_motor_set_count(NXT_PORT_C, 0);
					
					balance_init();
					
					init_DistanceFromPoint();
				}
				tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv);
				count++;
			}
			else
			{
				forward=0;
				turn = 0;
				
				figurelSeqNo = FIGUREL_SEQ_NO_05;
				count = 0;
			}
			#ifdef USE_LOG
			ecrobot_debug1((UINT)figurelSeqNo, (UINT)(forward), 0);
			ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(forward));
			#endif
			break;
		}
		case FIGUREL_SEQ_NO_05: // ���s�Ȃ�						�|�����s�O���Z�b�g
		{
			if(count == 0)
			{
				InitBalanceControlStraightP();
			}
			
			if(timer_1s > count)
			{
				tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv);
				BalanceControlStraightP(forward, 3.0f);
				count++;
			}
			else if(timer_2s > count)
			{
				if(timer_1s == count)
				{
					BalanceToTail_ChangeAngle(TAIL_ANGLE_BALANCE_RUN, 1.0f);
				}
				angle = (signed char)SCurve((float)angleTarget, (float)angleTargetPrev, angleAccelTmax, &angleAccelT);
				tail_pid_control(tailKp, tailKi, tailKd, angle, tailMaxMv, tailMinMv);
				
				BalanceControlStraightP(forward, 3.0f);
				
				count++;
			}
			else
			{
				figurelSeqNo = FIGUREL_SEQ_NO_06;
				count = 0;
			}
			#ifdef USE_LOG
			ecrobot_debug1((UINT)figurelSeqNo, (UINT)(forward), 0);
			ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(forward));
			#endif
			break;
		}
		case FIGUREL_SEQ_NO_06: // �|�����s�i���i����j			�i��
		{
			// �|���J�n����̋���(cm)
			float distance = calc_getDistanceFromPoint() * 100.0f;
			
			if(count == 0)
			{
				FigureLStageStm_ChangeSpeed(SEQ_NO_06_FORWARD, 0.3f);
			}
			
			offset = 0;
			if(distance >= SEQ_NO_06_DISTANCE)
			{
				figurelSeqNo = FIGUREL_SEQ_NO_07;
				
				FigureLStageStm_ChangeSpeed(0, 0.1f);
				
				count = 0;
			}
			else
			{
				if(SEQ_NO_06_TIMER_S < count && count < SEQ_NO_06_TIMER_E)	//250 �` 525
				{
					offset = SEQ_NO_06_OFFSET;
				}
				
				tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_BALANCE_RUN, tailMaxMv, tailMinMv);
				BalanceControl1StraightP(forward, 10.0f, offset);
				
				count++;
			}
			#ifdef USE_LOG
			ecrobot_debug1((UINT)figurelSeqNo, (UINT)(forward), 0);
			ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(forward));
			#endif
			break;
		}
		case FIGUREL_SEQ_NO_07: // �|�����s�i���i����j			�E�ɉ�]�i���C���̉E�ɏo�邽�߉E�������j
		{
			turn = 70;
			forward = 0;
	
			if(count == 0)
			{
				init_rotateDistance(TURN_RIGHT);		/* ��]�������菉���� */
				
			}
			
			//�^�[����������
			if(check_rotateComplete(SEQ_NO_07_TURN))
			{
				
				figurelSeqNo = FIGUREL_SEQ_NO_08;
				
				InitBalanceControlStraightP();
				init_DistanceFromPoint();
				
				FigureLStageStm_ChangeSpeed(SEQ_NO_08_FORWARD, 0.2f);
				turn = 0;
				
				count = 0;
			}
			else
			{
				BalanceControl(forward, turn);
				tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_BALANCE_RUN, tailMaxMv, tailMinMv); 
				count++;	
			}
			#ifdef USE_LOG
			ecrobot_debug1((UINT)figurelSeqNo, (UINT)(forward), 0);
			ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(forward));
			#endif
			break;
		}
		case FIGUREL_SEQ_NO_08: // �|�����s�i���i����j			�E�ɏ����O�i
		{
			// �|���J�n����̋���(cm)
			float distance = calc_getDistanceFromPoint() * 100.0f;
			
			if(distance >= SEQ_NO_08_DISTANCE)	// 5.0f
			{
				figurelSeqNo = FIGUREL_SEQ_NO_09;
				count = 0;
			}
			else
			{
				turn = 0;
				tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_BALANCE_RUN, tailMaxMv, tailMinMv);
				BalanceControlStraightP(forward, 3.0f);
				
				count++;
			}
			#ifdef USE_LOG
			ecrobot_debug1((UINT)figurelSeqNo, (UINT)(forward), 0);
			ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(forward));
			#endif
			break;
		}
		case FIGUREL_SEQ_NO_09: // �|�����s�i���i����j			���ɉ�]�i���C���̕��������j
		{
			turn = -70;
			forward = 0;
	
			if(count == 0)
			{
				init_rotateDistance(TURN_LEFT);		/* ��]�������菉���� */
				
			}
			
			//�^�[����������
			if(check_rotateComplete(SEQ_NO_09_TURN))
			{
				forward=0;
				turn = 0;
				InitFigureLStageStm_ChangeSpeed();
				
				figurelSeqNo = FIGUREL_SEQ_NO_10;
				
				init_DistanceFromPoint();
				
				count = 0;
			}
			else
			{
				BalanceControl(forward, turn);
				tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_BALANCE_RUN, tailMaxMv, tailMinMv); 
				count++;	
			}
			#ifdef USE_LOG
			ecrobot_debug1((UINT)figurelSeqNo, (UINT)(forward), 0);
			ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(forward));
			#endif
			break;
		}
		case FIGUREL_SEQ_NO_10: // �|�����s			�̏�Ń��C���g���[�X���A�X�s���ʒu�܂ňړ�
		{
			// �|���J�n����̋���(cm)
			float distance = calc_getDistanceFromPoint() * 100.0f;

			// ������PID
			ctrl_direction((float)forward);
			turn = get_model_turn();
			forward = get_model_forward();	
			
			if(count == 0)
			{
				FigureLStageStm_ChangeSpeed(SEQ_NO_10_FORWARD, 0.1f);
			}
			
			if(distance >= SEQ_NO_07_DISTANCE)
			{
				figurelSeqNo = FIGUREL_SEQ_NO_11;
				count = 0;
			}
			else
			{
				tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_BALANCE_RUN, tailMaxMv, tailMinMv);
				BalanceControl(forward, turn);
				
				count++;
			}
			#ifdef USE_LOG
			ecrobot_debug1((UINT)figurelSeqNo, (UINT)(forward), 0);
			ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(forward));
			#endif
			break;
		}
		case FIGUREL_SEQ_NO_11: // �|�����s���K�����s
		{
			if(count == 0)
			{
				FigureLStageStm_ChangeSpeed(0, 0.0f);
				
				BalanceToTail_Init();
			}
			
			if(BalanceToTailMain() == 1)
			{
				figurelSeqNo = FIGUREL_SEQ_NO_12;
				forward=0;
				turn = 0;
				count = 0;
			
			}else{
				count++;
			}
			#ifdef USE_LOG
			ecrobot_debug1((UINT)figurelSeqNo, (UINT)(forward), 0);
			ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(forward));
			#endif
			break;
		}
		case FIGUREL_SEQ_NO_12: // �K�����s		�X�s��
		{
			turn = -70;
			forward = 0;
	
			if(count == 0)
			{
				init_rotateDistance(TURN_LEFT);		/* ��]�������菉���� */
				
			}
			
			//�X�s����������
			if(check_rotateComplete(SEQ_NO_12_TURN))	// 365
			{
				forward=0;
				FigureLStageStm_ChangeSpeed(0, 0.1f);
				turn = 0;
				
				figurelSeqNo = FIGUREL_SEQ_NO_13;
				count = 0;
			}
			else
			{
				TailRunControl(forward, turn);
				tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv); 
				count++;	
			}
			#ifdef USE_LOG
			ecrobot_debug1((UINT)figurelSeqNo, (UINT)(forward), 0);
			ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(forward));
			#endif
			break;
		}
		case FIGUREL_SEQ_NO_13: // ���s�Ȃ�						�K��PID����
		{
			
			if(timer_1s > count)
			{
				//��~
				forward=0;
				turn = 0;
				
				TailRunControl(forward, turn);
				tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv);
			
				count++;
			}
			else if(timer_2s > count)
			{
				if(count == timer_1s)
				{
					//�|���O�Ƀ��[�^�p�x�����Z�b�g
					nxt_motor_set_count(NXT_PORT_B, 0);
					nxt_motor_set_count(NXT_PORT_C, 0);
					
					balance_init();
					
					init_DistanceFromPoint();
				}
				
				tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv);
			
				count++;
			}
			else
			{
				forward=0;
				turn = 0;
				
				figurelSeqNo = FIGUREL_SEQ_NO_14;
				count = 0;
			}
			#ifdef USE_LOG
			ecrobot_debug1((UINT)figurelSeqNo, (UINT)(forward), 0);
			ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(forward));
			#endif
			break;
		}
		case FIGUREL_SEQ_NO_14: // ���s�Ȃ�						�|�����s�O���Z�b�g
		{
			forward=0;
			turn = 0;
			
			if(timer_2s > count)
			{
				if(count == timer_1s)
				{
					BalanceToTail_ChangeAngle(TAIL_ANGLE_BALANCE_RUN, 1.0f);
				}
				
				angle = (signed char)SCurve((float)angleTarget, (float)angleTargetPrev, angleAccelTmax, &angleAccelT);
				tail_pid_control(tailKp, tailKi, tailKd, angle, tailMaxMv, tailMinMv);
				
				BalanceControl(forward, turn);
				
				count++;
			}
			else
			{
				figurelSeqNo = FIGUREL_SEQ_NO_15;
				
				count = 0;
			}
			#ifdef USE_LOG
			ecrobot_debug1((UINT)figurelSeqNo, (UINT)(forward), 0);
			ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(forward));
			#endif
			break;
		}
		case FIGUREL_SEQ_NO_15: // �̏�Ń��C���g���[�X���s���Ȃ���A�i�����~���
		{
			// �X�s���ʒu����̋���(cm)
			float distance = calc_getDistanceFromPoint() * 100.0f;
			
			// ������PID
			ctrl_direction((float)forward);
			turn = get_model_turn();
			forward = get_model_forward();	
			
			if(count == 0)
			{
				FigureLStageStm_ChangeSpeed(SEQ_NO_15_FORWARD, 0.1f);
			}
			
			if(distance > SEQ_NO_15_DISTANCE)
			{
				figurelSeqNo = FIGUREL_SEQ_NO_15_1;
				
				InitBalanceControlStraightP();
				init_DistanceFromPoint();
				
				count++;
			}
			else
			{	
				tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_BALANCE_RUN, tailMaxMv, tailMinMv);
				BalanceControl(forward, turn);
				
				count++;
			}
			#ifdef USE_LOG
			ecrobot_debug1((UINT)figurelSeqNo, (UINT)(forward), 0);
			ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(forward));
			#endif
			break;
		}
		case FIGUREL_SEQ_NO_15_1: // �̏�Œ��i���s�Œi�����~���
		{
			// �X�s���ʒu����̋���(cm)
			float distance = calc_getDistanceFromPoint() * 100.0f;
			
			if(count == 0)
			{
				FigureLStageStm_ChangeSpeed(SEQ_NO_15_FORWARD, 0.1f);
			}
			
			if(distance > SEQ_NO_15_1_DISTANCE)
			{
				figurelSeqNo = FIGUREL_SEQ_NO_16;
				forward = 0;
				count = 0;
			}
			else
			{	
				tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_BALANCE_RUN, tailMaxMv, tailMinMv);
				BalanceControlStraightP(forward, 3.0f);
				
				count++;
			}
			#ifdef USE_LOG
			ecrobot_debug1((UINT)figurelSeqNo, (UINT)(forward), 0);
			ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(forward));
			#endif
			break;
		}
		case FIGUREL_SEQ_NO_16: // �|�����s���K�����s
		{
			
			if(count == 0)
			{
				BalanceToTail_Init();
			}
			
			if(BalanceToTailMain() == 1)
			{
				figurelSeqNo = FIGUREL_SEQ_NO_17;
				forward = 0;
				count = 0;
			
			}else{
				count++;
			}
			#ifdef USE_LOG
			ecrobot_debug1((UINT)figurelSeqNo, (UINT)(forward), 0);
			ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(forward));
			#endif
			break;
		}
		case FIGUREL_SEQ_NO_17: // �K�����s		�E��90�x��]�i���C����T�����߁j
		{
			turn = 70;
			forward = 0;
	
			if(count == 0)
			{
				init_rotateDistance(TURN_RIGHT);		/* ��]�������菉���� */
				
			}
			
			//�^�[����������
			if(check_rotateComplete(SEQ_NO_17_TURN))
			{
				
				figurelSeqNo = FIGUREL_SEQ_NO_18;
				
				InitTailRunControlStraightP();
				init_DistanceFromPoint();
				
				turn = 0;
				
				count = 0;
			}
			else
			{
				TailRunControl(forward, turn);
				tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv); 
				count++;	
			}
			#ifdef USE_LOG
			ecrobot_debug1((UINT)figurelSeqNo, (UINT)(forward), 0);
			ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(forward));
			#endif
			break;
		}
		case FIGUREL_SEQ_NO_18: // �|�����s�i���i����j			���C���̉E���ɏo��
		{
			// �|���J�n����̋���(cm)
			float distance = calc_getDistanceFromPoint() * 100.0f;
			
			if(distance >= SEQ_NO_18_DISTANCE)
			{
				figurelSeqNo = FIGUREL_SEQ_NO_19;
				
				forward = 0;
				
				count = 0;
			}
			else
			{
				forward = SEQ_NO_18_FORWARD;	// 15
				turn = 0;
				tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv);
				TailRunControlStraightP(forward, 3.0f);
				
				count++;
			}
			#ifdef USE_LOG
			ecrobot_debug1((UINT)figurelSeqNo, (UINT)(forward), 0);
			ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(forward));
			#endif
			break;
		}
		case FIGUREL_SEQ_NO_19: // �|�����s�i���i����j			���C����T��
		{
			U16 lightSearchLine = (white + black) / 2;
			if(lightValue >= (lightSearchLine * SEQ_NO_19_GRAY))
			{
				figurelSeqNo = FIGUREL_SEQ_NO_20;
				count = 0;
			}
			else
			{
				forward = SEQ_NO_19_FORWARD;
				turn = 0;
				tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv);
				TailRunControlStraightP(forward, -3.0f);
				
				count++;
			}
			#ifdef USE_LOG
			ecrobot_debug1((UINT)figurelSeqNo, (UINT)(forward), 0);
			ecrobot_bt_data_logger((S8)(lightValue / 10), (S8)(lightSearchLine / 10));
			#endif
			break;
		}
		case FIGUREL_SEQ_NO_20: // �K�����s		���ɉ�]�i���C�����A�̂��߁j
		{
			turn = -70;
			forward = 0;
	
			if(count == 0)
			{
				init_rotateDistance(TURN_LEFT);		/* ��]�������菉���� */
				
			}
			
			//�^�[����������
			if(check_rotateComplete(SEQ_NO_20_TURN))
			{
				return 1;
			}
			else
			{
				TailRunControl(forward, turn);
				tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv); 
				count++;	
			}
			#ifdef USE_LOG
			ecrobot_debug1((UINT)figurelSeqNo, (UINT)(calc_getRotateDistance() * 100.0f), 0);
			ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(calc_getRotateDistance() * 100.0f));
			#endif
			break;
		}
		default:
		{
			forward = 0;
			turn = 0;
			break;
		}
	}
	
	/* �S�̃��M���O */
	//ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(forward));
	ecrobot_debug1((UINT)figurelSeqNo, 0, 0);
#ifdef USE_LOG
	//ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(forward));
	//ecrobot_debug1((UINT)figurelSeqNo, 0, 0);
#endif

	return 0;
}
