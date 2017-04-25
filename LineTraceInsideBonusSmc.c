// 2012/08/14 created by s.sahara

// IN���{�[�i�X���s��ԑJ�ڃ}�V��

#include "LineTraceInsideBonusSmc.h"

#include "SensorIn.h"
#include "Control.h"

#include "LineTraceControl.h"
#include "LineTraceParam.h"

#include "CourseDef.h"
#include "LineTraceInsideBasicSmc.h"

#include "Timer.h"
#include "BalanceToTail.h"

#if 0
// SEQ��ԃ^�C������
static UINT mT = 0;
static S8 mSeqTime[LT_IN2_SEQ_MAX];
static S8 mSeqDist[LT_IN2_SEQ_MAX];
#endif

static signed char forward;		// �O��i����
static signed char turn;		// ���񖽗�

static signed char forwardTarget = 0;		// �ڕW���x�w��
static signed char forwardTargetPrev = 0;		// �O��ڕW���x�w��
static float accelT = 0.0f;
static float accelTmax = 0;

static char onMarker = 0;
//static char onMarkerPrev = 0;

static float hensa0;
static float integral;

static float Kspd = 1.0f;
#if 0 //del by k.nakagawa ��Ԃ��Ƃ̃Q�C���ύX���폜
static float dirKp = BALANCERUN_DIRECTION_KP;
static float dirKi = BALANCERUN_DIRECTION_KI;
static float dirKd = BALANCERUN_DIRECTION_KD;
#endif

static int seqNoPrv = -1;

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


void LineTraceInsideBonusSmc_Init(signed char arg_forward)
{
#if 0
	U16 black = LineTraceControl_LightBlackThreshold();
	U16 white = LineTraceControl_LightWhiteThreshold();
#endif

	onMarker = 0;
	//onMarkerPrev = 0;

#if 0
	mT = 0;
	for(int i=0;i<LT_IN2_SEQ_MAX;i++)
	{
		mSeqTime[i] = 0;
		mSeqDist[i] = 0;
	}
#endif

	forward = arg_forward;
	forwardTarget = arg_forward; // �����ڕW���x�w��
	forwardTargetPrev = arg_forward;

	// �O��΍��ێ��l�̏�����
	hensa0 = 0.0f;
	integral = 0;

	seqNoPrv = -1;

//	init_model();
}

#if 0
// ��PU����p
void LineTraceInsideBonusSmc_logTurn(signed char turn)
{
	static S32 integralTurn = 0;
	//
	if(turn > 0){
		if(integralTurn < 0)integralTurn = 0;
	}
	else if(turn < 0){
		if(integralTurn > 0)integralTurn = 0;
	}
	integralTurn += turn;
#ifdef USE_LOG
	//ecrobot_bt_data_logger(turn, (S8)integralTurn);
#endif
	//ecrobot_debug1((U16)turn, (U16)integral, (U16)(integral * -1));
}

void SaveSeqEndTime(UINT seqNo, UINT absTime, float distanceAbs)
{
	mSeqTime[seqNo] = (S8)(absTime * 0.004 * 10.0); // [1/10sec]
	mSeqDist[seqNo] = (S8)(distanceAbs * 10.0);	// [1/10seq]
}

void LineTraceInsideBonusSmc_BTSendLapTime()
{
	for(int i=0;i<LT_IN2_SEQ_MAX;i++)
	{
#ifdef USE_LOG
		ecrobot_bt_data_logger(mSeqDist[i], mSeqTime[i]);
#endif
		systick_wait_ms(10);
	}
}
#endif

void LineTraceInsideBonusSmc_ChangeSpeed(float spd, float Tmax)
{
	forwardTargetPrev = forward;
	forwardTarget = spd;
	accelT = 0.0f;
	accelTmax = Tmax;
}

#if 0 //del by k.nakagawa ��Ԃ��Ƃ̃Q�C���ύX���폜
void LineTraceInsideBonusSmc_ChangeDirGain(float k)
{
	dirKp = BALANCERUN_DIRECTION_KP * k;
	dirKi = BALANCERUN_DIRECTION_KI * k;
	dirKd = BALANCERUN_DIRECTION_KD * k;
}
#endif

// IN�R�[�X�㔼(�{�[�i�X�R�[�X)�V�[�P���X
// seqNo : �V�[�P���X�ԍ�
// distanceAbs : �J�n����̐�΋���[m]
// courseJointDetected : �R�[�X�p���ڌ��m�o��
// distanceCorrection : �J�n����̐�΋����␳�l�o��
int LineTraceInsideBonusSmc_Smc(UINT seqNo, float distanceAbs, int* courseJointDetected, float* distanceCorrection)
{
	static int onMarkerCounter = 0;
	static float distanceAbsMarkerStart = 0.0f;
	static float initDistance = 0.0f;

	UINT seqNoNext = seqNo;

#if 0
	U16 black = LineTraceControl_LightBlackThreshold();
	U16 white = LineTraceControl_LightWhiteThreshold();
#endif

	// �K��PID�Ƃ肠�����K��
	float tailKp = TAIL_KP;
	float tailKi = TAIL_KI;
	float tailKd = TAIL_KD;
	float tailMaxMv = 100.0f;
	float tailMinMv = -100.0f;


	UINT str = 0;
	float mumu;	// �}�[�J�[���o�̂��߂̕ϐ�

	switch(seqNo)
	{
	case LT_IN2_SEQ_00: //�Ō�̒����ɓ�������ɁA���x�𒆑��ɗ��Ƃ��āA�}�[�J�[�T���V�[�P���X�ɓ��鏀��

		if(seqNoPrv != seqNo){
			LineTraceInsideBonusSmc_ChangeSpeed(40, 0.3f);
			initDistance = distanceAbs;
		}

		// ���肷��܂�0.10m���C���g���[�X���s
		if(distanceAbs - initDistance > 0.10f)
		{
			seqNoNext = LT_IN2_SEQ_00_1;
		}

		break;
	case LT_IN2_SEQ_00_1: // �����ŁA�}�[�J�[�T�����A�t�B�M���AL�O�}�[�J�[�O�[�����o

		mumu = get_model_mu();
		if(mumu<-0.04f || 0.04f<mumu)
		{
			distanceAbsMarkerStart = distanceAbs;
			change_model_black_to_gray();
			onMarker = 1;
			seqNoNext = LT_IN2_SEQ_01;
		}
		break;
	case LT_IN2_SEQ_01: //�t�B�M���AL�O�}�[�J�[���m�� �K�����s�ֈڍs
	{
		if(seqNoPrv != seqNo)
		{
			LineTraceInsideBonusSmc_ChangeSpeed(10, 0.1f);
			BalanceToTail_Init();
			onMarkerCounter = 0;
		}

		if(BalanceToTailMain() == 1)
		{
			seqNoNext = LT_IN2_SEQ_02;
		}
		break;
	}
	case LT_IN2_SEQ_02: //�}�[�J�[�������C���g���[�X
	{
		if(seqNoPrv != seqNo)
		{
			LineTraceInsideBonusSmc_ChangeSpeed(30, 0.5f);
			initDistance = distanceAbs;
			change_model_black_to_gray();
			reset_model_state();
		}

		tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv);

		if(distanceAbs - initDistance > 0.02f)
		{
			seqNoNext = LT_IN2_SEQ_02_1;
		}
		break;
	}
	case LT_IN2_SEQ_02_1: //�t�B�M���AL�J�n�܂�
	{

		tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv);

		mumu = get_model_mu();
		if(mumu<-0.04f || 0.04f<mumu)
		{
			change_model_gray_to_black();
			onMarker = 2;
			*courseJointDetected = IN_COURSE_PART_ON_FIGUREL;
			seqNoNext = LT_IN2_SEQ_03;
		}
		break;
	}
	case LT_IN2_SEQ_03: //�S�[���O�}�[�J�[�������C���g���[�X���s
		// kunitake �����֓��������_�Ń}�[�J�[���ł���Ɖ��聚�m�F���邱��
		if(seqNoPrv != seqNo)
		{
			//LineTraceInsideBonusSmc_ChangeSpeed(30, 0.3f);
			LineTraceInsideBonusSmc_ChangeSpeed(20, 0.3f);
			onMarker = 1;
			initDistance = distanceAbs;
			change_model_black_to_gray();
			reset_model_state();
		}

		tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv);

		if(distanceAbs - initDistance > 0.02f)
		{
			seqNoNext = LT_IN2_SEQ_04;
		}
		break;
	case LT_IN2_SEQ_04: // �}�[�J�[���𑖍s�A�}�[�J�[�I�[�����m�O
	{
		tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv);

		mumu = get_model_mu();
		if(mumu<-0.04f || 0.04f<mumu)
		{
			change_model_gray_to_black();
			seqNoNext = LT_IN2_SEQ_05;
		}
		break;
	}
	case LT_IN2_SEQ_05: //����
	{
		static float initDistance;
		if(seqNoPrv != seqNo)
		{
			initDistance = distanceAbs;
			reset_model_state();
		}

		tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv);

		//if(distanceAbs - initDistance > 0.25f)
		if(distanceAbs - initDistance > 0.30f)
		{
			seqNoNext = LT_IN2_SEQ_06;
		}
//		ecrobot_debug1((UINT)seqNo, (UINT)(distanceAbs * 1000.0f), onMarker);
		break;
	}
	case LT_IN2_SEQ_06: //�S�[��
	{
		TailRunControl(0, 0);
		tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv);
		break;
	}

	default:
		#if 1
		// ���̏��~���������Ȃ��ꍇ
		#else
		// ���̂܂ܐi�ޏꍇ�i�f�[�^��肵�����ꍇ�j
		str = 1;
		#endif
		// ecrobot_debug1((UINT)seqNo, (UINT)(distanceAbs * 1000.0f), onMarker);
		break;
	}

	forward = (signed char)SCurve((float)forwardTarget, (float)forwardTargetPrev, accelTmax, &accelT);
	forward = (S8)(forward * Kspd);	// debug�p�␳

	// SEQ��ԃ^�C���v��
	//if(seqNo != seqNoPrv) SaveSeqEndTime(seqNoPrv, mT, distanceAbs);


	// ���s�̐���
	//TurnRecord(turn); //turn�l���L�^���邽�߂ɁA���䖈�ɕK���ĂԂ���
	if(str) turn = 0;

	// 2013/07/17 mod by s.sahara �K�����s�֎~�ɂȂ����� >>>>
	if(seqNo < LT_IN2_SEQ_01)
	{
		if(1 == onMarker && onMarkerCounter <= 50)
		{
			BalanceControlStraightP(forward, 3.0f);
		}
		else
		{
#ifdef USE_LOG
//			ecrobot_bt_data_logger((S8)seqNo+10, (S8)(forward)); // �}�[�J���m臊m�F�p
#endif
			// ������PID
			ctrl_direction((float)forward);
			turn = get_model_turn();
			forward = get_model_forward();
			BalanceControl(forward, turn);
		}
	}
	else if(seqNo == LT_IN2_SEQ_02 || seqNo == LT_IN2_SEQ_02_1 || seqNo == LT_IN2_SEQ_03 || seqNo == LT_IN2_SEQ_04 || seqNo == LT_IN2_SEQ_05)
	{
		// ������PID
		ctrl_direction((float)forward);
		turn = get_model_turn();
		forward = get_model_forward();
		TailRunControl(forward, turn);
	}


	//BalanceControl(forward, turn);
	//TailRunControl(forward, turn);
	// 2013/07/17 mod by s.sahara �K�����s�֎~�ɂȂ����� <<<<

#ifdef USE_LOG
	//ecrobot_bt_data_logger((S8)(seqNo), (S8)(integral)); // �}�[�J���m臊m�F�p
	//ecrobot_bt_data_logger((S8)(md_mid_lightValue - 600), (S8)(md)); // �}�[�J���m臊m�F�p
	//ecrobot_bt_data_logger((S8)(md_mid_lightValue - 600), (S8)(md_slow_lightValue - 600)); // �}�[�J���m臊m�F�p
	//ecrobot_bt_data_logger((S8)(lightValue*0.1), (S8)(md_mid_lightValue*0.1));
	//ecrobot_bt_data_logger((S8)seqNo, (S8)turn);
	ecrobot_bt_data_logger((S8)seqNo+10, (S8)(get_model_mu()*1000)); // �}�[�J���m臊m�F�p
	//ecrobot_bt_data_logger((S8)seqNo+10, (S8)(get_model_y()*10)); // �}�[�J���m臊m�F�p
	//ecrobot_bt_data_logger((S8)seqNo+10, (S8)(get_model_center()-500)); // �}�[�J���m臊m�F�p
	//ecrobot_bt_data_logger((S8)(get_model_mu()*1000), (S8)(get_model_light()-500)); // �}�[�J���m臊m�F�p
	//ecrobot_bt_data_logger((S8)seqNo, (S8)forward);
#endif
	//LineTraceInsideBasicSmc_logTurn(turn); // PU����p

	//mT++;
	seqNoPrv = seqNo;
	return seqNoNext;
}

