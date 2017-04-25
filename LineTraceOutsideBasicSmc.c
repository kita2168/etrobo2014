// 2012/08/14 created by s.sahara

// OUT���x�[�V�b�N���s��ԑJ�ڃ}�V��

// 2013/07/17�`27 mod by s.sahara �x�[�V�b�N�X�e�[�W�ł͐K�����s���֎~�ɂȂ����ׁA�K�����s�֘A�̏�����|�����s�ɕύX�܂��͍폜

#include "LineTraceOutsideBasicSmc.h"

#include "SensorIn.h"
#include "Control.h"

#include "LineTraceControl.h"
#include "LineTraceParam.h"

#include "CourseDef.h"

#define abs(X)	((X) < 0 ? -(X) : (X))

// SEQ��ԃ^�C������
#if 0
static UINT mT = 0;
static S8 mSeqTime[LT_OUT1_SEQ_MAX];
static S8 mSeqDist[LT_OUT1_SEQ_MAX];
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


void LineTraceOutsideBasicSmc_Init(signed char arg_forward)
{
#if 0
	U16 black = LineTraceControl_LightBlackThreshold();
	U16 white = LineTraceControl_LightWhiteThreshold();
#endif

	onMarker = 0;
//	onMarkerPrev = 0;

#if 0
	mT = 0;
	for(int i=0;i<LT_OUT1_SEQ_MAX;i++)
	{
		mSeqTime[i] = 0;
		mSeqDist[i] = 0;
	}
#endif

	forwardTarget = arg_forward; // �����ڕW���x�w��
	forwardTargetPrev = arg_forward;

	// �O��΍��ێ��l�̏�����
	hensa0 = 0.0f;
	integral = 0;

	seqNoPrv = -1;

	init_model();
}

signed char LineTraceOutsideBasicSmc_GetForward()
{
	return forward;
}

#if 0
// ��PU����p
void LineTraceOutsideBasicSmc_logTurn(signed char turn)
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

void LineTraceOutsideBasicSmc_BTSendLapTime()
{
	for(int i=0;i<LT_OUT1_SEQ_MAX;i++)
	{
#ifdef USE_LOG
		ecrobot_bt_data_logger(mSeqDist[i], mSeqTime[i]);
#endif
		systick_wait_ms(10);
	}
}
#endif

void LineTraceOutsideBasicSmc_ChangeSpeed(float spd, float Tmax)
{
	forwardTargetPrev = forward;
	forwardTarget = spd;
	accelT = 0.0f;
	accelTmax = Tmax;
}

#if 0 //del by k.nakagawa ��Ԃ��Ƃ̃Q�C���ύX���폜
void LineTraceOutsideBasicSmc_ChangeDirGain(float k)
{
	dirKp = BALANCERUN_DIRECTION_KP * k;
	dirKi = BALANCERUN_DIRECTION_KI * k;
	dirKd = BALANCERUN_DIRECTION_KD * k;
}
#endif

// OUT�R�[�X�O��(�x�[�V�b�N�R�[�X)�V�[�P���X
// seqNo : �V�[�P���X�ԍ�
// distanceAbs : �J�n����̐�΋���[m]
// courseJointDetected : �R�[�X�p���ڌ��m�o��
// distanceCorrection : �J�n����̐�΋����␳�l�o��
int LineTraceOutsideBasicSmc_Smc(UINT seqNo, float distanceAbs, int* courseJointDetected, float* distanceCorrection)
{
#if 0 //del by k.nakagawa �}�[�J�[���m�p��`�̍폜
	static UINT lastStraightdetected;
	//static float distanceAbsMarkerStart = 0.0f;

	U16 black = LineTraceControl_LightBlackThreshold();
	U16 white = LineTraceControl_LightWhiteThreshold();

	//U16 lightValue = GetLightIn3();
	//U16 lightValue = GetLightInLPF();
	U16 md_mid_lightValue = md_mid_GetLightInLPF();
	U16 md_slow_lightValue = md_slow_GetLightInLPF();
	float md = ((float)(md_mid_lightValue) / (float)(md_slow_lightValue) - 1.0f) * 1000.0f;
#endif

#if 0 //del by k.nakagawa ���S�@�\�̍폜
	U8 autoBrake = false;
#endif

	UINT str = 0;
	UINT seqNoNext = seqNo;

	switch(seqNo)
	{
	case LT_OUT1_SEQ_00:
		#if 1
		//autoBrake = true;
		if(seqNoPrv != seqNo) LineTraceOutsideBasicSmc_ChangeSpeed(100, 0.3f);
		#else
		if(seqNoPrv != seqNo) LineTraceOutsideBasicSmc_ChangeSpeed(BALANCERUN_FORWARD_SPEED, 0.3f);
		#endif

		if(distanceAbs > OUT1_NORMAL_DISTANCE)
		{
			seqNoNext = LT_OUT1_SEQ_11;
		}
		//ecrobot_debug1((UINT)seqNo, (UINT)(distanceAbs * 1000.0f), 0);
		break;

	case LT_OUT1_SEQ_11:
		*courseJointDetected = OUT_COURSE_PART_BEFORE_LOOKUPGATE;
		ecrobot_debug1((UINT)seqNo, (UINT)(distanceAbs * 1000.0f), onMarker);
		break;

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

#if 0 //del by k.nakagawa ���S�@�\�̍폜
	// 2013/09/21 add by s.sahara ���S�@�\
	if(autoBrake)
	{
		#if 0 // 2013/09/21���_�A���C����E�������u���[�L�݂̂ŏ\��
		// �Ȑ����s�����u���[�L
		static float integralDirAng;
		float dirAngle = LinTraceControl_GetCurveAngle();
		float Kspdc = 1.0f - ((dirAngle / 90.0f) * (dirAngle / 90.0f) * integralDirAng);
		integralDirAng += (dirAngle / 90.0f) * (dirAngle / 90.0f); // ��R
		integralDirAng *= 0.975f;	// �Q��
		if(Kspdc < 0.1f) Kspdc = 0.1f; // �O�a
		#endif

		// ���C����E�������u���[�L
		float lightnorm = abs(GetNormalizedLightLevel()) * 0.01f;
		if(lightnorm <= 1.0f) lightnorm = 0.0f;
		else lightnorm -= 1.0f;
		float Kspdl = 1.0f - lightnorm * lightnorm * 0.5f;
		if(Kspdl < 0.2f) Kspdl = 0.2f; // �O�a

		//forward = (S8)(forward * Kspdc * Kspdl);
		forward = (S8)(forward * 1.0f * Kspdl);
		//ecrobot_debug1((UINT)(lightnorm*100), forward, (UINT)(Kspdl*100));
	}
#endif

	// SEQ��ԃ^�C���v��
	//if(seqNo != seqNoPrv) SaveSeqEndTime(seqNoPrv, mT, distanceAbs);

	// ������PID
#if 0
	turn = LineTraceControl_DirectionPID(dirKp, dirKi, dirKd, &hensa0, &integral, BALANCERUN_DIRECTION_MV_MAX, BALANCERUN_DIRECTION_MV_MIN);
	//ecrobot_debug1((UINT)seqNo, (UINT)(distanceAbs * 1000.0f), onMarker);
#else
	ctrl_direction((float)forward);
	turn = get_model_turn();
	forward = get_model_forward();
#endif

	// ���s�̐���
	//TurnRecord(turn); //turn�l���L�^���邽�߂ɁA���䖈�ɕK���ĂԂ���
	if(str) turn = 0;
	//if(onMarker) turn -= 1; // �E�ɍs���̂ō�

	BalanceControl(forward, turn);

#ifdef USE_LOG
	//ecrobot_bt_data_logger((S8)(seqNo), (S8)(integral)); // �}�[�J���m臊m�F�p
	//ecrobot_bt_data_logger((S8)(md_mid_lightValue - 600), (S8)(md)); // �}�[�J���m臊m�F�p
	//ecrobot_bt_data_logger((S8)(md_mid_lightValue - 600), (S8)(md_slow_lightValue - 600)); // �}�[�J���m臊m�F�p
	//ecrobot_bt_data_logger((S8)(lightValue*0.1), (S8)(md_mid_lightValue*0.1));
	//ecrobot_bt_data_logger((S8)seqNo, (S8)turn);
	//ecrobot_bt_data_logger((S8)seqNo, (S8)forward);
#endif
	//LineTraceInsideBasicSmc_logTurn(turn); // PU����p

	//mT++;
	seqNoPrv = seqNo;
	return seqNoNext;
}

