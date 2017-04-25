// 2012/08/13 created by s.sahara

//#ifdef USE_SAMPLE_COURCE

// �e�X�g���s��ԑJ�ڃ}�V��

#include "LineTraceTestSmc.h"

#include "SensorIn.h"
#include "Control.h"

#include "LineTraceControl.h"
#include "LineTraceParam.h"

#include "CourseDef.h"

// SEQ��ԃ^�C������
static UINT mT = 0;
static S8 mSeqTime[LT_TEST_SEQ_MAX];
static S8 mSeqDist[LT_TEST_SEQ_MAX];

static signed char forward;		// �O��i����
static signed char turn;		// ���񖽗�

static signed char forwardTarget = 0;		// �ڕW���x�w��
static signed char forwardTargetPrev = 0;		// �O��ڕW���x�w��
static float accelT = 0.0f;
static float accelTmax = 0;

static char onMarker = 0;
static char onMarkerPrev = 0;

static float hensa0;
static float integral;

static int seqNoPrv = -1;

//#################################################################
// kuni���C���g���[�X�p�̊֐��̐錾 add by kunitake <2014/08/13>
//#################################################################
extern void init_model();
extern void ctrl_direction(float ref_vel);
extern signed char get_model_forward();
extern signed char get_model_turn();

void LineTraceTestSmc_Init(signed char forward)
{
	U16 black = LineTraceControl_LightBlackThreshold();
	U16 white = LineTraceControl_LightWhiteThreshold();

	onMarker = 0;
	onMarkerPrev = 0;

	mT = 0;
	for(int i=0;i<LT_TEST_SEQ_MAX;i++)
	{
		mSeqTime[i] = 0;
		mSeqDist[i] = 0;
	}

	forwardTarget = forward; // �����ڕW���x�w��
	forwardTargetPrev = forward;

	#ifdef TEST_SLOW_FORWARD_RUN
	forward = 15;
	tail_control(0);
	#endif

	// �O��΍��ێ��l�̏�����
	hensa0 = 0.0f;
	integral = 0;

	seqNoPrv = -1;

	init_model();
}

#if 0
// ��PU����p
void LineTraceTestSmc_logTurn(signed char turn)
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
	ecrobot_bt_data_logger(turn, (S8)integralTurn);
#endif
	ecrobot_debug1((U16)turn+100.0f, (U16)integral, (U16)(integral * -1));
}

void SaveSeqEndTime(UINT seqNo, UINT absTime, float distanceAbs)
{
	mSeqTime[seqNo] = (S8)(absTime * 0.004 * 10.0); // [1/10sec]
	mSeqDist[seqNo] = (S8)(distanceAbs * 10.0);	// [1/10seq]
}
#endif

void LineTraceTestSmc_BTSendLapTime()
{
	for(int i=0;i<LT_TEST_SEQ_MAX;i++)
	{
#ifdef USE_LOG
		//ecrobot_bt_data_logger(mSeqDist[i], mSeqTime[i]);
#endif
		systick_wait_ms(10);
	}
}

void LineTraceTestSmc_ChangeSpeed(float spd, float Tmax)
{
	forwardTargetPrev = forward;
	forwardTarget = spd;
	accelT = 0.0f;
	accelTmax = Tmax;
}

// 2012/08/12 add by s.sahara >>>>
// seqNo : �V�[�P���X�ԍ�
// distanceAbs : �J�n����̐�΋���[m]
// courseJointDetected : �R�[�X�p���ڌ��m�o��
// distanceCorrection : �J�n����̐�΋����␳�l�o��
int LineTraceTestSmc_Smc(UINT seqNo, float distanceAbs, int* courseJointDetected, float* distanceCorrection)
{
	static UINT lastStraightdetected;
	static float distanceAbsMarkerStart = 0.0f;
	static float distanceAbs_MarkerDetected = 0.0f;

	UINT seqNoNext = seqNo;

	U16 black = LineTraceControl_LightBlackThreshold();
	U16 white = LineTraceControl_LightWhiteThreshold();

	float Kspd = 1.0f;
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

	//U16 lightValue = GetLightIn3();
	U16 lightValue = GetLightInLPF();
	U16 md_mid_lightValue = md_mid_GetLightInLPF();
	U16 md_slow_lightValue = md_slow_GetLightInLPF();
	float md = ((float)(md_mid_lightValue) / (float)(md_slow_lightValue) - 1.0f) * 1000.0f;

	UINT str = 0;

	switch(seqNo)
	{
	case 0: // �}�[�J���m
		if(seqNoPrv != seqNo) LineTraceTestSmc_ChangeSpeed(TAILRUN_FORWARD_SPEED * 0.2f, 0.3f);
		tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv);

		if(!onMarker)
		{
			if(forward > LOWSPEED_MARKER_DETECTION__SPD)
			{
			}
			else
			{
				forward = LOWSPEED_MARKER_DETECTION__SPD;
			}
			if(lightValue < LOWSPEED_MARKER_DETECTION__LIGHT_THRESHOLD)
			{
				distanceAbs_MarkerDetected = distanceAbs;
				Kstr = 0.1f;
				dirKp = TAILRUN_DIRECTION_KP * Kstr; dirKi = TAILRUN_DIRECTION_KI * Kstr; dirKd = TAILRUN_DIRECTION_KD * Kstr; // �}�[�J�i������̓Q�C�������߂�
				LineTraceControl_ChangeLightThreshold((white + black) / 2, white, &hensa0, &integral); // �O���[/��
				onMarker = 1;
			}
		}
		else
		{
			// �}�[�J�i�����̓Q�C�����߂�i�}�[�J�i�����E�ɂ����ׁj
			//if(distanceAbs < (LT_IN1_SEQ_08_DIST + MARKER_LENGTH - 0.10f))
			//{
			//	Kstr = 0.6f;
			//	//Kstr = 0.4f; // �シ��
			//	dirKp = DIRECTION_KP * Kstr; dirKi = DIRECTION_KI * Kstr; dirKd = DIRECTION_KD * Kstr;
			//}
			// ��0.8�ł��߂肪�x��
			//else if(distanceAbs < (LT_IN1_SEQ_08_DIST + MARKER_LENGTH - 0.05f))
			//{
			//	Kstr = 0.8f;
			//	dirKp = DIRECTION_KP * Kstr; dirKi = DIRECTION_KI * Kstr; dirKd = DIRECTION_KD * Kstr;
			//}
			if(forward > LOWSPEED_MARKER_DETECTION__SPD) // ���w�A�s���Ȃ̂Œᑬ�܂Ō���
			{
			}
			if((distanceAbs - distanceAbs_MarkerDetected) > 0.1f) // �}�[�J�i�������҂��i�������u�Ԃ͉E�������ߌ����̓��x�����オ���Ă���j
			{
				// �O���[����
				if(lightValue > LOWSPEED_MARKER_DETECTION__LIGHT_THRESHOLD)
				{
					LineTraceControl_ChangeLightThreshold(black, white, &hensa0, &integral); // ��/��
					onMarker = 0;
					// seqNo++;
				}
			}
			else
			{
				Kstr = 0.4f;
				dirKp = TAILRUN_DIRECTION_KP * Kstr; dirKi = TAILRUN_DIRECTION_KI * Kstr; dirKd = TAILRUN_DIRECTION_KD * Kstr; // �}�[�J�i������̓Q�C�������߂�
			}
		}

		ecrobot_debug1((UINT)seqNo, (UINT)(distanceAbs * 1000.0f), onMarker);
		break;
	case 1:
		if(seqNoPrv != seqNo) LineTraceTestSmc_ChangeSpeed(TAILRUN_FORWARD_SPEED * 0.4f, 0.3f);
		tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv);

		if(distanceAbs > (distanceAbs_MarkerDetected + 0.2f))
		{
			seqNo = 0;
		}
		ecrobot_debug1((UINT)seqNo, (UINT)(distanceAbs * 1000.0f), 0);
		break;

	case 2:
		if(seqNoPrv != seqNo) LineTraceTestSmc_ChangeSpeed(0, 0.3f);
		tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv);
		break;

	case 100: // �P�Ȃ�|���ᑬ�O�i
		if(seqNoPrv != seqNo) LineTraceTestSmc_ChangeSpeed(20, 0.3f);
		turn = 0;
		BalanceControl1(forward, turn, 0);
		ecrobot_debug1(forward, 0, 0);
		break;

	//... USE_SAMPLE_COURCE�͂����������s
	case 200: // ���C���g���[�XPID�Q�C�������p�i�K�����s��max���s����̂݁j
//...	ecrobot_sound_tone(100, 200, 30);	//... �f�o�b�O�p�T�E���h

		if(seqNoPrv != seqNo)
		{
			//ecrobot_sound_tone(100, 200, 30);	//... �f�o�b�O�p�T�E���h
			LineTraceTestSmc_ChangeSpeed(TAILRUN_FORWARD_SPEED, 1.0f);
		}
		//... �K�����グ���܂܂ɂ��邽�߃R�����g�A�E�g	tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv);
		break;
	}

	forward = (signed char)SCurve((float)forwardTarget, (float)forwardTargetPrev, accelTmax, &accelT);
	forward = (S8)(forward * Kspd);	// debug�p�␳

	// SEQ��ԃ^�C���v��
	//if(seqNo != seqNoPrv) SaveSeqEndTime(seqNoPrv, mT, distanceAbs);

#if 0
	#ifndef TEST_SLOW_FORWARD_RUN
	// ������PID
	turn = LineTraceControl_DirectionPID(dirKp, dirKi, dirKd, &hensa0, &integral, TAILRUN_DIRECTION_MV_MAX, TAILRUN_DIRECTION_MV_MIN);
#ifdef USE_LOG
	ecrobot_bt_data_logger((S8)(hensa0), (S8)(integral*0.1f));
#endif
	//ecrobot_debug1((UINT)seqNo, (UINT)(distanceAbs * 1000.0f), onMarker);
	//ecrobot_debug1((UINT)seqNo, (UINT)(turn + 100), (S8)(lightValue*0.1));

	// ���s�̐���
	//... �o�����X���s�̂��߃R�����g�A�E�g TailRunControl(forward, turn);
	BalanceControl1(forward, turn, 0);	//... �o�����X���s�̂��ߒǉ�

#ifdef USE_LOG
	//ecrobot_bt_data_logger(turn, (S8)(lightValue*0.1));
	//ecrobot_bt_data_logger((lightValue*0.1), (S8)(lightValueLpf*0.1));
	//ecrobot_bt_data_logger((S8)seqNo, (S8)turn);
#endif
	//LineTraceTestSmc_logTurn(turn); // PU����p
	#endif // #ifndef TEST_SLOW_FORWARD_RUN
#else
	ctrl_direction((float)forward);
	turn = get_model_turn();
	forward = get_model_forward();
	BalanceControl1(forward, turn, 0);	//... �o�����X���s�̂��ߒǉ�
#endif



	mT++;
	seqNoPrv = seqNo;
	return seqNoNext;
}
// 2012/08/12 add by s.sahara <<<<

//#endif // #ifdef USE_SAMPLE_COURCE
